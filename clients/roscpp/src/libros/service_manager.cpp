/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cstdio>
#include "ros/service_manager.h"
#include "ros/xmlrpc_manager.h"
#include "ros/connection_manager.h"
#include "ros/poll_manager.h"
#include "ros/service_publication.h"
#include "ros/service_client_link.h"
#include "ros/service_server_link.h"
#include "ros/this_node.h"
#include "ros/network.h"
#include "ros/master.h"
#include "ros/transport/transport_tcp.h"
#include "ros/transport/transport_udp.h"
#include "ros/transport/transport_uds_stream.h"
#include "ros/init.h"
#include "ros/connection.h"
#include "ros/file_log.h"

#include "xmlrpcpp/XmlRpc.h"

#include <ros/console.h>

using namespace XmlRpc; // A battle to be fought later
using namespace std; // sigh

namespace ros
{

const ServiceManagerPtr& ServiceManager::instance()
{
  static ServiceManagerPtr service_manager = boost::make_shared<ServiceManager>();
  return service_manager;
}

ServiceManager::ServiceManager()
: shutting_down_(false)
{
}

ServiceManager::~ServiceManager()
{
  shutdown();
}

void ServiceManager::start()
{
  shutting_down_ = false;

  poll_manager_ = PollManager::instance();
  connection_manager_ = ConnectionManager::instance();
  xmlrpc_manager_ = XMLRPCManager::instance();

}

void ServiceManager::shutdown()
{
  boost::recursive_mutex::scoped_lock shutdown_lock(shutting_down_mutex_);
  if (shutting_down_)
  {
    return;
  }

  shutting_down_ = true;

  ROSCPP_LOG_DEBUG("ServiceManager::shutdown(): unregistering our advertised services");
  {
    boost::mutex::scoped_lock ss_lock(service_publications_mutex_);

    for (L_ServicePublication::iterator i = service_publications_.begin();
         i != service_publications_.end(); ++i)
    {
      unregisterService((*i)->getName());
      //ROSCPP_LOG_DEBUG( "shutting down service %s", (*i)->getName().c_str());
      (*i)->drop();
    }
    service_publications_.clear();
  }

  L_ServiceServerLink local_service_clients;
  {
    boost::mutex::scoped_lock lock(service_server_links_mutex_);
    local_service_clients.swap(service_server_links_);
  }

  {
    L_ServiceServerLink::iterator it = local_service_clients.begin();
    L_ServiceServerLink::iterator end = local_service_clients.end();
    for (; it != end; ++it)
    {
      (*it)->getConnection()->drop(Connection::Destructing);
    }

    local_service_clients.clear();
  }

}

bool ServiceManager::advertiseService(const AdvertiseServiceOptions& ops)
{
  boost::recursive_mutex::scoped_lock shutdown_lock(shutting_down_mutex_);
  if (shutting_down_)
  {
    return false;
  }

  {
    boost::mutex::scoped_lock lock(service_publications_mutex_);

    if (isServiceAdvertised(ops.service))
    {
      ROS_ERROR("Tried to advertise a service that is already advertised in this node [%s]", ops.service.c_str());
      return false;
    }

    ServicePublicationPtr pub(boost::make_shared<ServicePublication>(ops.service, ops.md5sum, ops.datatype, ops.req_datatype, ops.res_datatype, ops.helper, ops.callback_queue, ops.tracked_object));
    service_publications_.push_back(pub);
  }

  XmlRpcValue args, result, payload;
  XmlRpcValue uds_args;
  args[0] = this_node::getName();
  args[1] = ops.service;
  char uri_buf[1024];
  snprintf(uri_buf, sizeof(uri_buf), "rosrpc://%s:%d",
           network::getHost().c_str(), connection_manager_->getTCPPort());
  args[2] = string(uri_buf);

  uds_args = args;
  snprintf(uri_buf, sizeof(uri_buf), "rosrpc://%s",
           connection_manager_->getUDSStreamPath().c_str());
  uds_args[3] = string(uri_buf);

  args[3] = xmlrpc_manager_->getServerURI();
  uds_args[4] = xmlrpc_manager_->getServerURI();
  if (!master::execute("registerServiceExt", uds_args, result, payload, true))
  {
    master::execute("registerService", args, result, payload, true);
  }

  return true;
}

bool ServiceManager::unadvertiseService(const string &serv_name)
{
  boost::recursive_mutex::scoped_lock shutdown_lock(shutting_down_mutex_);
  if (shutting_down_)
  {
    return false;
  }

  ServicePublicationPtr pub;
  {
    boost::mutex::scoped_lock lock(service_publications_mutex_);

    for (L_ServicePublication::iterator i = service_publications_.begin();
         i != service_publications_.end(); ++i)
    {
      if((*i)->getName() == serv_name && !(*i)->isDropped())
      {
        pub = *i;
        service_publications_.erase(i);
        break;
      }
    }
  }

  if (pub)
  {
    unregisterService(pub->getName());
    ROSCPP_LOG_DEBUG( "shutting down service [%s]", pub->getName().c_str());
    pub->drop();
    return true;
  }

  return false;
}

bool ServiceManager::unregisterService(const std::string& service)
{
  XmlRpcValue args, result, payload;
  XmlRpcValue uds_args;
  args[0] = this_node::getName();
  args[1] = service;
  char uri_buf[1024];
  snprintf(uri_buf, sizeof(uri_buf), "rosrpc://%s:%d",
           network::getHost().c_str(), connection_manager_->getTCPPort());
  args[2] = string(uri_buf);

  uds_args = args;
  snprintf(uri_buf, sizeof(uri_buf), "rosrpc://%s",
           connection_manager_->getUDSStreamPath().c_str());
  uds_args[3] = string(uri_buf);

  if (!master::execute("unregisterServiceExt", uds_args, result, payload, false))
  {
    master::execute("unregisterService", args, result, payload, false);
  }

  return true;
}

bool ServiceManager::isServiceAdvertised(const string& serv_name)
{
  for (L_ServicePublication::iterator s = service_publications_.begin(); s != service_publications_.end(); ++s)
  {
    if (((*s)->getName() == serv_name) && !(*s)->isDropped())
    {
      return true;
    }
  }

  return false;
}

ServicePublicationPtr ServiceManager::lookupServicePublication(const std::string& service)
{
  boost::mutex::scoped_lock lock(service_publications_mutex_);

  for (L_ServicePublication::iterator t = service_publications_.begin();
       t != service_publications_.end(); ++t)
  {
    if ((*t)->getName() == service)
    {
      return *t;
    }
  }

  return ServicePublicationPtr();
}

ServiceServerLinkPtr ServiceManager::createServiceServerLink(const std::string& service, bool persistent,
                                             const std::string& request_md5sum, const std::string& response_md5sum,
                                             const M_string& header_values)
{

  boost::recursive_mutex::scoped_lock shutdown_lock(shutting_down_mutex_);
  if (shutting_down_)
  {
    return ServiceServerLinkPtr();
  }

  uint32_t serv_port;
  std::string serv_host;
  std::string serv_uds_path;
  ConnectionPtr connection;
  bool connect_ret = false;

  if (!lookupServiceExt(service, serv_host, serv_port, serv_uds_path))
  {
    return ServiceServerLinkPtr();
  }

  bool is_internal = network::isInternal(serv_uds_path, serv_host);
  if (is_internal)
  {
    TransportUDSStreamPtr transport(boost::make_shared<TransportUDSStream>(&poll_manager_->getPollSet()));

    // Make sure to initialize the connection *before* transport->connect()
    // is called, otherwise we might miss a connect error (see #434).
    connection = boost::make_shared<Connection>();
    connection_manager_->addConnection(connection);
    connection->initialize(transport, false, HeaderReceivedFunc());

    if (transport->connect(serv_uds_path))
    {
      connect_ret = true;
    }
  }
  else
  {
    TransportTCPPtr transport(boost::make_shared<TransportTCP>(&poll_manager_->getPollSet()));

    // Make sure to initialize the connection *before* transport->connect()
    // is called, otherwise we might miss a connect error (see #434).
    connection = boost::make_shared<Connection>();
    connection_manager_->addConnection(connection);
    connection->initialize(transport, false, HeaderReceivedFunc());

    if (transport->connect(serv_host, serv_port))
    {
      connect_ret = true;
    }
  }

  if (connect_ret)
  {
    ServiceServerLinkPtr client(boost::make_shared<ServiceServerLink>(service, persistent, request_md5sum, response_md5sum, header_values));

    {
      boost::mutex::scoped_lock lock(service_server_links_mutex_);
      service_server_links_.push_back(client);
    }

    client->initialize(connection);

    return client;
  }

  if (is_internal)
  {
    ROSCPP_LOG_DEBUG("Failed to connect to service [%s] (mapped=[%s]) at [%s]", service.c_str(), service.c_str(), serv_uds_path.c_str());
  }
  else
  {
    ROSCPP_LOG_DEBUG("Failed to connect to service [%s] (mapped=[%s]) at [%s:%d]", service.c_str(), service.c_str(), serv_host.c_str(), serv_port);
  }

  return ServiceServerLinkPtr();
}

void ServiceManager::removeServiceServerLink(const ServiceServerLinkPtr& client)
{
  // Guard against this getting called as a result of shutdown() dropping all connections (where shutting_down_mutex_ is already locked)
  if (shutting_down_)
  {
    return;
  }

  boost::recursive_mutex::scoped_lock shutdown_lock(shutting_down_mutex_);
  // Now check again, since the state may have changed between pre-lock/now
  if (shutting_down_)
  {
    return;
  }

  boost::mutex::scoped_lock lock(service_server_links_mutex_);

  L_ServiceServerLink::iterator it = std::find(service_server_links_.begin(), service_server_links_.end(), client);
  if (it != service_server_links_.end())
  {
    service_server_links_.erase(it);
  }
}

bool ServiceManager::lookupService(const string &name, string &serv_host, uint32_t &serv_port)
{
  XmlRpcValue args, result, payload;
  args[0] = this_node::getName();
  args[1] = name;
  if (!master::execute("lookupService", args, result, payload, false))
    return false;

  string serv_uri(payload);
  if (!serv_uri.length()) // shouldn't happen. but let's be sure.
  {
    ROS_ERROR("lookupService: Empty server URI returned from master");

    return false;
  }

  if (!network::splitURI(serv_uri, serv_host, serv_port))
  {
    ROS_ERROR("lookupService: Bad service uri [%s]", serv_uri.c_str());

    return false;
  }

  return true;
}

bool ServiceManager::lookupServiceExt(const string &name, string &serv_host, uint32_t &serv_port, std::string& serv_uds_path)
{
  XmlRpcValue args, result, payload, uds_payload;
  args[0] = this_node::getName();
  args[1] = name;
  if (!master::execute("lookupServiceExt", args, result, uds_payload, false))
  {
    if (!master::execute("lookupService", args, result, payload, false))
    {
      return false;
    }
  }

  string serv_uri;
  string serv_uds_uri;

  if (uds_payload.valid() && uds_payload.size() != 0)
  {
    serv_uri = std::string(uds_payload[0]);
    serv_uds_uri = std::string(uds_payload[1]);
  }
  else
  {
    serv_uri = std::string(payload);
  }

  if (!serv_uri.length()) // shouldn't happen. but let's be sure.
  {
    ROS_ERROR("lookupService: Empty server URI returned from master");

    return false;
  }

  if (!network::splitURI(serv_uri, serv_host, serv_port))
  {
    ROS_ERROR("lookupService: Bad service uri [%s]", serv_uri.c_str());

    return false;
  }

  if (serv_uds_uri.length() && !network::splitURI(serv_uds_uri, serv_uds_path))
  {
    ROS_ERROR("lookupService: Bad service uri [%s]", serv_uds_uri.c_str());

    return false;
  }

  return true;
}

} // namespace ros

