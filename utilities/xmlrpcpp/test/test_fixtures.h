#include "xmlrpcpp/XmlRpc.h"

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <gtest/gtest.h>

// No arguments, result is "Hello".
class Hello : public XmlRpc::XmlRpcServerMethod {
public:
  Hello(XmlRpc::XmlRpcServer* s) : XmlRpc::XmlRpcServerMethod("Hello", s) {}

  virtual ~Hello() {}

  void execute(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);

  boost::mutex hello_mutex;
};

class XmlRpcTest : public ::testing::Test {
protected:
  XmlRpcTest();

  void work();

  virtual void SetUp();

  virtual void TearDown();

  // The server and its methods
  XmlRpc::XmlRpcServer s;
  Hello hello;

  // Server port number (for clients)
  int port;

  // Server thread
  bool server_done;
  boost::thread server_thread;
};
