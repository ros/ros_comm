#ifndef _XMLRPCCLIENTINFO_H_
#define _XMLRPCCLIENTINFO_H_

#include <string>

namespace XmlRpc
{
  struct XmlRpcClientInfo
  {
    short family;
    unsigned short port;
    std::string ip;
  };
}

#endif // _XMLRPCCLIENTINFO_H_
