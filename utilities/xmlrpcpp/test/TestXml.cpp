// TestXml.cpp : Test XML encoding and decoding.
// The characters <>&'" are illegal in xml and must be encoded.

#define WIN32_LEAN_AND_MEAN // Exclude rarely-used stuff from Windows headers

#include <iostream>
// If you are using MSVC++6, you should update <string> to fix
// BUG: getline Template Function Reads Extra Character
#include <string>
#include <stdlib.h>

#include "xmlrpcpp/XmlRpcUtil.h"

#include <gtest/gtest.h>

using namespace XmlRpc;

TEST(XmlRpc, BasicXml) {
  // Basic tests
  std::string empty;
  EXPECT_EQ(empty, XmlRpcUtil::xmlEncode(empty));
  EXPECT_EQ(empty, XmlRpcUtil::xmlDecode(empty));
  EXPECT_EQ(empty, XmlRpcUtil::xmlEncode(""));
  EXPECT_EQ(empty, XmlRpcUtil::xmlDecode(""));

  std::string raw("<>&'\"");
  EXPECT_EQ(XmlRpcUtil::xmlDecode(XmlRpcUtil::xmlEncode(raw)), raw);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
