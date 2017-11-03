#include "xmlrpcpp/XmlRpcUtil.h"

#include <gtest/gtest.h>

using namespace XmlRpc;

class FakeLogHandler : public XmlRpcLogHandler {
public:
  FakeLogHandler() : last_level(-1), last_msg(NULL){};

  virtual void log(int level, const char* msg) {
    last_level = level;
    last_msg = msg;
  }

  int last_level;
  const char* last_msg;
};

TEST(XmlRpc, Log) {
  FakeLogHandler fakelog;

  // Check that setting log handler is reflected in getLogHandler().
  XmlRpcLogHandler::setLogHandler(&fakelog);
  ASSERT_EQ(&fakelog, XmlRpcLogHandler::getLogHandler());

  // Check default verbosity.
  ASSERT_EQ(0, XmlRpcLogHandler::getVerbosity());
  EXPECT_EQ(0, XmlRpc::getVerbosity());

  // Test all messages masked at default verbosity.
  for (int i = 1; i < 6; i++) {
    XmlRpcUtil::log(i, "Hello");
    ASSERT_EQ(-1, fakelog.last_level);
    ASSERT_EQ(NULL, fakelog.last_msg);
  }

  // Test masking at levels below maximum verbosity.
  for (int i = 1; i < 5; i++) {
    XmlRpc::setVerbosity(i);

    for (int j = 1; j <= i; j++) {
      XmlRpcUtil::log(j, "Hello1");
      EXPECT_EQ(j, fakelog.last_level);
      EXPECT_STREQ("Hello1", fakelog.last_msg);

      fakelog.last_level = -1;
      fakelog.last_msg = NULL;
    }

    XmlRpcUtil::log(i + 1, "Hello2");
    ASSERT_EQ(-1, fakelog.last_level);
    ASSERT_EQ(NULL, fakelog.last_msg);
  }

  // Test no messages masked at max verbosity.
  XmlRpc::setVerbosity(5);
  for (int i = 1; i < 5; i++) {
    XmlRpcUtil::log(i, "Hello3");
    EXPECT_EQ(i, fakelog.last_level);
    EXPECT_STREQ("Hello3", fakelog.last_msg);

    fakelog.last_level = -1;
    fakelog.last_msg = NULL;
  }

  // Basic formatting test.
  XmlRpcUtil::log(2, "Hello %d", 42);
  EXPECT_EQ(2, fakelog.last_level);
  EXPECT_STREQ("Hello 42", fakelog.last_msg);
}

class FakeErrorHandler : public XmlRpcErrorHandler {
public:
  FakeErrorHandler() : last_msg(NULL){};

  virtual void error(const char* msg) {
    last_msg = msg;
  }

  const char* last_msg;
};

TEST(XmlRpc, error) {
  FakeErrorHandler errors;

  // Check that setErrorHandler is reflected in getErrorHandler.
  XmlRpcErrorHandler::setErrorHandler(&errors);
  EXPECT_EQ(&errors, XmlRpcErrorHandler::getErrorHandler());

  // Basic error check.
  XmlRpcUtil::error("Error!");
  EXPECT_STREQ("Error!", errors.last_msg);
  errors.last_msg = NULL;

  // Error check with formatting.
  XmlRpcUtil::error("%d: I'm a teapot", 408);
  EXPECT_STREQ("408: I'm a teapot", errors.last_msg);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
