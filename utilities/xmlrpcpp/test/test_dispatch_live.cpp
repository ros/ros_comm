#include "xmlrpcpp/XmlRpc.h"

#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <iostream>
#include <functional>

#include <gtest/gtest.h>

#include "test_fixtures.h"

using namespace XmlRpc;

TEST_F(XmlRpcTest, Hello) {
  XmlRpcClient c("localhost", port);
  XmlRpcValue noArgs, result;

  // Call the Hello method
  ASSERT_TRUE(c.execute("Hello", noArgs, result));

  EXPECT_FALSE(c.isFault());
  XmlRpcValue hello("Hello");
  EXPECT_EQ(result, hello);
}

TEST_F(XmlRpcTest, HelloNonBlock) {
  XmlRpcClient c("localhost", port);
  XmlRpcValue noArgs, result;

  // Call the Hello method, non-blocking
  ASSERT_TRUE(c.executeNonBlock("Hello", noArgs));

  bool done = false;
  for (int i = 0; i < 30; i++) {
    done = c.executeCheckDone(result);
    if (done)
      break;
    // run the client's dispatch loop to service the respond when it comes back
    c._disp.work(0.1);
  }

  ASSERT_TRUE(done);

  XmlRpcValue hello("Hello");
  EXPECT_EQ(result, hello);
}

TEST_F(XmlRpcTest, HelloNonBlock2) {
  XmlRpcClient c("localhost", port);
  XmlRpcValue noArgs, result;

  // Lock the hello mutex so that the service call cannot return immediately
  hello.hello_mutex.lock();

  // Call the Hello method, non-blocking
  ASSERT_TRUE(c.executeNonBlock("Hello", noArgs));

  bool done = false;
  for (int i = 0; i < 100; i++) {
    done = c.executeCheckDone(result);
    if (done)
      break;
    // run the client's dispatch loop to service the respond when it comes back
    c._disp.work(0.1);

    // unlock the hello mutex after 10 cycles
    if (i == 10)
      hello.hello_mutex.unlock();
  }

  ASSERT_TRUE(done);

  XmlRpcValue hello("Hello");
  EXPECT_EQ(result, hello);
}

TEST_F(XmlRpcTest, ClientDisconnect) {
  XmlRpcClient* c = new XmlRpcClient("localhost", port);
  XmlRpcValue noArgs, result;

  // Lock the hello mutex so that the service call cannot return immediately
  hello.hello_mutex.lock();

  // Call the Hello method, non-blocking
  ASSERT_TRUE(c->executeNonBlock("Hello", noArgs));

  // Destroy the client before the server can answer
  delete c;

  // Unlock the mutex so the server can finish
  hello.hello_mutex.unlock();
}

TEST_F(XmlRpcTest, ServerDisconnect) {
  XmlRpcClient c("localhost", port);
  XmlRpcValue noArgs, result;

  // Ask dispatch to close the socket once this request is done or error
  XmlRpc::setVerbosity(3);

  // Stop calling the work method on the server
  server_done = true;
  server_thread.join();

  // Call the Hello method, non-blocking
  ASSERT_TRUE(c.executeNonBlock("Hello", noArgs));

  // Destroy the server before it can answer
  s.shutdown();

  // Run the client to completion
  bool done = false;
  for (int i = 0; i < 100; i++) {
    done = c.executeCheckDone(result);
    if (done)
      break;
    // run the client's dispatch loop to service the respond when it comes back
    c._disp.work(0.1);
  }

  // The client should return true because the request is done, even though it
  // timed out and wasn't able to complete.
  EXPECT_TRUE(done);
  EXPECT_EQ(-1, c.getfd());

  EXPECT_EQ(result, XmlRpcValue()); // Expect empty result
}

TEST_F(XmlRpcTest, ServerDisconnect2) {
  XmlRpcClient c("localhost", port);
  XmlRpcValue noArgs, result;

  // Stop calling the work method on the server
  server_done = true;
  server_thread.join();
  // Close the server socket to reads (ie incoming connections)
  shutdown(s.getfd(), SHUT_RD);

  // Call the Hello method. Expect failure since the server socket is not
  // accepting new connections.
  ASSERT_FALSE(c.execute("Hello", noArgs, result));

  XmlRpcValue hello; // Expect empty result
  EXPECT_EQ(result, hello);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
