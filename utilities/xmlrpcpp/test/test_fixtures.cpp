
#include "test_fixtures.h"
// No arguments, result is "Hello".

using namespace XmlRpc;

void Hello::execute(XmlRpcValue& params, XmlRpcValue& result) {
  boost::unique_lock<boost::mutex> lock(hello_mutex);
  result = "Hello";
}

XmlRpcTest::XmlRpcTest() : hello(&s), port(0), server_done(false) {}

void XmlRpcTest::work() {
  while (!server_done) {
    s.work(0.1); // run the worker queue for 100ms
  }
}

void XmlRpcTest::SetUp() {
  // XmlRpc::setVerbosity(5);

  // Create the server socket on the specified port
  s.bindAndListen(0);
  port = s.get_port();

  // Enable introspection
  s.enableIntrospection(true);

  // Start the worker thread
  server_thread = boost::thread(boost::mem_fn(&XmlRpcTest::work), this);
}

void XmlRpcTest::TearDown() {
  // TODO(austin): determine if we need to do anything here to avoid
  // leaking resources
  server_done = true;
  if (server_thread.joinable()) {
    server_thread.join();
  }
  s.shutdown();

  // Reset verbosity in case a test raises the verbosity.
  XmlRpc::setVerbosity(0);
}
