/*
 * Unit tests for XmlRpc++
 *
 * Copyright (C) 2017, Zoox Inc
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Austin Hendrix <austin@zoox.com>
 *
 */

#define protected public
#include "xmlrpcpp/XmlRpcClient.h"
#undef protected
#include "xmlrpcpp/XmlRpcValue.h"

#include "mock_socket.h"
#include <errno.h>

#include <gtest/gtest.h>

using XmlRpc::XmlRpcClient;
using XmlRpc::XmlRpcValue;

namespace XmlRpc {
void PrintTo(const XmlRpcClient::ClientConnectionState& state,
             ::std::ostream* os) {
  *os << XmlRpcClient::connectionStateStr(state);
}
}; // namespace XmlRpc

// Helper function to check if our source is in the dispatch source list or not.
bool sourceInList(XmlRpc::XmlRpcSource* source,
                  const XmlRpc::XmlRpcDispatch::SourceList& list) {
  XmlRpc::XmlRpcDispatch::SourceList::const_iterator itr;
  for (itr = list.begin(); itr != list.end(); itr++) {
    if (itr->getSource() == source) {
      return true;
    }
  }
  return false;
}

// Tests for XmlRpcClient
//
// Test connectionStateStr macro
TEST(XmlRpcClient, connectionStateStr) {
#define TEST_STATE(state)                                                      \
  EXPECT_STREQ(#state, XmlRpcClient::connectionStateStr(XmlRpcClient::state))
  TEST_STATE(NO_CONNECTION);
  TEST_STATE(CONNECTING);
  TEST_STATE(WRITE_REQUEST);
  TEST_STATE(READ_HEADER);
  TEST_STATE(READ_RESPONSE);
  TEST_STATE(IDLE);
}

// Test state of client once constructor is done, including optional URL arg.
TEST_F(MockSocketTest, constructor) {
  XmlRpcClient a("localhost", 42);
  EXPECT_EQ("localhost", a._host);
  EXPECT_EQ(42, a._port);
  EXPECT_EQ("/RPC2", a._uri);

  EXPECT_EQ(XmlRpcClient::NO_CONNECTION, a._connectionState);
  EXPECT_EQ(false, a._executing);
  EXPECT_EQ(false, a._eof);
  EXPECT_EQ(true, a.getKeepOpen());
  EXPECT_EQ(-1, a.getfd());
  EXPECT_FALSE(sourceInList(&a, a._disp._sources));

  XmlRpcClient b("nowhere.com", 404, "/where");
  EXPECT_EQ("nowhere.com", b._host);
  EXPECT_EQ(404, b._port);
  EXPECT_EQ("/where", b._uri);
  // Don't really need to repeat the tests for the other variables.
}

// Test close() function:
//  * Does not call close when socket is already closed.
TEST_F(MockSocketTest, close_invalid_fd) {
  XmlRpcClient a("localhost", 42);

  ASSERT_EQ(-1, a.getfd());

  // Close when no file descriptor is set
  a.close();
  EXPECT_EQ(-1, a.getfd());
}

// Test close() function:
//  * Correctly calls close when socket is not closed.
TEST_F(MockSocketTest, close_valid_fd) {
  XmlRpcClient a("localhost", 42);

  // Set file descriptor and then expect that close is called once.
  a.setfd(5);
  Expect_close(5);
  a.close();
  EXPECT_EQ(-1, a.getfd());

  // Intermediate check that all expected calls have been executed.
  CheckCalls();

  // Calling close again should have no effect.
  a.close();
  EXPECT_EQ(-1, a.getfd());
}

TEST_F(MockSocketTest, close_destructor) {
  // Test that the XmlRpcClient destructor closes the file descriptor.
  {
    XmlRpcClient a("localhost", 42);

    // Set file descriptor and then expect that close is called once.
    a.setfd(5);
    Expect_close(5);
    // XmlRpcClient destructor called at the end of this scope.
  }
}

// Test execute() function? TODO(future work)
//  This is a bit complicated; maybe test the underlying functions and then
//  mock them out for this test.
// Test exectuteNonBlock() function? TODO(future work)
//  Again, complicated. Maybe test underlying functions.
// Test executeCheckDone() ? TODO(future work)
//  Complicated. There are known bugs here where the client never reports done
//  if the connection is lost; we should try to test that logic.
// Test handleEvent() TODO(future work)
//  For each ConnectionState
//   The correct sequence of socket functions is called.
//   The correct action is taken if the socket operations fail.

// Test setupConnection()
//  Correct handling of initial states.
//  Correct handling of errors.
TEST_F(MockSocketTest, setupConnection) {
  XmlRpcClient a("localhost", 42);

  // Default initial state; expect socket, setNonBlocking and connect.
  // NOTE(austin): This does not currently expect these calls in order; it just
  // checks that all of them have been made.
  Expect_socket(7);
  Expect_setNonBlocking(7, true);
  Expect_connect(7, "localhost", 42, true);

  EXPECT_TRUE(a.setupConnection());

  // Check that correct FD was set.
  EXPECT_EQ(7, a.getfd());

  // Check that source is in the dispatch source list now.
  EXPECT_TRUE(sourceInList(&a, a._disp._sources));

  // Check internal state is reset.
  EXPECT_EQ(XmlRpcClient::WRITE_REQUEST, a._connectionState);
  EXPECT_EQ(0, a._bytesWritten);
  CheckCalls();

  Expect_close(7);
}

TEST_F(MockSocketTest, setupConnection_eof) {
  XmlRpcClient a("localhost", 42);

  // Initial state: file descriptor set, but _eof. Expect close and socket,
  // setNonBlocking, connect.
  a.setfd(5);
  a._eof = true;
  Expect_close(5);

  Expect_socket(7);
  Expect_setNonBlocking(7, true);
  Expect_connect(7, "localhost", 42, true);

  EXPECT_TRUE(a.setupConnection());

  // Check that correct FD was set.
  EXPECT_EQ(7, a.getfd());

  // Check that source is in the dispatch source list now.
  EXPECT_TRUE(sourceInList(&a, a._disp._sources));

  // Check internal state is reset.
  EXPECT_EQ(XmlRpcClient::WRITE_REQUEST, a._connectionState);
  EXPECT_EQ(0, a._bytesWritten);
  CheckCalls();

  Expect_close(7);
}

TEST_F(MockSocketTest, setupConnection_close) {
  XmlRpcClient a("localhost", 42);

  // Initial state: file descriptor set, but _eof
  // NOTE(austin): Having multiple variables that imply that the fd is valid
  //               smells funny.
  a.setfd(5);
  a._connectionState = XmlRpcClient::WRITE_REQUEST;
  Expect_close(5);

  Expect_socket(7);
  Expect_setNonBlocking(7, true);
  Expect_connect(7, "localhost", 42, true);

  EXPECT_TRUE(a.setupConnection());

  // Check that correct FD was set.
  EXPECT_EQ(7, a.getfd());

  // Check that source is in the dispatch source list now.
  EXPECT_TRUE(sourceInList(&a, a._disp._sources));

  // Check internal state is reset.
  EXPECT_EQ(XmlRpcClient::WRITE_REQUEST, a._connectionState);
  EXPECT_EQ(0, a._bytesWritten);
  CheckCalls();

  Expect_close(7);
}

TEST_F(MockSocketTest, setupConnection_err) {
  XmlRpcClient a("localhost", 42);

  // Connect failure. Any failure that causes doConnect here will do; we choose
  // an arbitrary failure mode here and then separately test that doConnect
  // reports failure correctly in a separate set of tests.
  Expect_socket(7);
  Expect_setNonBlocking(7, true);
  Expect_connect(7, "localhost", 42, false);
  Expect_getError(ECONNREFUSED);
  Expect_close(7);

  EXPECT_FALSE(a.setupConnection());

  // Check that FD is not set.
  EXPECT_EQ(-1, a.getfd());

  // Check that source is not in the dispatch source list now.
  EXPECT_FALSE(sourceInList(&a, a._disp._sources));

  // Check internal state is NO_CONNECTION
  EXPECT_EQ(XmlRpcClient::NO_CONNECTION, a._connectionState);
}

// Check that _disp is left in the correct state after closing and then failing
// to reopen the socket.
TEST_F(MockSocketTest, setupConnection_eor_reopen) {
  XmlRpcClient a("localhost", 42);

  // Default initial state; expect socket, setNonBlocking and connect.
  // NOTE(austin): This does not currently expect these calls in order; it just
  // checks that all of them have been made.
  Expect_socket(7);
  Expect_setNonBlocking(7, true);
  Expect_connect(7, "localhost", 42, true);

  EXPECT_TRUE(a.setupConnection());

  // Check that correct FD was set.
  EXPECT_EQ(7, a.getfd());

  // Check that source is in the dispatch source list now.
  EXPECT_TRUE(sourceInList(&a, a._disp._sources));

  // Check internal state is reset.
  EXPECT_EQ(XmlRpcClient::WRITE_REQUEST, a._connectionState);
  EXPECT_EQ(0, a._bytesWritten);
  CheckCalls();

  // Simulate socket encountering EOF
  a._eof = true;

  // Expect close, socket, setNonBlocking and connect. Make connect fail.
  Expect_close(7);
  Expect_socket(8);
  Expect_setNonBlocking(8, true);
  Expect_connect(8, "localhost", 42, false);
  Expect_getError(ECONNREFUSED);
  Expect_close(8);

  EXPECT_FALSE(a.setupConnection());

  // Check that correct FD was set.
  EXPECT_EQ(-1, a.getfd());

  // Check that source is no longer in the dispatch source list now.
  EXPECT_FALSE(sourceInList(&a, a._disp._sources));

  // Check internal state is reset.
  EXPECT_EQ(XmlRpcClient::NO_CONNECTION, a._connectionState);
  CheckCalls();
}

// Test doConnect()
//  Correct sequence of calls to XmlRpcSocket
//  Correct handling of socket errors.
TEST_F(MockSocketTest, doConnect) {
  XmlRpcClient a("localhost", 42);

  // Expect that socket, setNonBlocking and connect are called.
  // NOTE(austin): this doesn't enforce ordering.
  Expect_socket(7);
  Expect_setNonBlocking(7, true);
  Expect_connect(7, "localhost", 42, true);

  EXPECT_TRUE(a.doConnect());

  // Check that correct FD was set.
  EXPECT_EQ(7, a.getfd());

  // Check that the expected calls have happened.
  CheckCalls();

  // Expect close on destruction.
  Expect_close(7);
}

TEST_F(MockSocketTest, doConnect_socketerr) {
  XmlRpcClient a("localhost", 42);

  Expect_socket(-1);
  Expect_getError(ENFILE);

  EXPECT_FALSE(a.doConnect());

  // Check that correct FD was set.
  EXPECT_EQ(-1, a.getfd());
}

TEST_F(MockSocketTest, doConnect_nonBlockErr) {
  XmlRpcClient a("localhost", 42);

  // Expect that setNonBlocking causes the socket to be closed and getError to
  // be called.
  Expect_socket(7);
  Expect_setNonBlocking(7, false);
  Expect_getError(EBADF);
  Expect_close(7);

  EXPECT_FALSE(a.doConnect());

  // Check that correct FD was set.
  EXPECT_EQ(-1, a.getfd());
}

TEST_F(MockSocketTest, doConnect_connectErr) {
  XmlRpcClient a("localhost", 42);

  // Expect that a connection failure causes the socket to be closed.
  Expect_socket(7);
  Expect_setNonBlocking(7, true);
  Expect_connect(7, "localhost", 42, false);
  Expect_getError(ECONNREFUSED);
  Expect_close(7);

  EXPECT_FALSE(a.doConnect());

  // Check that correct FD was set.
  EXPECT_EQ(-1, a.getfd());
}

// Test generateRequest()
//  Correct XML is generated for a variety of request types.
//  Correct XML is generated for empty request
TEST(XmlRpcClient, generateRequest) {
  XmlRpcClient a("localhost", 42);
  // generateRequest takes a method name and params and puts the result in the
  // _request member variable.
  std::string header = "POST /RPC2 HTTP/1.1\r\n"
                       "User-Agent: XMLRPC++ 0.7\r\n"
                       "Host: localhost:42\r\n"
                       "Content-Type: text/xml\r\n";

  // Various tests for the nominal case of a normal XmlRpcValue
  XmlRpcValue s("Hello");
  EXPECT_TRUE(a.generateRequest("DoFoo", s));
  EXPECT_EQ(header + "Content-length: 134\r\n\r\n"
                     "<?xml version=\"1.0\"?>\r\n"
                     "<methodCall><methodName>DoFoo</methodName>\r\n"
                     "<params><param><value>Hello</value></param></params></"
                     "methodCall>\r\n",
            a._request);

  XmlRpcValue b(true);
  EXPECT_TRUE(a.generateRequest("DoFoo", b));
  EXPECT_EQ(header + "Content-length: 149\r\n\r\n"
                     "<?xml version=\"1.0\"?>\r\n"
                     "<methodCall><methodName>DoFoo</methodName>\r\n"
                     "<params><param><value><boolean>1</boolean></value></"
                     "param></params></methodCall>\r\n",
            a._request);

  XmlRpcValue i(42);
  EXPECT_TRUE(a.generateRequest("DoFoo", i));
  EXPECT_EQ(header + "Content-length: 140\r\n\r\n"
                     "<?xml version=\"1.0\"?>\r\n"
                     "<methodCall><methodName>DoFoo</methodName>\r\n"
                     "<params><param><value><i4>42</i4></value></param></"
                     "params></methodCall>\r\n",
            a._request);

  // Array case is a bit special because it results in emitting multiple
  // <param> tags.
  XmlRpcValue arr;
  arr[0] = 2;
  arr[1] = 3;
  arr[2] = 5;
  arr[3] = 7;
  arr[4] = 11;
  arr[5] = 13;
  arr[6] = 17;
  EXPECT_TRUE(a.generateRequest("DoFoo", arr));
  EXPECT_EQ(header + "Content-length: 382\r\n\r\n"
                     "<?xml version=\"1.0\"?>\r\n"
                     "<methodCall><methodName>DoFoo</methodName>\r\n"
                     "<params>"
                     "<param><value><i4>2</i4></value></param>"
                     "<param><value><i4>3</i4></value></param>"
                     "<param><value><i4>5</i4></value></param>"
                     "<param><value><i4>7</i4></value></param>"
                     "<param><value><i4>11</i4></value></param>"
                     "<param><value><i4>13</i4></value></param>"
                     "<param><value><i4>17</i4></value></param>"
                     "</params></methodCall>\r\n",
            a._request);

  // Tests for errors in arguments.
  // Method name null
  // TODO(austin): this causes a segfault.
  /*
  EXPECT_TRUE(a.generateRequest(NULL, s));
  EXPECT_EQ(header + "Content-length: 134\r\n\r\n" \
      "<?xml version=\"1.0\"?>\r\n" \
      "<methodCall><methodName></methodName>\r\n" \
      "<params><param><value>Hello</value></param></params></methodCall>\r\n"
      , a._request);
      */

  // Empty(invalid) param(s)
  // NOTE(austin): this matches the existing implementation but seems unusual.
  // I don't have a specification to check this against so I'm just going to
  // assume it's OK.
  XmlRpcValue empty;
  EXPECT_FALSE(empty.valid());
  EXPECT_TRUE(a.generateRequest("DoEmpty", empty));
  EXPECT_EQ(header + "Content-length: 84\r\n\r\n"
                     "<?xml version=\"1.0\"?>\r\n"
                     "<methodCall><methodName>DoEmpty</methodName>\r\n"
                     "</methodCall>\r\n",
            a._request);
}

// Test generateHeader()
//  Correct header is generated for various sizes and content of request body.
TEST(XmlRpcClient, generateHeader) {
  XmlRpcClient a("localhost", 42);
  // generateRequest takes a method name and params and puts the result in the
  // _request member variable.
  std::string header = "POST /RPC2 HTTP/1.1\r\n"
                       "User-Agent: XMLRPC++ 0.7\r\n"
                       "Host: localhost:42\r\n"
                       "Content-Type: text/xml\r\n";

  // Nonzero length message body.
  EXPECT_EQ(header + "Content-length: 30\r\n\r\n", a.generateHeader(30));

  // Zero-length body as degenerate case.
  EXPECT_EQ(header + "Content-length: 0\r\n\r\n", a.generateHeader(0));
}

// Test writeRequest()
//  Test that everything in a single write works; Client is left in the correct
//   state.
//  Test that partial write works, client is left in the correct state.
//  Test that socket errors result in the correct state and that the socket
//   is closed (or not) correctly.
TEST_F(MockSocketTest, writeRequest) {
  XmlRpcClient a("localhost", 42);

  // Hack us into the correct initial state.
  a.setfd(7);
  a._connectionState = XmlRpcClient::WRITE_REQUEST;
  a._request = "FAKE_REQUEST";

  // Check that request length is the correct size.
  ASSERT_EQ(12, a._request.size());

  // Expect a write; write all bytes and return success.
  Expect_nbWrite(7, "FAKE_REQUEST", 12, true);

  // Expect that writeRequest is successful.
  EXPECT_TRUE(a.writeRequest());

  // Check that resulting state is READ_HEADER
  EXPECT_EQ(XmlRpcClient::READ_HEADER, a._connectionState);

  // Check that all expected function calls were made before destruction.
  CheckCalls();

  // Expect a close on destruction.
  Expect_close(7);
}

TEST_F(MockSocketTest, writeRequest_partial) {
  XmlRpcClient a("localhost", 42);

  // Hack us into the correct initial state.
  a.setfd(7);
  a._connectionState = XmlRpcClient::WRITE_REQUEST;
  a._request = "FAKE_REQUEST";

  // Check that request length is the correct size.
  ASSERT_EQ(12, a._request.size());

  // Expect a write; only write 5 bytes and return success.
  Expect_nbWrite(7, "FAKE_REQUEST", 5, true);
  // Expect that writeRequest is successful.
  EXPECT_TRUE(a.writeRequest());
  // Check that resulting state is READ_HEADER
  EXPECT_EQ(XmlRpcClient::WRITE_REQUEST, a._connectionState);
  // Check that all expected function calls were made.
  CheckCalls();

  // Expect a write; write remaining bytes and return success.
  Expect_nbWrite(7, "REQUEST", 7, true);
  // Expect that writeRequest is successful.
  EXPECT_TRUE(a.writeRequest());
  // Check that resulting state is READ_HEADER
  EXPECT_EQ(XmlRpcClient::READ_HEADER, a._connectionState);
  // Check that all expected function calls were made before destruction.
  CheckCalls();

  // Expect a close on destruction.
  Expect_close(7);
}

TEST_F(MockSocketTest, writeRequest_partial_error) {
  XmlRpcClient a("localhost", 42);

  // Hack us into the correct initial state.
  a.setfd(7);
  a._connectionState = XmlRpcClient::WRITE_REQUEST;
  a._request = "FAKE_REQUEST";

  // Check that request length is the correct size.
  ASSERT_EQ(12, a._request.size());

  // Expect a write; only write 5 bytes and return success.
  Expect_nbWrite(7, "FAKE_REQUEST", 5, true);
  // Expect that writeRequest is successful.
  EXPECT_TRUE(a.writeRequest());
  // Check that resulting state is READ_HEADER
  EXPECT_EQ(XmlRpcClient::WRITE_REQUEST, a._connectionState);
  // Check that all expected function calls were made.
  CheckCalls();

  // Expect a write; write no bytes and return failure.
  Expect_nbWrite(7, "REQUEST", 0, false);
  // Expect close, since the write failed.
  Expect_close(7);
  Expect_getError(ECONNREFUSED);
  // Expect that writeRequest fails.
  EXPECT_FALSE(a.writeRequest());
  // Check that resulting state is not connected.
  EXPECT_EQ(XmlRpcClient::NO_CONNECTION, a._connectionState);

  // Check that all expected function calls were made before destruction.
  CheckCalls();
}

TEST_F(MockSocketTest, writeRequest_error) {
  XmlRpcClient a("localhost", 42);

  // Hack us into the correct initial state.
  a.setfd(7);
  a._connectionState = XmlRpcClient::WRITE_REQUEST;
  a._request = "FAKE_REQUEST";

  // Check that request length is the correct size.
  ASSERT_EQ(12, a._request.size());

  // Expect a write; write no bytes and return error.
  Expect_nbWrite(7, "FAKE_REQUEST", 0, false);
  Expect_close(7);
  Expect_getError(ECONNREFUSED);
  // Expect that writeRequest fails.
  EXPECT_FALSE(a.writeRequest());
  // Check that resulting state is not connected.
  EXPECT_EQ(XmlRpcClient::NO_CONNECTION, a._connectionState);

  // Check that all expected function calls were made before destruction.
  CheckCalls();
}

// Header as it would be generated by XmlRpcpp and body.
const std::string header = "HTTP/1.1 200 OK\r\n"
                           "Server: XMLRPC++ 0.7\r\n"
                           "Content-Type: text/xml\r\n"
                           "Content-length: 114\r\n\r\n";
// Alternate header as returned by roscore
const std::string header2 = "HTTP/1.0 200 OK\r\n"
                            "Server: BaseHTTP/0.3 Python/2.7.6\r\n"
                            "Date: Mon, 30 Oct 2017 22:28:12 GMT\r\n"
                            "Content-type: text/xml\r\n"
                            "Content-length: 114\r\n\r\n";
// Generic response XML
const std::string response = "<?xml version=\"1.0\"?>\r\n"
                             "<methodResponse><params><param>\r\n"
                             "<value>Hello</value>\r\n"
                             "</param></params></methodResponse>\r\n";

// Test readHeader()
//  Test that a full read with just a header works correctly.
//  Test that various partial reads of the header work.
//  Test that stacked partial reads of the header eventually result in success.
//  Test that a read that returns both header and response data correctly
//   removes the header and leaves the response.
// TODO(future work): Test the header parser in readHeader.
// TODO(future work): Test cases where the Content-length header is absent,
//                    zero, negative or garbage.
//
// Test that readHeader works correctly with the header from XmlRpcpp
TEST_F(MockSocketTest, readHeader) {
  XmlRpcClient a("localhost", 42);

  // Hack us into the correct initial state.
  a.setfd(7);
  a._connectionState = XmlRpcClient::READ_HEADER;

  // Self-test that response length matches encoded values above.
  ASSERT_EQ(114, response.length());

  // Expect a read and have it return the header and the response.
  Expect_nbRead(7, header + response, false, true);

  EXPECT_TRUE(a.readHeader());

  // Check that state machine is in the correct state after getting the header.
  EXPECT_EQ(XmlRpcClient::READ_RESPONSE, a._connectionState);
  // Check that the remaining response is stored in _response
  EXPECT_EQ(response, a._response);
  EXPECT_EQ(114, a._contentLength); // Check decoded content length

  // Check that all expected function calls were made before destruction.
  CheckCalls();

  // Expect close on destruction.
  Expect_close(7);
}

// Test that readHeader works correctly with the header from roscore.
TEST_F(MockSocketTest, readHeader2) {
  XmlRpcClient a("localhost", 42);

  // Hack us into the correct initial state.
  a.setfd(7);
  a._connectionState = XmlRpcClient::READ_HEADER;

  // Expect a read and have it return the header and the response.
  Expect_nbRead(7, header2 + response, false, true);

  EXPECT_TRUE(a.readHeader());

  // Check that state machine is in the correct state after getting the header.
  EXPECT_EQ(XmlRpcClient::READ_RESPONSE, a._connectionState);
  // Check that the remaining response is stored in _response
  EXPECT_EQ(response, a._response);
  EXPECT_EQ(114, a._contentLength); // Check decoded content length

  // Check that all expected function calls were made before destruction.
  CheckCalls();

  // Expect close on destruction.
  Expect_close(7);
}

// Test that readHeader decodes the content-length when read only returns the
// header.
// TODO(future work): there appears to be a bug in the header parsing where it
// doesn't pass the header parsing state until it gets the first byte of the
// response body.
TEST_F(MockSocketTest, readHeader_only) {
  XmlRpcClient a("localhost", 42);

  // Hack us into the correct initial state.
  a.setfd(7);
  a._connectionState = XmlRpcClient::READ_HEADER;

  // Expect a read and have it return only the header.
  // NOTE(austin): the header parser doesn't terminate until it gets the first
  // byte of the response; so give it one additional byte.
  Expect_nbRead(7, header + " ", false, true);

  EXPECT_TRUE(a.readHeader());

  // Check that state machine is in the correct state after getting the header.
  EXPECT_EQ(XmlRpcClient::READ_RESPONSE, a._connectionState);
  // Check that the remaining response is stored in _response
  EXPECT_EQ(" ", a._response);
  EXPECT_EQ(114, a._contentLength); // Check decoded content length

  // Check that all expected function calls were made before destruction.
  CheckCalls();

  // Expect close on destruction.
  Expect_close(7);
}

// Test that readHeader correctly resumes after getting a partial header read.
TEST_F(MockSocketTest, readHeader_partial) {
  XmlRpcClient a("localhost", 42);

  // Hack us into the correct initial state.
  a.setfd(7);
  a._connectionState = XmlRpcClient::READ_HEADER;

  std::string header_part1 = header.substr(0, 32);
  std::string header_part2 = header.substr(32);

  // Expect a read and have it return only part of the header.
  Expect_nbRead(7, header_part1, false, true);
  EXPECT_TRUE(a.readHeader());
  // Check that state machine is in the correct state after getting the header.
  EXPECT_EQ(XmlRpcClient::READ_HEADER, a._connectionState);
  // Check that the partial header is stored in _header.
  EXPECT_EQ(header_part1, a._header);

  // Check that all expected function calls were made before we proceed.
  CheckCalls();

  // Expect a read and have it return the remainder of the header.
  Expect_nbRead(7, header_part2 + " ", false, true);
  EXPECT_TRUE(a.readHeader());
  // Check that state machine is in the correct state after getting the header.
  EXPECT_EQ(XmlRpcClient::READ_RESPONSE, a._connectionState);
  // Check that the remaining response is stored in _response
  EXPECT_EQ(" ", a._response);
  EXPECT_EQ(114, a._contentLength); // Check decoded content length

  // Check that all expected function calls were made before destruction.
  CheckCalls();

  // Expect close on destruction.
  Expect_close(7);
}

// Test that readHeader reports an error if the read fails.
TEST_F(MockSocketTest, readHeader_err) {
  XmlRpcClient a("localhost", 42);

  // Hack us into the correct initial state.
  a.setfd(7);
  a._connectionState = XmlRpcClient::READ_HEADER;

  // Expect a read and have it fail.
  Expect_nbRead(7, "", false, false);
  Expect_getError(ENOTCONN);
  // Expect the client to close the socket.
  Expect_close(7);

  EXPECT_FALSE(a.readHeader());

  // Check that state machine is in the correct state after getting the header.
  EXPECT_EQ(XmlRpcClient::NO_CONNECTION, a._connectionState);

  // Check that all expected function calls were made before destruction.
  CheckCalls();
}

// Test that readHeader reports an error if it gets EOF before it gets the
// content length.
TEST_F(MockSocketTest, readHeader_eof) {
  XmlRpcClient a("localhost", 42);

  // Hack us into the correct initial state.
  a.setfd(7);
  a._connectionState = XmlRpcClient::READ_HEADER;

  // Expect a read and have it return eof and success.
  Expect_nbRead(7, "", true, true);
  Expect_getError(ENOTCONN);
  // Expect the client to close the socket.
  Expect_close(7);

  EXPECT_FALSE(a.readHeader());

  // Check that state machine is in the correct state after getting the header.
  EXPECT_EQ(XmlRpcClient::NO_CONNECTION, a._connectionState);

  // Check that all expected function calls were made before destruction.
  CheckCalls();
}

// Test that readHeader reports an error and closes the socket if the second
// part of a partial read returns an error.
TEST_F(MockSocketTest, readHeader_partial_err) {
  XmlRpcClient a("localhost", 42);

  // Hack us into the correct initial state.
  a.setfd(7);
  a._connectionState = XmlRpcClient::READ_HEADER;

  std::string header_part1 = header.substr(0, 32);

  // Expect a read and have it return only part of the header.
  Expect_nbRead(7, header_part1, false, true);
  EXPECT_TRUE(a.readHeader());
  // Check that state machine is in the correct state after getting the header.
  EXPECT_EQ(XmlRpcClient::READ_HEADER, a._connectionState);
  // Check that the partial header is stored in _header.
  EXPECT_EQ(header_part1, a._header);

  // Check that all expected function calls were made before we proceed.
  CheckCalls();

  // Expect a read and have it return an error.
  Expect_nbRead(7, "", false, false);
  Expect_getError(ENOTCONN);
  Expect_close(7);
  EXPECT_FALSE(a.readHeader());
  // Check that state machine is in the correct state after getting the header.
  EXPECT_EQ(XmlRpcClient::NO_CONNECTION, a._connectionState);

  // Check that all expected function calls were made before destruction.
  CheckCalls();
}

// Test readResponse()
//  Test read of response in a single read call
//  Test response spread across several read calls
//  Test empty response
//  Test read errors on first or subsequent reads
//  Test for correct state after each of these events.

// Test that readResponse does nothing if _response is already populated by
// readHeader.
TEST_F(MockSocketTest, readResponse_noop) {
  XmlRpcClient a("localhost", 42);

  // Hack us into the correct initial state.
  a.setfd(7);
  a._connectionState = XmlRpcClient::READ_RESPONSE;
  // Response was already read by readHeader
  // (verified by previous tests).
  a._response = response;
  a._contentLength = 114;
  ASSERT_EQ(a._contentLength, a._response.length());

  // Don't expect any calls. Expect readResponse to return false since we don't
  // need to keep monitoring this client.
  EXPECT_FALSE(a.readResponse());

  // Check that state machine is in the correct state after getting the header.
  EXPECT_EQ(XmlRpcClient::IDLE, a._connectionState);
  // Check that the remaining response is stored in _response
  EXPECT_EQ(response, a._response);
  EXPECT_EQ(114, a._contentLength); // Check decoded content length

  // Check that all expected function calls were made before destruction.
  CheckCalls();

  // Expect close on destruction.
  Expect_close(7);
}

// Test that readResponse works if the initial buffer is empty.
TEST_F(MockSocketTest, readResponse) {
  XmlRpcClient a("localhost", 42);

  // Hack us into the correct initial state.
  a.setfd(7);
  a._connectionState = XmlRpcClient::READ_RESPONSE;
  // Start with empty response and pre-populated content length.
  a._response = "";
  a._contentLength = 114;
  ASSERT_EQ(114, response.length());

  // Expect a read, have it return the full response and success.
  Expect_nbRead(7, response, false, true);
  // Expect readResponse to return false since we're done monitoring this
  // socket.
  EXPECT_FALSE(a.readResponse());

  // Check that state machine is in the correct state after getting the header.
  EXPECT_EQ(XmlRpcClient::IDLE, a._connectionState);
  // Check that the remaining response is stored in _response
  EXPECT_EQ(response, a._response);

  // Check that all expected function calls were made before destruction.
  CheckCalls();

  // Expect close on destruction.
  Expect_close(7);
}

// Test that readResponse works if the initial buffer is empty.
TEST_F(MockSocketTest, readResponse_partial) {
  XmlRpcClient a("localhost", 42);

  // Hack us into the correct initial state.
  a.setfd(7);
  a._connectionState = XmlRpcClient::READ_RESPONSE;
  // Start with empty response and pre-populated content length.
  a._response = "";
  a._contentLength = 114;
  ASSERT_EQ(114, response.length());

  const std::string response_part1 = response.substr(0, 60);
  const std::string response_part2 = response.substr(60);

  // Expect a read, have it return the first part of the response.
  Expect_nbRead(7, response_part1, false, true);
  EXPECT_TRUE(a.readResponse());

  // Check that the first part of the response is stored in _response
  EXPECT_EQ(response_part1, a._response);
  // Check that state machine is in the correct state after getting the header.
  EXPECT_EQ(XmlRpcClient::READ_RESPONSE, a._connectionState);

  // Check that all expected function calls were made before destruction.
  CheckCalls();

  // Expect a read, have it return the remainder of the response and success.
  Expect_nbRead(7, response_part2, false, true);

  // Expect readResponse to return false now since we're done monitoring this
  // socket.
  EXPECT_FALSE(a.readResponse());

  // Check that state machine is in the correct state after getting the header.
  EXPECT_EQ(XmlRpcClient::IDLE, a._connectionState);
  // Check that the remaining response is stored in _response
  EXPECT_EQ(response, a._response);

  // Check that all expected function calls were made before destruction.
  CheckCalls();

  // Expect close on destruction.
  Expect_close(7);
}

// Test that readResponse closes the socket and returns an error if the read
// fails.
TEST_F(MockSocketTest, readResponse_err) {
  XmlRpcClient a("localhost", 42);

  // Hack us into the correct initial state.
  a.setfd(7);
  a._connectionState = XmlRpcClient::READ_RESPONSE;
  // Start with empty response and pre-populated content length.
  a._response = "";
  a._contentLength = 114;
  ASSERT_EQ(114, response.length());

  // Expect a read, have it return an error.
  Expect_nbRead(7, "", false, false);
  // Expect getError and close()
  Expect_getError(ENOTCONN);
  Expect_close(7);
  // Expect readResponse to return false since we're done monitoring this
  // socket.
  EXPECT_FALSE(a.readResponse());

  // Check that state machine is in the correct state after getting the header.
  EXPECT_EQ(XmlRpcClient::NO_CONNECTION, a._connectionState);

  // Check that all expected function calls were made before destruction.
  CheckCalls();
}

// Test that readResponse works if the initial buffer is empty.
TEST_F(MockSocketTest, readResponse_eof) {
  XmlRpcClient a("localhost", 42);

  // Hack us into the correct initial state.
  a.setfd(7);
  a._connectionState = XmlRpcClient::READ_RESPONSE;
  // Start with empty response and pre-populated content length.
  a._response = "";
  a._contentLength = 114;
  ASSERT_EQ(114, response.length());

  // Expect a read, have it return nothing and EOF.
  Expect_nbRead(7, "", true, true);
  // Expect that we close the socket.
  Expect_close(7);
  // Expect readResponse to return false since we're done monitoring this
  // socket.
  EXPECT_FALSE(a.readResponse());

  // Check that state machine is in the correct state after getting the header.
  EXPECT_EQ(XmlRpcClient::NO_CONNECTION, a._connectionState);

  // Check that all expected function calls were made before destruction.
  CheckCalls();
}

// Test parseResponse
//  Test correct parsing of various response types: empty, int, double,
//   string, bool, list, struct, date, base64, etc
//  Test for correct handling of empty response body
//  Test for correct parsing/handling of partial response
//  Test for correct handling of garbage data
//  Test for correct handling of mismatched XML tags at top level
//  Test for correct handling of ALL UPPERCASE, lowercase and CamelCase tags

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
