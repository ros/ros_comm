#pragma once
#include <string>
#include <gtest/gtest.h>

class MockSocketTest : public ::testing::Test {
protected:
  void SetUp();

  void CheckCalls();
  void TearDown();

  void Expect_socket(int ret);
  void Expect_close(int fd);
  void Expect_setNonBlocking(int fd, bool ret);
  void Expect_setReuseAddr(int fd, bool ret);
  void Expect_bind(int fd, int port, bool ret);
  void Expect_listen(int fd, int backlog, bool ret);
  void Expect_accept(int fd, int ret);
  void Expect_connect(int fd, const std::string& host, int port, bool ret);
  void Expect_nbRead(int fd, const std::string& s, bool eof, bool ret);
  void Expect_nbWrite(int fd, const std::string& s, int bytes, bool ret);
  void Expect_getError(int ret);
  void Expect_get_port(int socket, int ret);
};
