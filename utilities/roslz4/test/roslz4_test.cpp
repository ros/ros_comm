/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Ben Charrow
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
********************************************************************/

#include <gtest/gtest.h>

#include <roslz4/lz4s.h>

class CompressATest :public testing::Test {
protected:
  void SetUp() {
    for (size_t i = 0; i < sizeof(input); ++i) {
      input[i] = 'a';
    }
    for (size_t i = 0; i < sizeof(output); ++i) {
      output[i] = 0;
    }
    for (size_t i = 0; i < sizeof(other); ++i) {
      other[i] = 0;
    }
  }

  char input[1024];
  char output[1048];
  char other[1024];
};

TEST_F(CompressATest, Stream) {
  // Compression
  roslz4_stream stream;
  int ret;
  ret = roslz4_compressStart(&stream, 4);
  ASSERT_EQ(ROSLZ4_OK, ret);

  stream.input_left = sizeof(input);
  stream.input_next = input;
  stream.output_left = sizeof(output);
  stream.output_next = output;

  int counter;
  for (counter = 0; ret == ROSLZ4_OK; ++counter) {
    ret = roslz4_compress(&stream, ROSLZ4_FINISH);
  }
  ASSERT_EQ(ROSLZ4_STREAM_END, ret);

  int output_size = stream.output_next - output;
  roslz4_compressEnd(&stream);

  // Decompression
  stream.input_left = output_size;
  stream.input_next = output;
  stream.output_left = sizeof(other);
  stream.output_next = other;

  ret = roslz4_decompressStart(&stream);
  ASSERT_EQ(ROSLZ4_OK, ret);

  ret = roslz4_decompress(&stream);
  ASSERT_EQ(ROSLZ4_STREAM_END, ret);

  roslz4_decompressEnd(&stream);

  for (size_t i = 0; i < sizeof(other); ++i) {
    ASSERT_EQ(input[i], other[i]) <<  "Original and uncompressed data differ at index " << i;
  }
}

TEST_F(CompressATest, Oneshot) {
  // Compression
  unsigned int comp_size = sizeof(output);
  int ret = roslz4_buffToBuffCompress(input, sizeof(input), output, &comp_size,
                                      4);
  ASSERT_EQ(ROSLZ4_OK, ret);

  // Decompression
  unsigned int decomp_size = sizeof(other);
  ret = roslz4_buffToBuffDecompress(output, comp_size, other, &decomp_size);
  ASSERT_EQ(ROSLZ4_OK, ret);
  ASSERT_EQ(sizeof(input), decomp_size);

  for (size_t i = 0; i < sizeof(other); ++i) {
    ASSERT_EQ(input[i], other[i]) << "Original and uncompressed data differ at index " << i;
  }
}

TEST_F(CompressATest, OneshotDataCorruption) {
  unsigned int comp_size = sizeof(output);
  int ret = roslz4_buffToBuffCompress(input, sizeof(input), output, &comp_size,
                                      4);
  ASSERT_EQ(ROSLZ4_OK, ret);

  output[20] += 1;

  unsigned int decomp_size = sizeof(other);
  ret = roslz4_buffToBuffDecompress(output, comp_size, other, &decomp_size);
  ASSERT_EQ(ROSLZ4_DATA_ERROR, ret);
}


int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
