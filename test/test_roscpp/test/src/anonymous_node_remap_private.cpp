/*
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Open Source Robotics Foundation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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

#include <algorithm>
#include <string>

#include <ros/ros.h>
#include <gtest/gtest.h>

std::string name_;
bool removeName_ {false};

/**
 * \brief This test tests that even anonymous nodes launched directly via rosrun
 *        (without an explicit __name:= arg) will get the private names remapped
 *        correctly.
 */
TEST(AnonymousNodeRemapPrivate, test)
{
  ros::NodeHandle pnh("~");

  if (!removeName_)
  {
    EXPECT_EQ(name_, ros::this_node::getName());
    EXPECT_EQ(name_, pnh.getNamespace());
  }
  
  const auto remappedName = "/remapped";
  EXPECT_EQ(remappedName, pnh.resolveName("topic"));
  EXPECT_EQ(remappedName, ros::names::resolve("~topic"));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  
  // the first arg of this test has to be the name of the test node, or "noname"
  // "noname" means that no __name:= will be passed to node::init().
  
  name_ = ros::names::resolve(argv[1], false);
  removeName_ = name_ == "/noname";
  
  // remove __name:= so that ros::init() does not use it
  if (removeName_)
  {
    for (int i = 0; i < argc; ++i)
    {
      if (std::string(argv[i]).find("__name:=") == 0)
      {
        std::swap(argv[i], argv[argc - 1]);
        --argc;
        break;
      }
    }
  }
  
  ros::init(argc, argv, "anonymous_node_remap_private",
            ros::InitOption::AnonymousName);
  return RUN_ALL_TESTS();
}
