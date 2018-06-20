/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
 *     * Neither the name of Willow Garage, Inc. nor the names of its
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

/* Author: Josh Faust */

/*
 * Test parameters
 */

#include <string>
#include <sstream>
#include <fstream>

#include <gtest/gtest.h>

#include <time.h>
#include <stdlib.h>

#include "ros/ros.h"
#include <ros/param.h>

using namespace ros;

TEST(Params, allParamTypes)
{
  std::string string_param;
  EXPECT_TRUE( param::get( "string", string_param ) );
  EXPECT_TRUE( string_param == "test" );

  int int_param = 0;
  EXPECT_TRUE( param::get( "int", int_param ) );
  EXPECT_TRUE( int_param == 10 );

  double double_param = 0.0;
  EXPECT_TRUE( param::get( "double", double_param ) );
  EXPECT_DOUBLE_EQ( double_param, 10.5 );

  bool bool_param = true;
  EXPECT_TRUE( param::get( "bool", bool_param ) );
  EXPECT_FALSE( bool_param );
}

TEST(Params, setThenGetString)
{
  param::set( "test_set_param", std::string("asdf") );
  std::string param;
  ASSERT_TRUE( param::get( "test_set_param", param ) );
  ASSERT_STREQ( "asdf", param.c_str() );
  
  XmlRpc::XmlRpcValue v;
  param::get("test_set_param", v);
  ASSERT_EQ(v.getType(), XmlRpc::XmlRpcValue::TypeString);
}

TEST(Params, setThenGetStringCached)
{
  std::string param;
  ASSERT_FALSE( param::getCached( "test_set_param_setThenGetStringCached", param) );

  param::set( "test_set_param_setThenGetStringCached", std::string("asdf") );

  ASSERT_TRUE( param::getCached( "test_set_param_setThenGetStringCached", param) );
  ASSERT_STREQ( "asdf", param.c_str() );
}

TEST(Params, setThenGetStringCachedNodeHandle)
{
	NodeHandle nh;
  std::string param;
  ASSERT_FALSE( nh.getParamCached( "test_set_param_setThenGetStringCachedNodeHandle", param) );

  nh.setParam( "test_set_param_setThenGetStringCachedNodeHandle", std::string("asdf") );

  ASSERT_TRUE( nh.getParamCached( "test_set_param_setThenGetStringCachedNodeHandle", param) );
  ASSERT_STREQ( "asdf", param.c_str() );
}

TEST(Params, setThenGetNamespaceCached)
{
  std::string stringParam;
  XmlRpc::XmlRpcValue structParam;
  const std::string ns = "test_set_param_setThenGetStringCached2";
  ASSERT_FALSE(param::getCached(ns, stringParam));

  param::set(ns, std::string("a"));
  ASSERT_TRUE(param::getCached(ns, stringParam));
  ASSERT_STREQ("a", stringParam.c_str());

  param::set(ns + "/foo", std::string("b"));
  ASSERT_TRUE(param::getCached(ns + "/foo", stringParam));
  ASSERT_STREQ("b", stringParam.c_str());
  ASSERT_TRUE(param::getCached(ns, structParam));
  ASSERT_TRUE(structParam.hasMember("foo"));
  ASSERT_STREQ("b", static_cast<std::string>(structParam["foo"]).c_str());
}

TEST(Params, setThenGetCString)
{
  param::set( "test_set_param", "asdf" );
  std::string param;
  ASSERT_TRUE( param::get( "test_set_param", param ) );
  ASSERT_STREQ( "asdf", param.c_str() );
}

TEST(Params, setThenGetInt)
{
  param::set( "test_set_param", 42);
  int param;
  ASSERT_TRUE( param::get( "test_set_param", param ) );
  ASSERT_EQ( 42, param );
  XmlRpc::XmlRpcValue v;
  param::get("test_set_param", v);
  ASSERT_EQ(v.getType(), XmlRpc::XmlRpcValue::TypeInt);
}

TEST(Params, unknownParam)
{
  std::string param;
  ASSERT_FALSE( param::get( "this_param_really_should_not_exist", param ) );
}

TEST(Params, deleteParam)
{
  param::set( "test_delete_param", "asdf" );
  param::del( "test_delete_param" );
  std::string param;
  ASSERT_FALSE( param::get( "test_delete_param", param ) );
}

TEST(Params, hasParam)
{
  ASSERT_TRUE( param::has( "string" ) );
}

TEST(Params, setIntDoubleGetInt)
{
  param::set("test_set_int_as_double", 1);
  param::set("test_set_int_as_double", 3.0f);

  int i = -1;
  ASSERT_TRUE(param::get("test_set_int_as_double", i));
  ASSERT_EQ(3, i);
  double d = 0.0f;
  ASSERT_TRUE(param::get("test_set_int_as_double", d));
  ASSERT_EQ(3.0, d);
}

TEST(Params, getIntAsDouble)
{
  param::set("int_param", 1);
  double d = 0.0;
  ASSERT_TRUE(param::get("int_param", d));
  ASSERT_EQ(1.0, d);
}

TEST(Params, getDoubleAsInt)
{
  param::set("double_param", 2.3);
  int i = -1;
  ASSERT_TRUE(param::get("double_param", i));
  ASSERT_EQ(2, i);

  param::set("double_param", 3.8);
  i = -1;
  ASSERT_TRUE(param::get("double_param", i));
  ASSERT_EQ(4, i);
}

TEST(Params, searchParam)
{
  std::string ns = "/a/b/c/d/e/f";
  std::string result;

  param::set("/s_i", 1);
  ASSERT_TRUE(param::search(ns, "s_i", result));
  ASSERT_STREQ(result.c_str(), "/s_i");
  param::del("/s_i");

  param::set("/a/b/s_i", 1);
  ASSERT_TRUE(param::search(ns, "s_i", result));
  ASSERT_STREQ(result.c_str(), "/a/b/s_i");
  param::del("/a/b/s_i");

  param::set("/a/b/c/d/e/f/s_i", 1);
  ASSERT_TRUE(param::search(ns, "s_i", result));
  ASSERT_STREQ(result.c_str(), "/a/b/c/d/e/f/s_i");
  param::del("/a/b/c/d/e/f/s_i");

  bool cont = true;
  while (!cont)
  {
  	ros::WallDuration(0.1).sleep();
  }

  ASSERT_FALSE(param::search(ns, "s_j", result));
}

TEST(Params, searchParamNodeHandle)
{
  NodeHandle n("/a/b/c/d/e/f");
  std::string result;

  n.setParam("/s_i", 1);
  ASSERT_TRUE(n.searchParam("s_i", result));
  ASSERT_STREQ(result.c_str(), "/s_i");
  n.deleteParam("/s_i");

  n.setParam("/a/b/s_i", 1);
  ASSERT_TRUE(n.searchParam("s_i", result));
  ASSERT_STREQ(result.c_str(), "/a/b/s_i");
  n.deleteParam("/a/b/s_i");

  n.setParam("/a/b/c/d/e/f/s_i", 1);
  ASSERT_TRUE(n.searchParam("s_i", result));
  ASSERT_STREQ(result.c_str(), "/a/b/c/d/e/f/s_i");
  n.deleteParam("/a/b/c/d/e/f/s_i");

  ASSERT_FALSE(n.searchParam("s_j", result));
}

TEST(Params, searchParamNodeHandleWithRemapping)
{
  M_string remappings;
  remappings["s_c"] = "s_b";
  NodeHandle n("/a/b/c/d/e/f", remappings);
  std::string result;

  n.setParam("/s_c", 1);
  ASSERT_FALSE(n.searchParam("s_c", result));
  n.setParam("/s_b", 1);
  ASSERT_TRUE(n.searchParam("s_c", result));
}

// See ROS ticket #2381
TEST(Params, getMissingXmlRpcValueParameterCachedTwice)
{
  XmlRpc::XmlRpcValue v;
  ASSERT_FALSE(ros::param::getCached("invalid_xmlrpcvalue_param", v));
  ASSERT_FALSE(ros::param::getCached("invalid_xmlrpcvalue_param", v));
}

// See ROS ticket #2353
TEST(Params, doublePrecision)
{
  ros::param::set("bar", 0.123456789123456789);
  double d;
  ASSERT_TRUE(ros::param::get("bar", d));
  EXPECT_DOUBLE_EQ(d, 0.12345678912345678);
}

std::vector<std::string> vec_s, vec_s2;
std::vector<double> vec_d, vec_d2;
std::vector<float> vec_f, vec_f2;
std::vector<int> vec_i, vec_i2;
std::vector<bool> vec_b, vec_b2;

TEST(Params, vectorStringParam)
{
  const std::string param_name = "vec_str_param";

  vec_s.clear();
  vec_s.push_back("foo");
  vec_s.push_back("bar");
  vec_s.push_back("baz");

  ros::param::set(param_name, vec_s);

  ASSERT_FALSE(ros::param::get(param_name, vec_d));
  ASSERT_FALSE(ros::param::get(param_name, vec_f));
  ASSERT_FALSE(ros::param::get(param_name, vec_i));
  ASSERT_FALSE(ros::param::get(param_name, vec_b));

  ASSERT_TRUE(ros::param::get(param_name, vec_s2));

  ASSERT_EQ(vec_s.size(), vec_s2.size());
  ASSERT_TRUE(std::equal(vec_s.begin(), vec_s.end(), vec_s2.begin()));

  // Test empty vector
  vec_s.clear();
  ros::param::set(param_name, vec_s);
  ASSERT_TRUE(ros::param::get(param_name, vec_s2));
  ASSERT_EQ(vec_s.size(), vec_s2.size());
}

TEST(Params, vectorDoubleParam)
{
  const std::string param_name = "vec_double_param";

  vec_d.clear();
  vec_d.push_back(-0.123456789);
  vec_d.push_back(3);
  vec_d.push_back(3.01);
  vec_d.push_back(7.01);

  ros::param::set(param_name, vec_d);

  ASSERT_FALSE(ros::param::get(param_name, vec_s));
  ASSERT_TRUE(ros::param::get(param_name, vec_i));
  ASSERT_TRUE(ros::param::get(param_name, vec_b));
  ASSERT_TRUE(ros::param::get(param_name, vec_f));

  ASSERT_TRUE(ros::param::get(param_name, vec_d2));

  ASSERT_EQ(vec_d.size(), vec_d2.size());
  ASSERT_TRUE(std::equal(vec_d.begin(), vec_d.end(), vec_d2.begin()));
}

TEST(Params, vectorFloatParam)
{
  const std::string param_name = "vec_float_param";

  vec_f.clear();
  vec_f.push_back(-0.123456789);
  vec_f.push_back(0.0);
  vec_f.push_back(3);
  vec_f.push_back(3.01);

  ros::param::set(param_name, vec_f);

  ASSERT_FALSE(ros::param::get(param_name, vec_s));
  ASSERT_TRUE(ros::param::get(param_name, vec_i));
  ASSERT_TRUE(ros::param::get(param_name, vec_b));
  ASSERT_TRUE(ros::param::get(param_name, vec_d));

  ASSERT_EQ(vec_b[0],true);
  ASSERT_EQ(vec_b[1],false);

  ASSERT_TRUE(ros::param::get(param_name, vec_f2));

  ASSERT_EQ(vec_f.size(), vec_f2.size());
  ASSERT_TRUE(std::equal(vec_f.begin(), vec_f.end(), vec_f2.begin()));
}

TEST(Params, vectorIntParam)
{
  const std::string param_name = "vec_int_param";

  vec_i.clear();
  vec_i.push_back(-1);
  vec_i.push_back(0);
  vec_i.push_back(1337);
  vec_i.push_back(2);

  ros::param::set(param_name, vec_i);

  ASSERT_FALSE(ros::param::get(param_name, vec_s));
  ASSERT_TRUE(ros::param::get(param_name, vec_d));
  ASSERT_TRUE(ros::param::get(param_name, vec_f));
  ASSERT_TRUE(ros::param::get(param_name, vec_b));

  ASSERT_EQ(vec_b[0],true);
  ASSERT_EQ(vec_b[1],false);

  ASSERT_TRUE(ros::param::get(param_name, vec_i2));

  ASSERT_EQ(vec_i.size(), vec_i2.size());
  ASSERT_TRUE(std::equal(vec_i.begin(), vec_i.end(), vec_i2.begin()));
}

TEST(Params, vectorBoolParam)
{
  const std::string param_name = "vec_bool_param";

  vec_b.clear();
  vec_b.push_back(true);
  vec_b.push_back(false);
  vec_b.push_back(true);
  vec_b.push_back(true);

  ros::param::set(param_name, vec_b);

  ASSERT_FALSE(ros::param::get(param_name, vec_s));
  ASSERT_TRUE(ros::param::get(param_name, vec_d));
  ASSERT_TRUE(ros::param::get(param_name, vec_f));
  ASSERT_TRUE(ros::param::get(param_name, vec_i));

  ASSERT_EQ(vec_i[0],1);
  ASSERT_EQ(vec_i[1],0);

  ASSERT_TRUE(ros::param::get(param_name, vec_b2));

  ASSERT_EQ(vec_b.size(), vec_b2.size());
  ASSERT_TRUE(std::equal(vec_b.begin(), vec_b.end(), vec_b2.begin()));
}

std::map<std::string,std::string> map_s, map_s2;
std::map<std::string,double> map_d, map_d2;
std::map<std::string,float> map_f, map_f2;
std::map<std::string,int> map_i, map_i2;
std::map<std::string,bool> map_b, map_b2;

TEST(Params, mapStringParam)
{
  const std::string param_name = "map_str_param";

  map_s.clear();
  map_s["a"] = "apple";
  map_s["b"] = "blueberry";
  map_s["c"] = "carrot";

  ros::param::set(param_name, map_s);

  ASSERT_FALSE(ros::param::get(param_name, map_d));
  ASSERT_FALSE(ros::param::get(param_name, map_f));
  ASSERT_FALSE(ros::param::get(param_name, map_i));
  ASSERT_FALSE(ros::param::get(param_name, map_b));

  ASSERT_TRUE(ros::param::get(param_name, map_s2));

  ASSERT_EQ(map_s.size(), map_s2.size());
  ASSERT_TRUE(std::equal(map_s.begin(), map_s.end(), map_s2.begin()));
}

TEST(Params, mapDoubleParam)
{
  const std::string param_name = "map_double_param";

  map_d.clear();
  map_d["a"] = 0.0;
  map_d["b"] = -0.123456789;
  map_d["c"] = 123456789;

  ros::param::set(param_name, map_d);

  ASSERT_FALSE(ros::param::get(param_name, map_s));
  ASSERT_TRUE(ros::param::get(param_name, map_f));
  ASSERT_TRUE(ros::param::get(param_name, map_i));
  ASSERT_TRUE(ros::param::get(param_name, map_b));

  ASSERT_TRUE(ros::param::get(param_name, map_d2));

  ASSERT_EQ(map_d.size(), map_d2.size());
  ASSERT_TRUE(std::equal(map_d.begin(), map_d.end(), map_d2.begin()));
}

TEST(Params, mapFloatParam)
{
  const std::string param_name = "map_float_param";

  map_f.clear();
  map_f["a"] = 0.0;
  map_f["b"] = -0.123456789;
  map_f["c"] = 123456789;

  ros::param::set(param_name, map_f);

  ASSERT_FALSE(ros::param::get(param_name, map_s));
  ASSERT_TRUE(ros::param::get(param_name, map_d));
  ASSERT_TRUE(ros::param::get(param_name, map_i));
  ASSERT_TRUE(ros::param::get(param_name, map_b));

  ASSERT_TRUE(ros::param::get(param_name, map_f2));

  ASSERT_EQ(map_f.size(), map_f2.size());
  ASSERT_TRUE(std::equal(map_f.begin(), map_f.end(), map_f2.begin()));
}

TEST(Params, mapIntParam)
{
  const std::string param_name = "map_int_param";

  map_i.clear();
  map_i["a"] = 0;
  map_i["b"] = -1;
  map_i["c"] = 1337;

  ros::param::set(param_name, map_i);

  ASSERT_FALSE(ros::param::get(param_name, map_s));
  ASSERT_TRUE(ros::param::get(param_name, map_d));
  ASSERT_TRUE(ros::param::get(param_name, map_f));
  ASSERT_TRUE(ros::param::get(param_name, map_b));

  ASSERT_TRUE(ros::param::get(param_name, map_i2));

  ASSERT_EQ(map_i.size(), map_i2.size());
  ASSERT_TRUE(std::equal(map_i.begin(), map_i.end(), map_i2.begin()));
}

TEST(Params, mapBoolParam)
{
  const std::string param_name = "map_bool_param";

  map_b.clear();
  map_b["a"] = true;
  map_b["b"] = false;
  map_b["c"] = true;

  ros::param::set(param_name, map_b);

  ASSERT_FALSE(ros::param::get(param_name, map_s));
  ASSERT_TRUE(ros::param::get(param_name, map_d));
  ASSERT_TRUE(ros::param::get(param_name, map_f));
  ASSERT_TRUE(ros::param::get(param_name, map_i));

  ASSERT_EQ(map_i["a"],1);
  ASSERT_EQ(map_i["b"],0);

  ASSERT_TRUE(ros::param::get(param_name, map_b2));

  ASSERT_EQ(map_b.size(), map_b2.size());
  ASSERT_TRUE(std::equal(map_b.begin(), map_b.end(), map_b2.begin()));
}

TEST(Params, paramTemplateFunction)
{
  EXPECT_EQ( param::param<std::string>( "string", "" ), "test" );
  EXPECT_EQ( param::param<std::string>( "gnirts", "test" ), "test" );

  EXPECT_EQ( param::param<int>( "int", 0 ), 10 );
  EXPECT_EQ( param::param<int>( "tni", 10 ), 10 );

  EXPECT_DOUBLE_EQ( param::param<double>( "double", 0.0 ), 10.5 );
  EXPECT_DOUBLE_EQ( param::param<double>( "elbuod", 10.5 ), 10.5 );

  EXPECT_EQ( param::param<bool>( "bool", true ), false );
  EXPECT_EQ( param::param<bool>( "loob", true ), true );
}

TEST(Params, paramNodeHandleTemplateFunction)
{
  NodeHandle nh;

  EXPECT_EQ( nh.param<std::string>( "string", "" ), "test" );
  EXPECT_EQ( nh.param<std::string>( "gnirts", "test" ), "test" );

  EXPECT_EQ( nh.param<int>( "int", 0 ), 10 );
  EXPECT_EQ( nh.param<int>( "tni", 10 ), 10 );

  EXPECT_DOUBLE_EQ( nh.param<double>( "double", 0.0 ), 10.5 );
  EXPECT_DOUBLE_EQ( nh.param<double>( "elbuod", 10.5 ), 10.5 );

  EXPECT_EQ( nh.param<bool>( "bool", true ), false );
  EXPECT_EQ( nh.param<bool>( "loob", true ), true );
}

TEST(Params, getParamNames) {
  std::vector<std::string> test_params;
  EXPECT_TRUE(ros::param::getParamNames(test_params));
  EXPECT_LT(10u, test_params.size());
}

TEST(Params, getParamCachedSetParamLoop) {
  NodeHandle nh;
  const std::string name = "changeable_int";
  for (int i = 0; i < 100; i++) {
    nh.setParam(name, i);
    int v = 0;
    ASSERT_TRUE(nh.getParamCached(name, v));
    ASSERT_EQ(i, v);
  }
}

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "params");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
