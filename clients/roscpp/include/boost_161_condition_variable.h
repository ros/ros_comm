#ifndef BOOST_161_THREAD_CONDITION_VARIABLE_HPP
#define BOOST_161_THREAD_CONDITION_VARIABLE_HPP

//  condition_variable.hpp
//
//  (C) Copyright 2007 Anthony Williams
//
//  Distributed under the Boost Software License, Version 1.0. (See
//  accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)

#include <boost/thread/detail/platform.hpp>
#if defined(BOOST_THREAD_PLATFORM_WIN32)
#include <boost/thread/win32/condition_variable.hpp>
namespace boost_161 {
  using condition_variable = boost::condition_variable;
}
#elif defined(BOOST_THREAD_PLATFORM_PTHREAD)
//#include <boost/thread/pthread/condition_variable.hpp>
#include "boost_161_pthread_condition_variable.h"
#else
#error "Boost threads unavailable on this platform"
#endif

#endif

