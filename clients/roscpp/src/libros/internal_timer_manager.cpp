/*
 * Copyright (C) 2010, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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

// Make sure we use CLOCK_MONOTONIC for the condition variable if not Apple.
#if !defined(__APPLE__) && !defined(WIN32)
#define BOOST_THREAD_HAS_CONDATTR_SET_CLOCK_MONOTONIC
#endif

#include "ros/timer_manager.h"
#include "ros/internal_timer_manager.h"

// check if we have really included the backported boost condition variable
// just in case someone messes with the include order...
#if BOOST_VERSION < 106100
#ifndef USING_BACKPORTED_BOOST_CONDITION_VARIABLE
#error "steady timer needs boost version >= 1.61 or the backported headers!"
#endif
#endif

namespace ros
{

static InternalTimerManagerPtr g_timer_manager;

InternalTimerManagerPtr getInternalTimerManager()
{
  return g_timer_manager;
}

void initInternalTimerManager()
{
  if (!g_timer_manager)
  {
    g_timer_manager.reset(new InternalTimerManager);
  }
}

} // namespace ros
