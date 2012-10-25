/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
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

#ifndef ROSCPP_COMMON_H
#define ROSCPP_COMMON_H

#include <stdint.h>
#include <assert.h>
#include <stddef.h>
#include <string>

#include "ros/assert.h"
#include "ros/forwards.h"
#include "ros/serialized_message.h"

#include <boost/shared_array.hpp>

#define ROS_VERSION_MAJOR roscpp_VERSION_MAJOR
#define ROS_VERSION_MINOR roscpp_VERSION_MINOR
#define ROS_VERSION_PATCH roscpp_VERSION_PATCH
#define ROS_VERSION_COMBINED(major, minor, patch) (((major) << 20) | ((minor) << 10) | (patch))
#define ROS_VERSION ROS_VERSION_COMBINED(ROS_VERSION_MAJOR, ROS_VERSION_MINOR, ROS_VERSION_PATCH)

#define ROS_VERSION_GE(major1, minor1, patch1, major2, minor2, patch2) (ROS_VERSION_COMBINED(major1, minor1, patch1) >= ROS_VERSION_COMBINED(major2, minor2, patch2))
#define ROS_VERSION_MINIMUM(major, minor, patch) ROS_VERSION_GE(ROS_VERSION_MAJOR, ROS_VERSION_MINOR, ROS_VERSION_PATCH, major, minor, patch)

#include <ros/macros.h>

// Import/export for windows dll's and visibility for gcc shared libraries.

#ifdef ROS_BUILD_SHARED_LIBS // ros is being built around shared libraries
  #ifdef roscpp_EXPORTS // we are building a shared lib/dll
    #define ROSCPP_DECL ROS_HELPER_EXPORT
  #else // we are using shared lib/dll
    #define ROSCPP_DECL ROS_HELPER_IMPORT
  #endif
#else // ros is being built around static libraries
  #define ROSCPP_DECL
#endif

namespace ros
{

void disableAllSignalsInThisThread();

}

#endif

