/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, Open Source Robotics Foundation
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
*********************************************************************/

#include "rosbag/gpgme_utils.h"

#include <boost/format.hpp>

namespace rosbag
{

void initGpgme() {
    // Check version method must be called before en/decryption
    gpgme_check_version(0);
    // Set locale
    setlocale(LC_ALL, "");
    gpgme_set_locale(NULL, LC_CTYPE, setlocale(LC_CTYPE, NULL));
#ifdef LC_MESSAGES
    gpgme_set_locale(NULL, LC_MESSAGES, setlocale(LC_MESSAGES, NULL));
#endif
}

void getGpgKey(gpgme_ctx_t& ctx, std::string const& user, gpgme_key_t& key) {
    gpgme_error_t err;
    // Asterisk means an arbitrary user.
    if (user == std::string("*")) {
        err = gpgme_op_keylist_start(ctx, 0, 0);
    } else {
        err = gpgme_op_keylist_start(ctx, user.c_str(), 0);
    }
    if (err) {
        throw BagException((boost::format("gpgme_op_keylist_start returned %1%") % gpgme_strerror(err)).str());
    }
    while (true) {
        err = gpgme_op_keylist_next(ctx, &key);
        if (!err) {
            if (user == std::string("*") || strcmp(key->uids->name, user.c_str()) == 0) {
                break;
            }
            gpgme_key_release(key);
        } else if (gpg_err_code(err) == GPG_ERR_EOF) {
            if (user == std::string("*")) {
                // A method throws an exception (instead of returning a specific value) if the key is not found
                // This allows rosbag client applications to work without modifying their source code
                throw BagException("GPG key not found");
            } else {
                throw BagException((boost::format("GPG key not found for a user %1%") % user.c_str()).str());
            }
        } else {
            throw BagException((boost::format("gpgme_op_keylist_next returned %1%") % err).str());
        }
    }
    err = gpgme_op_keylist_end(ctx);
    if (err) {
        throw BagException((boost::format("gpgme_op_keylist_end returned %1%") % gpgme_strerror(err)).str());
    }
}


}  // namespace rosbag
