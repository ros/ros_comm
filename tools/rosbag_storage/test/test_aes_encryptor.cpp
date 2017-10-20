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

#include <cstdio>

#include <gtest/gtest.h>

#include <gpgme.h>

#include "std_msgs/String.h"

#include "rosbag/bag.h"
#include "rosbag/encryptor.h"
#include "rosbag/view.h"

const char *GPG_KEY_USER = "Foo00";
const char *GPG_PRIVATE_SUBKEY = "-----BEGIN PGP PRIVATE KEY BLOCK-----\n"
    "Version: GnuPG v1\n\n"
    "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"
    "LYxADpgShsNtJiU/Dbn8jA+BlK/77fGnkvnc"
    "=YcIw"
    "-----END PGP PRIVATE KEY BLOCK-----";
const char *MESSAGE = "message foo";
const char *TOPIC_NAME = "topic_bar";

void importGpgKey(gpgme_ctx_t &ctx) {
    gpgme_data_t key_data;
    gpgme_error_t err = gpgme_data_new_from_mem(&key_data, GPG_PRIVATE_SUBKEY, std::strlen(GPG_PRIVATE_SUBKEY), 1);
    if (err) {
        gpgme_release(ctx);
        FAIL() << "gpgme_data_new_from_mem returned " << gpgme_strerror(err);
    }
    err = gpgme_op_import(ctx, key_data);
    if (err) {
        gpgme_data_release(key_data);
        gpgme_release(ctx);
        FAIL() << "gpgme_op_import returned " << gpgme_strerror(err);
    }
    gpgme_import_result_t res = gpgme_op_import_result(ctx);
    EXPECT_TRUE(res->imports);
    gpgme_data_release(key_data);
}

TEST(AesCbcEncryptor, EncryptAndDecryptBag) {
    // Import key
    rosbag::initGpgme();
    gpgme_ctx_t ctx;
    gpgme_error_t err = gpgme_new(&ctx);
    if (err) {
        FAIL() << "Failed to create a GPG context: " << gpgme_strerror(err);
    }
    importGpgKey(ctx);

    // Test if the key has been imported
    gpgme_key_t key;
    try {
        rosbag::getGpgKey(ctx, GPG_KEY_USER, key);
    }
    catch (rosbag::BagException const& e) {
        gpgme_release(ctx);
        FAIL() << "Failed to get the imported GPG key: " << e.what();
    }

    // Write a message to an encrypted bag file
    std::string bag_file_name = std::string(tmpnam(0));
    rosbag::Bag bag(bag_file_name, rosbag::bagmode::Write);
    bag.setEncryptorPlugin("rosbag/AesCbcEncryptor", GPG_KEY_USER);
    std_msgs::String msg;
    msg.data = MESSAGE;
    bag.write(TOPIC_NAME, ros::TIME_MIN, msg);
    bag.close();

    // Test the message decrypted from the bag file
    bag.open(bag_file_name, rosbag::bagmode::Read);
    rosbag::View view(bag);
    EXPECT_EQ(view.size(), 1);
    EXPECT_EQ(TOPIC_NAME, view.begin()->getTopic());
    EXPECT_EQ(MESSAGE, view.begin()->instantiate<std_msgs::String>()->data);
    bag.close();

    // Remove the bag file
    boost::filesystem::remove(bag_file_name);
    // Delete the key
    gpgme_op_delete(ctx, key, 1);
    // Release GPG context
    gpgme_release(ctx);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
