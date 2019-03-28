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

#ifndef ROSBAG_AES_ENCRYPTION_H
#define ROSBAG_AES_ENCRYPTION_H

#include "rosbag/encryptor.h"

#ifndef _WIN32
  #include <openssl/aes.h>

namespace rosbag {

class AesCbcEncryptor : public EncryptorBase
{
public:
    static const std::string GPG_USER_FIELD_NAME;
    static const std::string ENCRYPTED_KEY_FIELD_NAME;

public:
    AesCbcEncryptor() { }
    ~AesCbcEncryptor() { }

    void initialize(Bag const& bag, std::string const& gpg_key_user);
    uint32_t encryptChunk(const uint32_t chunk_size, const uint64_t chunk_data_pos, ChunkedFile& file);
    void decryptChunk(ChunkHeader const& chunk_header, Buffer& decrypted_chunk, ChunkedFile& file) const;
    void addFieldsToFileHeader(ros::M_string& header_fields) const;
    void readFieldsFromFileHeader(ros::M_string const& header_fields);
    void writeEncryptedHeader(boost::function<void(ros::M_string const&)>, ros::M_string const& header_fields, ChunkedFile&);
    bool readEncryptedHeader(boost::function<bool(ros::Header&)>, ros::Header& header, Buffer& header_buffer, ChunkedFile&);

private:
    void buildSymmetricKey();

private:
    // User name of GPG key used for symmetric key encryption
    std::string gpg_key_user_;
    // Symmetric key for encryption/decryption
    std::basic_string<unsigned char> symmetric_key_;
    // Encrypted symmetric key
    std::string encrypted_symmetric_key_;
    // AES keys for encryption/decryption
    AES_KEY aes_encrypt_key_;
    AES_KEY aes_decrypt_key_;
};
}
#endif

#endif
