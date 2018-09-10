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

#include "rosbag/bag.h"
#include "rosbag/aes_encryptor.h"
#include "rosbag/gpgme_utils.h"

#include <openssl/rand.h>

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rosbag::AesCbcEncryptor, rosbag::EncryptorBase)

namespace rosbag
{

const std::string AesCbcEncryptor::GPG_USER_FIELD_NAME = "gpg_user";
const std::string AesCbcEncryptor::ENCRYPTED_KEY_FIELD_NAME = "encrypted_key";

//! Encrypt string using GPGME
/*!
 * \return Encrypted string
 * \param user User name of the GPG key to be used for encryption
 * \param input Input string to be encrypted
 *
 * This method encrypts the given string using the GPG key owned by the specified user.
 * This method throws BagException in case of errors.
 */
static std::string encryptStringGpg(std::string& user, std::basic_string<unsigned char> const& input) {
    gpgme_ctx_t ctx;
    gpgme_error_t err = gpgme_new(&ctx);
    if (err) {
        throw BagException((boost::format("Failed to create a GPG context: %1%") % gpgme_strerror(err)).str());
    }

    gpgme_key_t keys[2] = {NULL, NULL};
    getGpgKey(ctx, user, keys[0]);
    if (user == std::string("*")) {
        user = std::string(keys[0]->uids->name);
    }

    gpgme_data_t input_data;
    err = gpgme_data_new_from_mem(&input_data, reinterpret_cast<const char*>(input.c_str()), input.length(), 1);
    if (err) {
        gpgme_release(ctx);
        throw BagException(
            (boost::format("Failed to encrypt string: gpgme_data_new_from_mem returned %1%") % gpgme_strerror(err)).str());
    }
    gpgme_data_t output_data;
    err = gpgme_data_new(&output_data);
    if (err) {
        gpgme_data_release(input_data);
        gpgme_release(ctx);
        throw BagException(
            (boost::format("Failed to encrypt string: gpgme_data_new returned %1%") % gpgme_strerror(err)).str());
    }
    err = gpgme_op_encrypt(ctx, keys, static_cast<gpgme_encrypt_flags_t>(GPGME_ENCRYPT_ALWAYS_TRUST), input_data, output_data);
    if (err) {
        gpgme_data_release(output_data);
        gpgme_data_release(input_data);
        gpgme_release(ctx);
        throw BagException((boost::format("Failed to encrypt: %1%.  Have you installed a public key %2%?") % gpgme_strerror(err) % user).str());
    }
    gpgme_key_release(keys[0]);
    std::size_t output_length = gpgme_data_seek(output_data, 0, SEEK_END);
    std::string output(output_length, 0);
    gpgme_data_seek(output_data, 0, SEEK_SET);
    ssize_t bytes_read = gpgme_data_read(output_data, &output[0], output_length);
    // Release resources and return
    gpgme_data_release(output_data);
    gpgme_data_release(input_data);
    gpgme_release(ctx);
    if (-1 == bytes_read) {
        throw BagException("Failed to read encrypted string");
    }
    return output;
}

//! Decrypt string using GPGME
/*!
 * \return Decrypted string
 * \param user User name of the GPG key to be used for decryption
 * \param input Encrypted string
 *
 * This method decrypts the given encrypted string. This method throws BagException in case of errors.
 */
static std::basic_string<unsigned char> decryptStringGpg(std::string const& user, std::string const& input) {
    gpgme_ctx_t ctx;
    gpgme_error_t err = gpgme_new(&ctx);
    if (err) {
        throw BagException((boost::format("Failed to create a GPG context: %1%") % gpgme_strerror(err)).str());
    }

    gpgme_data_t input_data;
    err = gpgme_data_new_from_mem(&input_data, input.c_str(), input.length(), 1);
    if (err) {
        gpgme_release(ctx);
        throw BagException(
            (boost::format("Failed to decrypt bag: gpgme_data_new_from_mem returned %1%") % gpgme_strerror(err)).str());
    }
    gpgme_data_t output_data;
    err = gpgme_data_new(&output_data);
    if (err) {
        gpgme_data_release(input_data);
        gpgme_release(ctx);
        throw BagException(
            (boost::format("Failed to decrypt bag: gpgme_data_new returned %1%") % gpgme_strerror(err)).str());
    }
    err = gpgme_op_decrypt(ctx, input_data, output_data);
    if (err) {
        gpgme_data_release(output_data);
        gpgme_data_release(input_data);
        gpgme_release(ctx);
        throw BagException((boost::format("Failed to decrypt bag: %1%.  Have you installed a private key %2%?") % gpgme_strerror(err) % user).str());
    }
    std::size_t output_length = gpgme_data_seek(output_data, 0, SEEK_END);
    if (output_length != AES_BLOCK_SIZE) {
        gpgme_data_release(output_data);
        gpgme_data_release(input_data);
        gpgme_release(ctx);
        throw BagException("Decrypted string length mismatches");
    }
    std::basic_string<unsigned char> output(output_length, 0);
    gpgme_data_seek(output_data, 0, SEEK_SET);
    ssize_t bytes_read = gpgme_data_read(output_data, reinterpret_cast<char*>(&output[0]), output_length);
    // Release resources and return
    gpgme_data_release(output_data);
    gpgme_data_release(input_data);
    gpgme_release(ctx);
    if (-1 == bytes_read) {
        throw BagException("Failed to read decrypted symmetric key");
    }
    return output;
}

static std::string readHeaderField(ros::M_string const& header_fields, std::string const& field_name) {
    ros::M_string::const_iterator it = header_fields.find(field_name);
    if (it == header_fields.end()) {
        return std::string();
    }
    return it->second;
}

void AesCbcEncryptor::initialize(Bag const& bag, std::string const& gpg_key_user) {
    // GPGME must be initialized even when reading
    initGpgme();
    // Encryption user can be set only when writing a bag file
    if (bag.getMode() != bagmode::Write) {
        return;
    }
    if (gpg_key_user_ == gpg_key_user) {
        return;
    }
    if (gpg_key_user_.empty()) {
        gpg_key_user_ = gpg_key_user;
        buildSymmetricKey();
        AES_set_encrypt_key(&symmetric_key_[0], AES_BLOCK_SIZE*8, &aes_encrypt_key_);
    } else {
        // Encryption user cannot change once set
        throw BagException(
            (boost::format("Encryption user has already been set to %s") % gpg_key_user_.c_str()).str());
    }
}

uint32_t AesCbcEncryptor::encryptChunk(const uint32_t chunk_size, const uint64_t chunk_data_pos, ChunkedFile& file) {
    // Read existing (compressed) chunk
    std::basic_string<unsigned char> compressed_chunk(chunk_size, 0);
    file.seek(chunk_data_pos);
    file.read((char*) &compressed_chunk[0], chunk_size);
    // Apply PKCS#7 padding to the chunk
    std::size_t pad_size = AES_BLOCK_SIZE - chunk_size % AES_BLOCK_SIZE;
    compressed_chunk.resize(compressed_chunk.length() + pad_size, pad_size);
    // Encrypt chunk
    std::basic_string<unsigned char> encrypted_chunk(compressed_chunk.length(), 0);
    std::basic_string<unsigned char> iv(AES_BLOCK_SIZE, 0);
    if (!RAND_bytes(&iv[0], AES_BLOCK_SIZE)) {
        throw BagException("Failed to build initialization vector");
    }
    file.seek(chunk_data_pos);
    file.write((char*) &iv[0], AES_BLOCK_SIZE);
    AES_cbc_encrypt(&compressed_chunk[0], &encrypted_chunk[0], encrypted_chunk.length(), &aes_encrypt_key_, &iv[0], AES_ENCRYPT);
    // Write encrypted chunk
    file.write((char*) &encrypted_chunk[0], encrypted_chunk.length());
    file.truncate(chunk_data_pos + AES_BLOCK_SIZE + encrypted_chunk.length());
    return AES_BLOCK_SIZE + encrypted_chunk.length();
}

void AesCbcEncryptor::decryptChunk(ChunkHeader const& chunk_header, Buffer& decrypted_chunk, ChunkedFile& file) const {
    // Test encrypted chunk size
    if (chunk_header.compressed_size % AES_BLOCK_SIZE != 0) {
        throw BagFormatException((boost::format("Error in encrypted chunk size: %d") % chunk_header.compressed_size).str());
    }
    // Read encrypted chunk
    if (chunk_header.compressed_size < AES_BLOCK_SIZE) {
        throw BagFormatException((boost::format("No initialization vector in encrypted chunk: %d") % chunk_header.compressed_size).str());
    }
    std::basic_string<unsigned char> iv(AES_BLOCK_SIZE, 0);
    file.read((char*) &iv[0], AES_BLOCK_SIZE);
    std::basic_string<unsigned char> encrypted_chunk(chunk_header.compressed_size - AES_BLOCK_SIZE, 0);
    file.read((char*) &encrypted_chunk[0], chunk_header.compressed_size - AES_BLOCK_SIZE);
    // Decrypt chunk
    decrypted_chunk.setSize(chunk_header.compressed_size - AES_BLOCK_SIZE);
    AES_cbc_encrypt(&encrypted_chunk[0], (unsigned char*) decrypted_chunk.getData(), chunk_header.compressed_size - AES_BLOCK_SIZE,
        &aes_decrypt_key_, &iv[0], AES_DECRYPT);
    if (decrypted_chunk.getSize() == 0) {
        throw BagFormatException("Decrypted chunk is empty");
    }
    decrypted_chunk.setSize(decrypted_chunk.getSize() - *(decrypted_chunk.getData()+decrypted_chunk.getSize()-1));
}

void AesCbcEncryptor::addFieldsToFileHeader(ros::M_string &header_fields) const {
    header_fields[ENCRYPTOR_FIELD_NAME] = "rosbag/AesCbcEncryptor";
    header_fields[GPG_USER_FIELD_NAME] = gpg_key_user_;
    header_fields[ENCRYPTED_KEY_FIELD_NAME] = encrypted_symmetric_key_;
}

void AesCbcEncryptor::readFieldsFromFileHeader(ros::M_string const& header_fields) {
    encrypted_symmetric_key_ = readHeaderField(header_fields, ENCRYPTED_KEY_FIELD_NAME);
    if (encrypted_symmetric_key_.empty()) {
        throw BagFormatException("Encrypted symmetric key is not found in header");
    }
    gpg_key_user_ = readHeaderField(header_fields, GPG_USER_FIELD_NAME);
    if (gpg_key_user_.empty()) {
        throw BagFormatException("GPG key user is not found in header");
    }
    symmetric_key_ = decryptStringGpg(gpg_key_user_, encrypted_symmetric_key_);
    AES_set_decrypt_key(&symmetric_key_[0], AES_BLOCK_SIZE*8, &aes_decrypt_key_);
}

void AesCbcEncryptor::writeEncryptedHeader(boost::function<void(ros::M_string const&)>, ros::M_string const& header_fields, ChunkedFile& file) {
    boost::shared_array<uint8_t> header_buffer;
    uint32_t header_len;
    ros::Header::write(header_fields, header_buffer, header_len);
    // Apply PKCS#7 padding to the header
    std::size_t pad_size = AES_BLOCK_SIZE - header_len % AES_BLOCK_SIZE;
    uint32_t encrypted_buffer_size = header_len + pad_size;
    std::basic_string<unsigned char> header_buffer_with_pad(encrypted_buffer_size, pad_size);
    memcpy(&header_buffer_with_pad[0], header_buffer.get(), header_len);
    // Encrypt chunk
    std::basic_string<unsigned char> encrypted_buffer(encrypted_buffer_size, 0);
    std::basic_string<unsigned char> iv(AES_BLOCK_SIZE, 0);
    if (!RAND_bytes(&iv[0], AES_BLOCK_SIZE)) {
        throw BagException("Failed to build initialization vector");
    }
    encrypted_buffer_size += AES_BLOCK_SIZE;
    file.write((char*) &encrypted_buffer_size, 4);
    encrypted_buffer_size -= AES_BLOCK_SIZE;
    file.write((char*) &iv[0], AES_BLOCK_SIZE);
    AES_cbc_encrypt(&header_buffer_with_pad[0], &encrypted_buffer[0], encrypted_buffer_size, &aes_encrypt_key_, &iv[0], AES_ENCRYPT);
    // Write
    file.write((char*) &encrypted_buffer[0], encrypted_buffer_size);
}

bool AesCbcEncryptor::readEncryptedHeader(boost::function<bool(ros::Header&)>, ros::Header& header, Buffer& header_buffer, ChunkedFile& file) {
    // Read the encrypted header length
    uint32_t encrypted_header_len;
    file.read((char*) &encrypted_header_len, 4);
    if (encrypted_header_len % AES_BLOCK_SIZE != 0) {
        throw BagFormatException((boost::format("Error in encrypted header length: %d") % encrypted_header_len).str());
    }
    if (encrypted_header_len < AES_BLOCK_SIZE) {
        throw BagFormatException((boost::format("No initialization vector in encrypted header: %d") % encrypted_header_len).str());
    }
    // Read encrypted header
    std::basic_string<unsigned char> iv(AES_BLOCK_SIZE, 0);
    file.read((char*) &iv[0], AES_BLOCK_SIZE);
    encrypted_header_len -= AES_BLOCK_SIZE;
    std::basic_string<unsigned char> encrypted_header(encrypted_header_len, 0);
    file.read((char*) &encrypted_header[0], encrypted_header_len);
    // Decrypt header
    header_buffer.setSize(encrypted_header_len);
    AES_cbc_encrypt(&encrypted_header[0], (unsigned char*) header_buffer.getData(), encrypted_header_len, &aes_decrypt_key_, &iv[0], AES_DECRYPT);
    if (header_buffer.getSize() == 0) {
        throw BagFormatException("Decrypted header is empty");
    }
    header_buffer.setSize(header_buffer.getSize() - *(header_buffer.getData()+header_buffer.getSize()-1));
    // Parse the header
    std::string error_msg;
    return header.parse(header_buffer.getData(), header_buffer.getSize(), error_msg);
}

void AesCbcEncryptor::buildSymmetricKey() {
    // Compose a new symmetric key for a bag file to be written
    if (gpg_key_user_.empty()) {
        return;
    }
    symmetric_key_.resize(AES_BLOCK_SIZE);
    if (!RAND_bytes(&symmetric_key_[0], AES_BLOCK_SIZE)) {
        throw BagException("Failed to build symmetric key");
    }
    // Encrypted session key is written in bag file header
    encrypted_symmetric_key_ = encryptStringGpg(gpg_key_user_, symmetric_key_);
}

}  // namespace rosbag
