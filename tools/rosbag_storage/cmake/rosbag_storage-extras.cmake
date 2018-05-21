# PR https://github.com/ros/ros_comm/pull/1206 added support for encryption
# and decryption of rosbags by utilizing libgpgme-dev.  On armhf, libgpgme-dev
# is compiled with -D_FILE_OFF_BITS=64, which means that all downstream
# consumers of it must also be compiled with that
# (https://www.gnupg.org/documentation/manuals/gpgme/Largefile-Support-_0028LFS_0029.html
# has some more information).  Add that flag to the CMAKE_CXX_FLAGS for all
# architectures where the size of the pointer is less than 8 bytes.
if(CMAKE_SIZEOF_VOID_P LESS 8)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_FILE_OFFSET_BITS=64")
endif()
