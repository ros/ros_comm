/*
 * Sony CONFIDENTIAL
 *
 * Copyright 2018 Sony Corporation
 *
 * DO NOT COPY AND/OR REDISTRIBUTE WITHOUT PERMISSION.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include "ros/transport/transport_uds.h"
#include <sstream>
#include <boost/filesystem.hpp>
#include <string>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/lexical_cast.hpp>
#include "ros/file_log.h"

namespace ros
{

uint32_t TransportUDS::s_uds_feature_ = 0;

const std::string TransportUDS::generateServerUDSPath()
{
  std::string uuid_str;

  boost::uuids::uuid uuid = boost::uuids::random_generator()();
  uuid_str = boost::lexical_cast<std::string>(uuid);
  ROSCPP_LOG_DEBUG("Abstract Socket Name is %s", uuid_str.c_str());
  return uuid_str.c_str();
}

const std::string TransportUDS::generateServerUDSPath(uint32_t counter)
{
  std::stringstream str;
  str << boost::filesystem::temp_directory_path().string() << server_type_ << (int)getpid() << "-" << counter;
  return str.str();
}

}
