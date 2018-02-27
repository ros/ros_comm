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

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "../include/test_perf_benchmark/common.h"

#include <sstream>
#include <string>
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/file.h>
#include <vector>
#include <algorithm>

using namespace std;

static int shm_fd_ = -1;
static ShmData* addr_;

static void WaitSubscribers(int _sub_num)
{
  while (true)
  {
    flock(shm_fd_, LOCK_EX);
    int need_subscriber_count = addr_->need_subscriber_count;
    flock(shm_fd_, LOCK_UN);
    if (need_subscriber_count == 0)
    {
      break;
    }
    usleep(100);
  }

  flock(shm_fd_, LOCK_EX);
  addr_->need_subscriber_count = _sub_num;
  flock(shm_fd_, LOCK_UN);
}

static void PrintResult(const vector<double> transmit_durations)
{
  if (transmit_durations.size() != 0)
  {
    auto result = minmax_element(transmit_durations.begin(), transmit_durations.end());
    auto sum = accumulate(transmit_durations.begin(), transmit_durations.end(), .0f);
    cout << right << setw(25) << "min(s): " << fixed << setprecision(9) << *result.first << endl;
    cout << right << setw(25) << "max(s): " << fixed << setprecision(9) << *result.second << endl;
    cout << right << setw(25) << "count: " << transmit_durations.size() << endl;
    cout << right << setw(25) << "sum(s): " << fixed << setprecision(9) << sum << endl;
    cout << right << setw(25) << "average(s): " << fixed << setprecision(9)
    << sum / transmit_durations.size() << endl;
  }
}

int main(int argc, char **argv)
{
  long int buffer_size;
  struct timespec ts_transmission_start;
  vector<double> transmission_durations_;

  if (argc != 4)
  {
    cout << "usage:\n\t./talker [number of test loop count] [message size(choose in 256B|4KB|256KB|2M|8MB)] [number of subscriber]\n" << endl;
    return EXIT_FAILURE;
  }

  //message size control
  if (strcmp(argv[2],"256B") == 0) {
    buffer_size = 256;
  } else if (strcmp(argv[2],"4KB") ==0) {
    buffer_size = 4 * 1024;
  } else if (strcmp(argv[2],"256KB") == 0) {
    buffer_size = 256 * 1024;
  } else if (strcmp(argv[2],"2MB") == 0) {
    buffer_size = 2 * 1024 * 1024;
  } else if (strcmp(argv[2],"8MB") == 0) {
    buffer_size = 8 * 1024 * 1024;
  } else {
    cout << "usage:\n\t./talker [number of test loop count] [message size(choose in 256B|4KB|256KB|2M|8MB)] [number of subscriber]\n" << endl;
    return EXIT_FAILURE;
  }
  int loop_count = atoi(argv[1]);
  int sub_num = atoi(argv[3]);

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter_perf_test", 1000);

  ros::Rate loop_rate(2);

  // setup shm data
  shm_fd_ = open(SHM_MAP_FILE, O_RDWR | O_CREAT, FILE_PERM);
  if (shm_fd_ < 0)
  {
    ROS_ERROR("open file %s failed", SHM_MAP_FILE);
    return EXIT_FAILURE;
  }

  int size = sizeof (ShmData);
  int sysret = fallocate(shm_fd_, 0, 0, (off_t) size);
  if (sysret != 0)
  {
    if (errno == ENOSYS || errno == EOPNOTSUPP)
    {
      sysret = posix_fallocate(shm_fd_, 0, (off_t) size);
    }
  }
  if (sysret < 0)
  {
    ROS_ERROR("fallocate error");
    return EXIT_FAILURE;
  }

  addr_ = (ShmData*) mmap(NULL, (size_t) size, PROT_WRITE | PROT_READ,
                      MAP_SHARED | MAP_POPULATE, shm_fd_, 0);
  if (addr_ == NULL)
  {
    ROS_ERROR("mmap error");
    return EXIT_FAILURE;
  }
  addr_->msg_size = buffer_size;
  addr_->need_subscriber_count = sub_num;
  addr_->send_times = loop_count;

  // message to be transport
  std_msgs::String msg;
  stringstream s_stream;
  char *message;
  message = new char[buffer_size];
  memset(message, '0', buffer_size);
  message[buffer_size - 1] = '\0';
  s_stream << message;
  msg.data = s_stream.str();

  for (int i = 0; i <= loop_count; i++)
  {
    WaitSubscribers(sub_num);

    if (i == 0)
    {
      usleep(1000000);
    } else {
      flock(shm_fd_, LOCK_EX);
      double duration = (addr_->ts_transmission_end.tv_sec - ts_transmission_start.tv_sec)
          + double(addr_->ts_transmission_end.tv_nsec - ts_transmission_start.tv_nsec) / 1000000000;
      flock(shm_fd_, LOCK_UN);
      transmission_durations_.push_back(duration);
      if (i == loop_count)
      {
        break;
      }
    }

    timespec_get(&ts_transmission_start, TIME_UTC);

    chatter_pub.publish(msg);

    ros::spinOnce();
  }

  PrintResult(transmission_durations_);

  if (addr_ != NULL)
  {
    munmap(addr_, (size_t) size);
  }

  if (shm_fd_ != -1)
  {
    close(shm_fd_);
    unlink(SHM_MAP_FILE);
  }

  delete [] message;

  return 0;
}
