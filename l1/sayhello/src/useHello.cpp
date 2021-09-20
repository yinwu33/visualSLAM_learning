#include "hello.h"
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <gtest/gtest.h>

DEFINE_int32(num, 0, "say hello times");

int main( int argc, char** argv ) {
  // gflags
  google::ParseCommandLineFlags(&argc, &argv, true);
  int num = FLAGS_num;

  // glog
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;

  for(int i = 0; i < num; ++i)
    sayHello();

  LOG(INFO) << "here is glog";

  return 0;
}
