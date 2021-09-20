#include "hello.h"

#include <glog/logging.h>
#include <iostream>

void sayHello() {
    LOG(INFO) << "Hello SLAM GLOG";
    std::cout<<"Hello SLAM"<<std::endl;
    }
