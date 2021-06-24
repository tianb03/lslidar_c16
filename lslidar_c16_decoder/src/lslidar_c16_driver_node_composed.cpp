// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "lslidar_c16_driver/lslidar_c16_driver.h"
#include "lslidar_c16_decoder/convert.h"
#include "rclcpp/rclcpp.hpp"

volatile sig_atomic_t flag = 1;

static void my_handler(int sig)
{
  flag = 0;
}

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  signal(SIGINT, my_handler);

  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto driver = std::make_shared<lslidar_c16_driver::lslidarDriver>(options);
  exec.add_node(driver);
  auto decoder = std::make_shared<lslidar_c16_decoder::Convert>(options);
  exec.add_node(decoder);

  while(rclcpp::ok() && driver->poll())
  {
    exec.spin_some();
  }

  return 0;
}
