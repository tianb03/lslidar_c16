/*
 * This file is part of lslidar_c16 driver.
 *
 * The driver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the driver.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <rclcpp/rclcpp.hpp>
#include "lslidar_c16_driver/lslidar_c16_driver.h"

using namespace lslidar_c16_driver;
volatile sig_atomic_t flag = 1;

static void my_handler(int sig)
{
  flag = 0;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  signal(SIGINT, my_handler);

  // start the driver
  auto dvr = std::make_shared<lslidar_c16_driver::lslidarDriver>(rclcpp::NodeOptions());

  // loop until shut down or end of file
  while(rclcpp::ok() && dvr->poll())
  {
    rclcpp::spin_some(dvr);
  }

  return 0;
}
