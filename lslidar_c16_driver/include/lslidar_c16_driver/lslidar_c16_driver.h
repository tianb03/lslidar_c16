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

#ifndef _LS_C16_DRIVER_H_
#define _LS_C16_DRIVER_H_

#include <string>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include "input.h"
#include <lslidar_c16_msgs/msg/lslidar_c16_scan_unified.hpp>

namespace lslidar_c16_driver
{
class lslidarDriver final : public rclcpp::Node
{
public:
  /**
 * @brief lslidarDriver
 * @param node          raw packet output topic
 * @param private_nh    通过这个节点传参数
 */
  explicit lslidarDriver(const rclcpp::NodeOptions & options);
  ~lslidarDriver() override;
  lslidarDriver(lslidarDriver && c) = delete;
  lslidarDriver & operator=(lslidarDriver && c) = delete;
  lslidarDriver(const lslidarDriver & c) = delete;
  lslidarDriver & operator=(const lslidarDriver & c) = delete;

  bool poll(void);

private:
  void difopPoll(void);
  void pollThread();
  /// Callback for skip num for time synchronization
  // void skipNumCallback(const std_msgs::Int32::ConstPtr& skip_num);

  // configuration parameters
  struct
  {
    std::string frame_id;  ///< tf frame ID
    std::string model;     ///< device model name
    int npackets;          ///< number of packets to collect
    double rpm;            ///< device rotation rate (RPMs)
    double time_offset;    ///< time in seconds added to each  time stamp
    int cut_angle;
    int return_mode;     //return wave number
  } config_;

  std::unique_ptr<Input> msop_input_;
  std::unique_ptr<Input> difop_input_;
  rclcpp::Publisher<lslidar_c16_msgs::msg::LslidarC16ScanUnified>::SharedPtr msop_output_;
  rclcpp::Publisher<lslidar_c16_msgs::msg::LslidarC16Packet>::SharedPtr difop_output_;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr output_sync_;
  // Converter convtor_
  std::unique_ptr<std::thread> difop_thread_;

  // add for time synchronization
  bool time_synchronization_;
  unsigned char packetTimeStamp[10];
  uint64_t pointcloudTimeStamp;
  uint64_t GPSStableTS;
  uint64_t GPSCountingTS;
  uint64_t last_FPGA_ts;
  uint64_t GPS_ts;
  int cnt_gps_ts;
  rclcpp::Time timeStamp;
  uint64_t usec_start;

  bool scan_fill;
  std::unique_ptr<lslidar_c16_msgs::msg::LslidarC16ScanUnified> scan_start;

  // We use this future/promise pair to notify threads that we are shutting down
  std::shared_future<void> future_;
  std::promise<void> exit_signal_;

};

}  // namespace lslidar_c16_driver

#endif  // _LS_C16_DRIVER_H_
