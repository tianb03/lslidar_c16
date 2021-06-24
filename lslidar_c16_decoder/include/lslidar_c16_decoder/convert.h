/*
 * This file is part of lslidar_n301 driver.
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

#ifndef _CONVERT_H_
#define _CONVERT_H_

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "rawdata.h"
#include <sensor_msgs/msg/time_reference.hpp>

namespace lslidar_c16_decoder {
    class Convert final : public rclcpp::Node{
    public:
        explicit Convert(const rclcpp::NodeOptions & options);
        ~Convert() override {}
        Convert(Convert && c) = delete;
        Convert & operator=(Convert && c) = delete;
        Convert(const Convert & c) = delete;
        Convert & operator=(const Convert & c) = delete;

    private:

        void processScan(const lslidar_c16_msgs::msg::LslidarC16ScanUnified::ConstPtr scanMsg);

        void timeSync(const sensor_msgs::msg::TimeReference::ConstPtr time_msg);
        /// Pointer to dynamic reconfigure service srv_
        void publishScan(lslidar_c16_msgs::msg::LslidarC16Sweep::SharedPtr sweepData, int scanNum);

        std::shared_ptr<lslidar_rawdata::RawData> data_;
        rclcpp::Subscription<lslidar_c16_msgs::msg::LslidarC16ScanUnified>::SharedPtr packet_sub_;
        rclcpp::Subscription<sensor_msgs::msg::TimeReference>::SharedPtr sync_sub_;
        rclcpp::Time global_time;
        double last_time;
        lslidar_c16_msgs::msg::LslidarC16ScanUnified::SharedPtr scan_recv;
        lslidar_c16_msgs::msg::LslidarC16Sweep::SharedPtr sweep_data;
        bool scan_start;
        bool publish_scan;
        int scan_num;
        std::string scan_frame_id;
        size_t scan_nums;
        bool time_synchronization_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
        std::vector<int> indices;
    };

}  // namespace lslidar_c16_decoder

#endif
