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

#include "lslidar_c16_decoder/convert.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>


namespace lslidar_c16_decoder {
    std::string model;

/** @brief Constructor. */
    Convert::Convert(const rclcpp::NodeOptions & options)
      : rclcpp::Node("lslidar_decoder_node", options) {
        scan_nums = 0;
        scan_start = true;

        data_ = std::make_shared<lslidar_rawdata::RawData>(shared_from_this());
        data_->loadConfigFile();  // load lidar parameters
        model = this->declare_parameter("model", std::string("LSC16"));

        // advertise output point cloud (before subscribing to input data)
        std::string output_points_topic;
        output_points_topic = this->declare_parameter("output_points_topic", std::string("lslidar_point_cloud"));
        output_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_points_topic, 10);
        scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 100);

        // subscribe to lslidar packets
        std::string input_packets_topic;
        input_packets_topic = this->declare_parameter("input_packets_topic", std::string("lslidar_packet"));
        packet_sub_ = this->create_subscription<lslidar_c16_msgs::msg::LslidarC16ScanUnified>(
            input_packets_topic, rclcpp::QoS(10), std::bind(&Convert::processScan, this, std::placeholders::_1));

        time_synchronization_ = this->declare_parameter("time_synchronization", false);
        scan_num = this->declare_parameter("scan_num", 8);
        publish_scan = this->declare_parameter("publish_scan", false);
        scan_frame_id = this->declare_parameter("scan_frame_id", std::string("laser_link"));
        if (scan_num < 0) {
            scan_num = 0;
            RCLCPP_WARN(this->get_logger(), "channel_num_ outside of the index, select channel 0 instead!");
        } else if (scan_num > 15) {
            scan_num = 15;
            RCLCPP_WARN(this->get_logger(), "channel_num_ outside of the index, select channel 15 instead!");
        }
        RCLCPP_INFO(this->get_logger(), "select scan_num: %d", scan_num);

        if (time_synchronization_) {
            sync_sub_ = this->create_subscription<sensor_msgs::msg::TimeReference>(
                "sync_header", rclcpp::QoS(10), std::bind(&Convert::timeSync, this, std::placeholders::_1));
        }
    }

    void Convert::timeSync(const sensor_msgs::msg::TimeReference::ConstPtr time_msg) {
        global_time = time_msg->header.stamp;
    }


    void Convert::publishScan(lslidar_c16_msgs::msg::LslidarC16Sweep::SharedPtr sweepData, int scanNum) {
        sensor_msgs::msg::LaserScan::SharedPtr scan(new sensor_msgs::msg::LaserScan());
        if (sweepData->scans[scanNum].points.size() <= 1)
            return;

        uint16_t point_num = 2000; //320000/16线/10圈
        double angle_base = M_PI * 2 / point_num;

        scan->header.frame_id = scan_frame_id;
        if (time_synchronization_)
            scan->header.stamp = sweepData->header.stamp;  // timestamp will obtained from sweep data stamp
        else
            scan->header.stamp = this->now();

        scan->angle_min = 0;
        scan->angle_max = 2 * M_PI;
        scan->angle_increment = (scan->angle_max - scan->angle_min) / point_num;

        scan->range_min = 0.15;
        scan->range_max = 150;
        scan->ranges.reserve(point_num);
        scan->ranges.assign(point_num, std::numeric_limits<float>::infinity());
        scan->intensities.reserve(point_num);
        scan->intensities.assign(point_num, std::numeric_limits<float>::infinity());

        for (uint16_t i = 0; i < sweepData->scans[scanNum].points.size() - 1; i++) {
            int point_idx = sweepData->scans[scanNum].points[i].azimuth / angle_base;

            if (point_idx >= point_num)
                point_idx = 0;
            if (point_idx < 0)
                point_idx = point_num - 1;

            scan->ranges[point_num - 1 - point_idx] = sweepData->scans[scanNum].points[i].distance;

            scan->intensities[point_num - 1 - point_idx] = sweepData->scans[scanNum].points[i].intensity;
        }
        scan_pub->publish(std::move(*scan));
    }


/** @brief Callback for raw scan messages. */
    void Convert::processScan(const lslidar_c16_msgs::msg::LslidarC16ScanUnified::ConstPtr scanMsg) {
        lslidar_rawdata::VPointCloud::Ptr outPoints(new lslidar_rawdata::VPointCloud);
        sweep_data = lslidar_c16_msgs::msg::LslidarC16Sweep::SharedPtr(new lslidar_c16_msgs::msg::LslidarC16Sweep());

        if (time_synchronization_)
            outPoints->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
        else {
            outPoints->header.stamp = this->now().nanoseconds() / 1000ull;
        }

        outPoints->header.frame_id = scanMsg->header.frame_id;

        outPoints->clear();
        outPoints->height = 16;
        outPoints->width = 24 * (int) scanMsg->packets.size();

        outPoints->is_dense = false;
        outPoints->resize(outPoints->height * outPoints->width);

        // process each packet provided by the driver
        data_->block_num = 0;

        for (int i = 0; i < scanMsg->packets.size(); ++i) {
            data_->unpack(scanMsg->packets[i], outPoints, i, sweep_data);
        }

        sensor_msgs::msg::PointCloud2 outMsg;
        pcl::toROSMsg(*outPoints, outMsg);
        output_->publish(std::move(outMsg));

        if (publish_scan){publishScan(sweep_data, scan_num);}

    }
}  // namespace lslidar_c16_decoder

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(lslidar_c16_decoder::Convert)
