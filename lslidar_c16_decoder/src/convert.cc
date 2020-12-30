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
    Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh) : data_(new lslidar_rawdata::RawData()) {
        scan_nums = 0;
        scan_start = true;

        data_->loadConfigFile(node, private_nh);  // load lidar parameters
        private_nh.param("model", model, std::string("LSC16"));

        // advertise output point cloud (before subscribing to input data)
        std::string output_points_topic;
        private_nh.param("output_points_topic", output_points_topic, std::string("lslidar_point_cloud"));
        output_ = node.advertise<sensor_msgs::PointCloud2>(output_points_topic, 10);

        scan_pub = node.advertise<sensor_msgs::LaserScan>("scan", 100);

        // subscribe to lslidar packets
        std::string input_packets_topic;
        private_nh.param("input_packets_topic", input_packets_topic, std::string("lslidar_packet"));
        packet_sub_ = node.subscribe(input_packets_topic, 10, &Convert::processScan, (Convert *) this,
                                     ros::TransportHints().tcpNoDelay(true));

        private_nh.param("time_synchronization", time_synchronization_, false);
        private_nh.param("scan_num", scan_num, 8);
        private_nh.param("publish_scan", publish_scan, false);
        private_nh.param("scan_frame_id", scan_frame_id, std::string("laser_link"));
        if (scan_num < 0) {
            scan_num = 0;
            ROS_WARN("channel_num_ outside of the index, select channel 0 instead!");
        } else if (scan_num > 15) {
            scan_num = 15;
            ROS_WARN("channel_num_ outside of the index, select channel 15 instead!");
        }
        ROS_INFO("select scan_num: %d", scan_num);

        if (time_synchronization_) {
            sync_sub_ = node.subscribe("sync_header", 10, &Convert::timeSync, (Convert *) this,
                                       ros::TransportHints().tcpNoDelay(true));
        }
    }

    void Convert::timeSync(const sensor_msgs::TimeReferenceConstPtr &time_msg) {
        global_time = time_msg->header.stamp;
    }


    void Convert::publishScan(lslidar_c16_msgs::LslidarC16SweepPtr &sweepData, int scanNum) {
        sensor_msgs::LaserScan::Ptr scan(new sensor_msgs::LaserScan);
        if (sweepData->scans[scanNum].points.size() <= 1)
            return;

        uint16_t point_num = 2000; //320000/16线/10圈
        double angle_base = M_PI * 2 / point_num;

        scan->header.frame_id = scan_frame_id;
        if (time_synchronization_)
            scan->header.stamp = sweepData->header.stamp;  // timestamp will obtained from sweep data stamp
        else
            scan->header.stamp = ros::Time::now();

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
        scan_pub.publish(scan);
    }


/** @brief Callback for raw scan messages. */
    void Convert::processScan(const lslidar_c16_msgs::LslidarC16ScanUnified::ConstPtr &scanMsg) {
        lslidar_rawdata::VPointCloud::Ptr outPoints(new lslidar_rawdata::VPointCloud);
        sweep_data = lslidar_c16_msgs::LslidarC16SweepPtr(new lslidar_c16_msgs::LslidarC16Sweep());

        if (time_synchronization_)
            outPoints->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
        else {
            outPoints->header.stamp = ros::Time::now().toNSec() / 1000ull;
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

        sensor_msgs::PointCloud2 outMsg;
        pcl::toROSMsg(*outPoints, outMsg);
        output_.publish(outMsg);

        if (publish_scan){publishScan(sweep_data, scan_num);}

    }
}  // namespace lslidar_c16_decoder
