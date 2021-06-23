/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009-2012 Austin Robot Technology, Jack O'Quin
 *	Copyright (C) 2017 Robosense, Tony Zhang
 *	Copyright (C) 2021 BDR, Zhensheng Li
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver implementation for the RILIDAR 3D LIDARs
 */
#include "lslidar_c16_driver/lslidar_c16_driver.h"
#include <lslidar_c16_msgs/msg/lslidar_c16_scan_unified.hpp>


namespace lslidar_c16_driver {
    static const unsigned int POINTS_ONE_CHANNEL_PER_SECOND = 20000;
    static const unsigned int BLOCKS_ONE_CHANNEL_PER_PKT = 12;

    lslidarDriver::lslidarDriver(const rclcpp::NodeOptions & options)
    : rclcpp::Node("lslidar_driver_node", options) {
        scan_fill = false;
        // use private node handle to get parameters
        config_.frame_id = this->declare_parameter("frame_id", std::string("lslidar"));

        // get model name, validate string, determine packet rate
        config_.model = this->declare_parameter("model", std::string("LSC16"));
        double packet_rate;  // packet frequency (Hz)

        packet_rate = 840;   //20000/24

        config_.rpm = this->declare_parameter("rpm", 300.0);
        config_.return_mode = this->declare_parameter("return_mode", 1);
        double frequency = (config_.rpm / 60.0);  // expected Hz rate
        printf("driver return mode = %d\n", config_.return_mode);
        // default number of packets for each scan is a single revolution
        // (fractions rounded up)
        int npackets = (int) ceil(packet_rate / frequency);
        // config_.npackets = this->declare_parameter("npackets");
        RCLCPP_INFO(this->get_logger(), "publishing %d packets per scan", config_.npackets);

        std::string dump_file;
        dump_file = this->declare_parameter("pcap", std::string(""));

        int msop_udp_port;
        msop_udp_port = this->declare_parameter("msop_port", static_cast<int>(MSOP_DATA_PORT_NUMBER));
        int difop_udp_port;
        difop_udp_port = this->declare_parameter("difop_port", static_cast<int>(DIFOP_DATA_PORT_NUMBER));

        scan_start = std::make_unique<lslidar_c16_msgs::msg::LslidarC16ScanUnified>();
        scan_start->packets.resize(10);

        // open rslidar input device or file
        if (dump_file != "")  // have PCAP file?
        {
            // read data from packet capture file
            msop_input_.reset(new lslidar_c16_driver::InputPCAP(this, msop_udp_port, packet_rate, dump_file));
            difop_input_.reset(new lslidar_c16_driver::InputPCAP(this, difop_udp_port, packet_rate, dump_file));
        } else {
            // read data from live socket
            msop_input_.reset(new lslidar_c16_driver::InputSocket(this, msop_udp_port));
            difop_input_.reset(new lslidar_c16_driver::InputSocket(this, difop_udp_port));

        }

        // raw packet output topic
        std::string output_packets_topic;
        output_packets_topic = this->declare_parameter("output_packets_topic", std::string("lslidar_packet"));
        msop_output_ = this->create_publisher<lslidar_c16_msgs::msg::LslidarC16ScanUnified>("output_packets_topic", 10);

        std::string output_difop_topic;
        output_difop_topic = this->declare_parameter("output_difop_topic", std::string("lslidar_packet_difop"));
        difop_output_ = this->create_publisher<lslidar_c16_msgs::msg::LslidarC16Packet>("output_difop_topic", 10);

        difop_thread_ = std::make_unique<std::thread>(&lslidarDriver::pollThread, this);
        time_synchronization_ = this->declare_parameter("time_synchronization", false);


        if (time_synchronization_) {
            output_sync_ = this->create_publisher<sensor_msgs::msg::TimeReference>("sync_header", 1);
        }
    }


    lslidarDriver::~lslidarDriver() {
        exit_signal_.set_value();
        if (difop_thread_ != NULL) {
            printf("error");
            difop_thread_->join();
        }

    }

/** poll the device
 *
 *  @returns true unless end of file reached
 */
    bool lslidarDriver::poll(void) {  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
        auto scan = std::make_unique<lslidar_c16_msgs::msg::LslidarC16ScanUnified>();
        // Since the rslidar delivers data at a very high rate, keep
        // reading and publishing scans as fast as possible.
        int mode = config_.return_mode;
        uint64_t GPSCurrentTS;
        if (difop_input_->getUpdateFlag()) {
            int packets_rate = ceil(POINTS_ONE_CHANNEL_PER_SECOND / BLOCKS_ONE_CHANNEL_PER_PKT);
            packets_rate = ceil(packets_rate / 2);
            config_.rpm = difop_input_->getRpm();

//    if(config_.rpm >= 300 && config_.rpm < 600){
//      config_.rpm = 300;
//    }else if(config_.rpm >= 600 && config_.rpm < 1200){
//      config_.rpm = 600;
//    }else if(config_.rpm >= 1200){
//      config_.rpm = 1200;
//    }
            //config_.npackets = ceil(packets_rate * 60 / config_.rpm) * mode;
            config_.npackets = ceil(packets_rate * 60 / config_.rpm) * mode;

            config_.npackets = config_.npackets * 11 / 10;

            difop_input_->clearUpdateFlag();
            //ROS_INFO("packet rate is %d, rpm is %3.3f, npacket is %d", packets_rate, config_.rpm, config_.npackets);
        }

        //ROS_INFO("rpm is %3.3f, npacket is %d", config_.rpm, config_.npackets);
        scan->packets.clear();
        scan->packets.resize(config_.npackets);
        int azi1, azi2;
        if (scan_fill) {
            scan->packets[0] = scan_start->packets[0];
            GPSCurrentTS = GPSCountingTS;
        } else {
            while (true) {
                while (true) {
                    int rc = msop_input_->getPacket(&scan->packets[0], config_.time_offset);
                    if (rc == 0)
                        break;
                    if (rc < 0)
                        return false;
                }

                azi1 = 256 * scan->packets[0].data[3] + scan->packets[0].data[2];
                azi2 = 256 * scan->packets[0].data[1103] + scan->packets[0].data[1102];
                if (azi1 > 35000 && azi2 < 1000) break;
            }
        }
        scan_fill = false;
        // use in standard behaviour only

        for (int i = 1; i < config_.npackets; ++i) {

            while (true) {
                // keep reading until full packet received
                //ROS_INFO_STREAM("time_offset: " << config_.time_offset);
                int rc = msop_input_->getPacket(&scan->packets[i], config_.time_offset);
                if (rc == 0)
                    break;  // got a full packet?
                if (rc < 0)
                    return false;  // end of file reached?

            }
            azi1 = 256 * scan->packets[i].data[3] + scan->packets[i].data[2];
            azi2 = 256 * scan->packets[i].data[1103] + scan->packets[i].data[1102];
            //azi2 = (azi2 +20) % 36000;
            //if ( (azi1 > 35800 && azi2 < 100 )) {

            if ((azi1 > 35000 && azi2 < 1000) || (azi1 < 500 && i > config_.npackets / 2)) {

                scan_fill = true;
                scan_start->packets[0] = scan->packets[i];
                // ROS_INFO_STREAM("azi1: " << azi1 <<"  "<< "azi2: " << azi2 << "   i:" << i);
                break;
            }
        }

        if (time_synchronization_) {
            sensor_msgs::msg::TimeReference sync_header;

            // it is already the msop msg
            // use the first packets
            lslidar_c16_msgs::msg::LslidarC16Packet pkt = scan->packets[0];
            uint64_t packet_timestamp;

            static uint64_t last_gps_time;    //上一个设备包的gps时间
            static uint64_t last_packet_seconds;  //上一个数据包的时间戳
            packet_timestamp = (pkt.data[1200] +
                                pkt.data[1201] * pow(2, 8) +
                                pkt.data[1202] * pow(2, 16) +
                                pkt.data[1203] * pow(2, 24)) * 1e3; //ns

            //timeStamp = rclcpp::Time(GPSCurrentTS, packet_timestamp);// s,ns
            if (last_packet_seconds > 800000000 && packet_timestamp < 200000000) {
                timeStamp = rclcpp::Time(GPSCurrentTS, packet_timestamp);
            } else {
                timeStamp = rclcpp::Time(last_gps_time, packet_timestamp);
            }

            last_gps_time = GPSCurrentTS;
            last_packet_seconds = packet_timestamp;

            //ROS_INFO("Lidar_time: %f, GPS_time:%lu, fpga_time: ns:%lu",timeStamp.toSec(), GPSCurrentTS, packet_timestamp);
            sync_header.header.stamp = timeStamp;

            output_sync_->publish(std::move(sync_header));
        }

        // publish message using time of last packet read
        //  ROS_INFO("Publishing a full scan.");
        if (time_synchronization_) {
            scan->header.stamp = timeStamp;

        } else {
            scan->header.stamp = scan->packets.back().stamp;
		//scan->header.stamp = rclcpp::Time::now();
        }
        scan->header.frame_id = config_.frame_id;
        msop_output_->publish(std::move(scan));

        return true;
    }

    void lslidarDriver::difopPoll(void) {
        // reading and publishing scans as fast as possible.
        auto difop_packet_ptr = std::make_unique<lslidar_c16_msgs::msg::LslidarC16Packet>();
        while (rclcpp::ok()) {
            // keep reading
            lslidar_c16_msgs::msg::LslidarC16Packet difop_packet_msg;
            int rc = difop_input_->getPacket(&difop_packet_msg, config_.time_offset);
            if (rc == 0) {
                //std::cout << "Publishing a difop data." << std::endl;
                RCLCPP_DEBUG(this->get_logger(), "Publishing a difop data.");
                *difop_packet_ptr = difop_packet_msg;
                difop_output_->publish(std::move(difop_packet_ptr));
                int version_data = difop_packet_msg.data[1202];
                if (2 == version_data) {
                    this->packetTimeStamp[4] = difop_packet_msg.data[41];
                    this->packetTimeStamp[5] = difop_packet_msg.data[40];
                    this->packetTimeStamp[6] = difop_packet_msg.data[39];
                    this->packetTimeStamp[7] = difop_packet_msg.data[38];
                    this->packetTimeStamp[8] = difop_packet_msg.data[37];
                    this->packetTimeStamp[9] = difop_packet_msg.data[36];
                } else {
                    this->packetTimeStamp[4] = difop_packet_msg.data[57];
                    this->packetTimeStamp[5] = difop_packet_msg.data[56];
                    this->packetTimeStamp[6] = difop_packet_msg.data[55];
                    this->packetTimeStamp[7] = difop_packet_msg.data[54];
                    this->packetTimeStamp[8] = difop_packet_msg.data[53];
                    this->packetTimeStamp[9] = difop_packet_msg.data[52];
                }
                //ROS_INFO_STREAM("time: " << difop_packet_msg.data[57]);
                //ROS_INFO_STREAM("time: " << difop_packet_msg.data[56]);

                struct tm cur_time;
                memset(&cur_time, 0, sizeof(cur_time));
                cur_time.tm_sec = this->packetTimeStamp[4] + 1;
                cur_time.tm_min = this->packetTimeStamp[5];
                cur_time.tm_hour = this->packetTimeStamp[6] + 8;
                cur_time.tm_mday = this->packetTimeStamp[7];
                cur_time.tm_mon = this->packetTimeStamp[8] - 1;
                cur_time.tm_year = this->packetTimeStamp[9] + 2000 - 1900;
                this->pointcloudTimeStamp = mktime(&cur_time);

                if (GPSCountingTS != this->pointcloudTimeStamp) {
                    cnt_gps_ts = 0;
                    GPSCountingTS = this->pointcloudTimeStamp;
                    // ROS_ERROR("GPSCountingTS=%lu",GPSCountingTS);
                    //to beijing time printing
                    //ROS_INFO("GPS: y:%d m:%d d:%d h:%d m:%d s:%d",cur_time.tm_year+1900,cur_time.tm_mon+1,cur_time.tm_mday,cur_time.tm_hour+8,cur_time.tm_min,cur_time.tm_sec);
                } else if (cnt_gps_ts == 3) {
                    GPSStableTS = GPSCountingTS;
                } else {
                    cnt_gps_ts++;
                }
            }
            if (rc < 0)
                return;  // end of file reached?
            rclcpp::spin_some(this->get_node_base_interface());
        }
    }

void lslidarDriver::pollThread()
{
  std::future_status status;

  do {
    difopPoll();
    status = future_.wait_for(std::chrono::seconds(0));
  } while (status == std::future_status::timeout);
}

// add for time synchronization
}  // namespace lslidar_c16_driver

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(lslidar_c16_driver::lslidarDriver)
