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

#ifndef _RAWDATA_H
#define _RAWDATA_H

#include <rclcpp/rclcpp.hpp>
#include <lslidar_c16_msgs/msg/lslidar_c16_packet.hpp>
#include <lslidar_c16_msgs/msg/lslidar_c16_scan_unified.hpp>
#include <lslidar_c16_msgs/msg/lslidar_c16_sweep.hpp>
#include "std_msgs/msg/string.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <stdio.h>

#define DEG_TO_RAD 0.017453292
#define RAD_TO_DEG 57.29577951

namespace lslidar_rawdata {

// static const float  ROTATION_SOLUTION_ = 0.18f;  //水平角分辨率 10hz
    static const int SIZE_BLOCK = 100;
    static const int RAW_SCAN_SIZE = 3;
    static const int SCANS_PER_BLOCK = 32;
    static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);  // 96

    static const float ROTATION_RESOLUTION = 0.01f;   /**< degrees 旋转角分辨率*/
    static const uint16_t ROTATION_MAX_UNITS = 36000; /**< hundredths of degrees */

    static const unsigned int POINTS_ONE_CHANNEL_PER_SECOND = 20000;
    static const unsigned int BLOCKS_ONE_CHANNEL_PER_PKT = 12;

    static const float DISTANCE_MAX = 200.0f;            /**< meters */
    static const float DISTANCE_MIN = 0.2f;              /**< meters */
    static const float DISTANCE_RESOLUTION = 0.01f;      /**< meters */
    static const float DISTANCE_RESOLUTION_NEW = 0.005f; /**< meters */
    static const float DISTANCE_MAX_UNITS = (DISTANCE_MAX / DISTANCE_RESOLUTION + 1.0f);
/** @todo make this work for both big and little-endian machines */
    static const uint16_t UPPER_BANK = 0xeeff;  //
    static const uint16_t LOWER_BANK = 0xddff;

/** Special Defines for LSC16 support **/
    static const int LSC16_FIRINGS_PER_BLOCK = 2;
    static const int LSC16_SCANS_PER_FIRING = 16;
    static const float LSC16_BLOCK_TDURATION = 100.0f;  // [µs]
    static const float LSC16_DSR_TOFFSET = 3.125f;        // [µs]
    static const float LSC16_FIRING_TOFFSET = 50.0f;    // [µs]


    static const int TEMPERATURE_MIN = 31;

/** \brief Raw LSLIDAR C16 data block.
 *
 *  Each block contains data from either the upper or lower laser
 *  bank.  The device returns three times as many upper bank blocks.
 *
 *  use stdint.h types, so things work with both 64 and 32-bit machines
 */
// block
    typedef struct raw_block {
        uint16_t header;  ///< UPPER_BANK or LOWER_BANK
        uint8_t rotation_1;
        uint8_t rotation_2;  /// combine rotation1 and rotation2 together to get 0-35999, divide by 100 to get degrees
        uint8_t data[BLOCK_DATA_SIZE];  // 96
    } raw_block_t;

    struct PointXYZITM {
        PCL_ADD_POINT4D
        uint8_t intensity;
        uint8_t lines;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
    } EIGEN_ALIGN16;
// enforce SSE padding for correct memory alignment
/** used for unpacking the first two data bytes in a block
 *
 *  They are packed into the actual data stream misaligned.  I doubt
 *  this works on big endian machines.
 */
    union two_bytes {
        uint16_t uint;
        uint8_t bytes[2];
    };
// Pre-compute the sine and cosine for the altitude angles.
    static const double scan_altitude_original_2[16] = {
            -0.2617993877991494, 0.017453292519943295,
            -0.22689280275926285, 0.05235987755982989,
            -0.19198621771937624, 0.08726646259971647,
            -0.15707963267948966, 0.12217304763960307,
            -0.12217304763960307, 0.15707963267948966,
            -0.08726646259971647, 0.19198621771937624,
            -0.05235987755982989, 0.22689280275926285,
            -0.017453292519943295, 0.2617993877991494
    };
    static const double scan_altitude_original_1[16] = {
            -0.17453292519943295,0.01160643952576229,
            -0.15123277968530863,0.03490658503988659,
            -0.12793263417118436,0.05811946409141117,
            -0.10471975511965978,0.08141960960553547,
            -0.08141960960553547,0.10471975511965978,
            -0.05811946409141117,0.12793263417118436,
            -0.03490658503988659,0.15123277968530863,
            -0.01160643952576229,0.17453292519943295
    };

    static double scan_altitude[16];

    static const double cos_scan_altitude[16] = {
            std::cos(scan_altitude[0]), std::cos(scan_altitude[1]),
            std::cos(scan_altitude[2]), std::cos(scan_altitude[3]),
            std::cos(scan_altitude[4]), std::cos(scan_altitude[5]),
            std::cos(scan_altitude[6]), std::cos(scan_altitude[7]),
            std::cos(scan_altitude[8]), std::cos(scan_altitude[9]),
            std::cos(scan_altitude[10]), std::cos(scan_altitude[11]),
            std::cos(scan_altitude[12]), std::cos(scan_altitude[13]),
            std::cos(scan_altitude[14]), std::cos(scan_altitude[15]),
    };

    static const double sin_scan_altitude[16] = {
            std::sin(scan_altitude[0]), std::sin(scan_altitude[1]),
            std::sin(scan_altitude[2]), std::sin(scan_altitude[3]),
            std::sin(scan_altitude[4]), std::sin(scan_altitude[5]),
            std::sin(scan_altitude[6]), std::sin(scan_altitude[7]),
            std::sin(scan_altitude[8]), std::sin(scan_altitude[9]),
            std::sin(scan_altitude[10]), std::sin(scan_altitude[11]),
            std::sin(scan_altitude[12]), std::sin(scan_altitude[13]),
            std::sin(scan_altitude[14]), std::sin(scan_altitude[15]),
    };
    static const int PACKET_SIZE = 1206;
    static const int BLOCKS_PER_PACKET = 12;
    static const int PACKET_STATUS_SIZE = 4;
    static const int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);

/** \brief Raw lslidar packet.
 *
 *  revolution is described in the device manual as incrementing
 *    (mod 65536) for each physical turn of the device.  Our device
 *    seems to alternate between two different values every third
 *    packet.  One value increases, the other decreases.
 *
 *  \todo figure out if revolution is only present for one of the
 *  two types of status fields
 *
 *  status has either a temperature encoding or the microcode level
 */
    typedef struct raw_packet {
        raw_block_t blocks[BLOCKS_PER_PACKET];
        uint16_t revolution;
        uint8_t status[PACKET_STATUS_SIZE];
    } raw_packet_t;

    union vertical_point {
        uint8_t uint[2];
        uint16_t value;
    };

    typedef PointXYZITM VPoint;
    typedef pcl::PointCloud<VPoint> VPointCloud;

/** \brief lslidar data conversion class */
    class RawData {
    public:
        RawData(rclcpp::Node::SharedPtr private_nh);

        ~RawData() {
        }

        /*load the cablibrated files: angle, distance, intensity*/
        void loadConfigFile();

        /*unpack the UDP packet and opuput PCL PointXYZI type*/
        void unpack(const lslidar_c16_msgs::msg::LslidarC16Packet &pkt, VPointCloud::Ptr pointcloud,int Packet_num,lslidar_c16_msgs::msg::LslidarC16Sweep::SharedPtr& sweep_data);

        void processDifop(const lslidar_c16_msgs::msg::LslidarC16Packet::ConstPtr difop_msg);

        rclcpp::Subscription<lslidar_c16_msgs::msg::LslidarC16Packet>::SharedPtr difop_sub_;
        bool is_init_curve_;
        bool is_init_angle_;
        bool is_init_top_fw_;
        int block_num = 0;
        int intensity_mode_;
        int intensityFactor;

    private:
        float R1_;
        float R2_;
        bool angle_flag_;
        float start_angle_;
        float end_angle_;
        float max_distance_;
        float min_distance_;
        float distance_unit_;
        int dis_resolution_mode_;
        int return_mode_;
        int degree_mode_;
        int rpm_;
        bool info_print_flag_;
        std::string calibration_file_;
        float scan_start_angle_;
        float scan_end_angle_;

        int packet_end;



        bool cbMethod_;
        bool config_vert_;
        bool config_vert_angle;
        bool print_vert_;
        bool config_vert_file_;
        double vert_angle;
        double cos_scan_altitude_caliration[LSC16_SCANS_PER_FIRING];
        double sin_scan_altitude_caliration[LSC16_SCANS_PER_FIRING];

        rclcpp::Node::SharedPtr private_nh_;
    };

    static float sin_azimuth_table[ROTATION_MAX_UNITS];
    static float cos_azimuth_table[ROTATION_MAX_UNITS];


    static float VERT_ANGLE[32];
    static float HORI_ANGLE[32];
    static float aIntensityCal[7][32];
    static float aIntensityCal_old[1600][32];
    static const bool Curvesis_new = true;
    static int g_ChannelNum[32][51];
    static float CurvesRate[32];

    static const float temper = 31.0;
    static const int tempPacketNum = 0;
    static int numOfLasers = 16;
    static const int TEMPERATURE_RANGE = 40;



}  // namespace lslidar_rawdata

POINT_CLOUD_REGISTER_POINT_STRUCT(lslidar_rawdata::PointXYZITM,
                                  (float, x, x)(float, y, y)(float, z, z)
                                          (uint8_t, intensity, intensity)
                                          (uint8_t, lines, lines)
                                          )

#endif  // __RAWDATA_H
