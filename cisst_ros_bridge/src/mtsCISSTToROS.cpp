/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-21

  (C) Copyright 2013-2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include "cisstParameterTypes/prmPositionCartesianGet.h"
#include "geometry_msgs/msg/transform.hpp"
#include <cisst_ros_bridge/mtsCISSTToROS.h>

#include <limits>

#if ROS1
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#elif ROS2
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#endif

void mtsCISSTToROS(const double & cisstData,
                   CISST_RAL_MSG(std_msgs, Float32) & rosData,
                   const std::string &)
{
    rosData.data = cisstData;
}

void mtsCISSTToROS(const double & cisstData,
                   CISST_RAL_MSG(std_msgs, Float64) & rosData,
                   const std::string &)
{
    rosData.data = cisstData;
}

void mtsCISSTToROS(const int & cisstData,
                   CISST_RAL_MSG(std_msgs, Int32) & rosData,
                   const std::string &)
{
    rosData.data = cisstData;
}

void mtsCISSTToROS(const size_t & cisstData,
                   CISST_RAL_MSG(std_msgs, UInt64) & rosData,
                   const std::string &)
{
    rosData.data = cisstData;
}

void mtsCISSTToROS(const bool & cisstData,
                   CISST_RAL_MSG(std_msgs, Bool) & rosData,
                   const std::string &)
{
    rosData.data = cisstData;
}

void mtsCISSTToROS(const bool & cisstData,
                   CISST_RAL_MSG(cisst_msgs, BoolStamped) & rosData,
                   const std::string &)
{
    rosData.data = cisstData;
}

void mtsCISSTToROS(const bool & cisstData,
                   CISST_RAL_MSG(sensor_msgs, Joy) & rosData,
                   const std::string &)
{
    rosData.axes.resize(0);
    rosData.buttons.resize(1);
    rosData.buttons[0] = cisstData;
}

void mtsCISSTToROS(const std::string & cisstData,
                   CISST_RAL_MSG(std_msgs, String) & rosData,
                   const std::string &)
{
    rosData.data = cisstData;
}

void mtsCISSTToROS(const mtsMessage & cisstData,
                   CISST_RAL_MSG(std_msgs, String) & rosData,
                   const std::string &)
{
    rosData.data = cisstData.Message;
}

void mtsCISSTToROS(const prmEventButton & cisstData,
                   CISST_RAL_MSG(std_msgs, Bool) & rosData,
                   const std::string &)
{
    if (cisstData.Type() == prmEventButton::PRESSED) {
        rosData.data = true;
    } else if (cisstData.Type() == prmEventButton::RELEASED) {
        rosData.data = false;
    }
}

void mtsCISSTToROS(const prmEventButton & cisstData,
                   CISST_RAL_MSG(cisst_msgs, BoolStamped) & rosData,
                   const std::string &)
{
    if (cisstData.Type() == prmEventButton::PRESSED) {
        rosData.data = true;
    } else if (cisstData.Type() == prmEventButton::RELEASED) {
        rosData.data = false;
    }
}

void mtsCISSTToROS(const prmEventButton & cisstData,
                   CISST_RAL_MSG(sensor_msgs, Joy) & rosData,
                   const std::string &)
{
    rosData.axes.resize(0);
    rosData.buttons.resize(1);
    if (cisstData.Type() == prmEventButton::PRESSED) {
        rosData.buttons[0] = 1;
    } else if (cisstData.Type() == prmEventButton::RELEASED) {
        rosData.buttons[0] = 0;
    } else if (cisstData.Type() == prmEventButton::CLICKED) {
        rosData.buttons[0] = 2;
    }
}

void mtsCISSTToROS(const Eigen::MatrixXd & cisstData,
                   CISST_RAL_MSG(std_msgs, Float64MultiArray) & rosData,
                   const std::string &)
{
    rosData.layout.dim.resize(2);
    rosData.layout.dim[0].label  = "rows";
    rosData.layout.dim[0].size   = cisstData.rows();
    rosData.layout.dim[0].stride = 1;
    rosData.layout.dim[1].label  = "cols";
    rosData.layout.dim[1].size   = cisstData.cols();
    rosData.layout.dim[1].stride = cisstData.rows();
    rosData.layout.data_offset = 0;
    const size_t size = cisstData.size();
    if (size != 0) {
        rosData.data.resize(size);
        auto row_major_linear_view = cisstData.reshaped<Eigen::RowMajor>();
        std::copy(row_major_linear_view.begin(), row_major_linear_view.end(),
                  rosData.data.begin());
    }
}

void mtsCISSTToROS(const Eigen::VectorXd & cisstData,
                   CISST_RAL_MSG(std_msgs, Float64MultiArray) & rosData,
                   const std::string &)
{
    rosData.layout.dim.resize(2);
    rosData.layout.dim[0].label  = "rows";
    rosData.layout.dim[0].size   = 1;
    rosData.layout.dim[0].stride = 1;
    rosData.layout.dim[1].label  = "cols";
    rosData.layout.dim[1].size   = cisstData.size();
    rosData.layout.dim[1].stride = 1;
    rosData.layout.data_offset = 0;
    const size_t size = cisstData.size();
    if (size != 0) {
        rosData.data.resize(size);
        std::copy(cisstData.begin(), cisstData.end(),
                  rosData.data.begin());
    }
}

void mtsCISSTToROS(const Eigen::Vector3d & cisstData,
                   CISST_RAL_MSG(geometry_msgs, Vector3) & rosData,
                   const std::string &)
{
    rosData.x = cisstData.x();
    rosData.y = cisstData.y();
    rosData.z = cisstData.z();
}

void mtsCISSTToROS(const Eigen::Quaterniond & cisstData,
                   CISST_RAL_MSG(geometry_msgs, Quaternion) & rosData,
                   const std::string &)
{
    rosData.x = cisstData.x();
    rosData.y = cisstData.y();
    rosData.z = cisstData.z();
    rosData.w = cisstData.w();
}

void mtsCISSTToROS(const prmPositionCartesianGet & cisstData,
                   CISST_RAL_MSG(geometry_msgs, Pose) & rosData,
                   const std::string & debugInfo)
{
    mtsCISSTToROS(cisstData.Position(), rosData, debugInfo);
}

void mtsCISSTToROS(const prmPositionCartesianSet & cisstData,
                   CISST_RAL_MSG(geometry_msgs, Pose) & rosData,
                   const std::string & debugInfo)
{
    mtsCISSTToROS(cisstData.Goal(), rosData, debugInfo);
}

void mtsCISSTToROS(const Eigen::Isometry3d & cisstData,
                   CISST_RAL_MSG(geometry_msgs, Pose) & rosData,
                   const std::string &)
{
    rosData.position.x = cisstData.translation().x();
    rosData.position.y = cisstData.translation().y();
    rosData.position.z = cisstData.translation().z();

    const Eigen::Quaterniond q(cisstData.rotation());
    rosData.orientation.x = q.x();
    rosData.orientation.y = q.y();
    rosData.orientation.z = q.z();
    rosData.orientation.w = q.w();
}

void mtsCISSTToROS(const prmPositionCartesianGet & cisstData,
                   CISST_RAL_MSG(geometry_msgs, Transform) & rosData,
                   const std::string & debugInfo)
{
    mtsCISSTToROS(cisstData.Position(), rosData, debugInfo);
}

void mtsCISSTToROS(const prmPositionCartesianSet & cisstData,
                   CISST_RAL_MSG(geometry_msgs, Transform) & rosData,
                   const std::string & debugInfo)
{
    mtsCISSTToROS(cisstData.Goal(), rosData, debugInfo);
}

void mtsCISSTToROS(const Eigen::Isometry3d & cisstData,
                   CISST_RAL_MSG(geometry_msgs, Transform) & rosData,
                   const std::string &)
{
    rosData.translation.x = cisstData.translation().x();
    rosData.translation.y = cisstData.translation().y();
    rosData.translation.z = cisstData.translation().z();

    const Eigen::Quaterniond q(cisstData.rotation());
    rosData.rotation.x = q.x();
    rosData.rotation.y = q.y();
    rosData.rotation.z = q.z();
    rosData.rotation.w = q.w();
}

void mtsCISSTToROS(const prmForceCartesianGet & cisstData,
                   CISST_RAL_MSG(geometry_msgs, Wrench)  & rosData,
                   const std::string & debugInfo)
{
    mtsCISSTToROS(cisstData.Force(), rosData, debugInfo);
}

void mtsCISSTToROS(const prmForceCartesianSet & cisstData,
                   CISST_RAL_MSG(geometry_msgs, Wrench)  & rosData,
                   const std::string & debugInfo)
{
    mtsCISSTToROS(cisstData.Force(), rosData, debugInfo);
}

void mtsCISSTToROS(const Eigen::Vector<double, 6> & cisstData,
                   CISST_RAL_MSG(geometry_msgs, Wrench) & rosData,
                   const std::string &)
{
    rosData.force.x = cisstData(0);
    rosData.force.y = cisstData(1);
    rosData.force.z = cisstData(2);
    rosData.torque.x = cisstData(3);
    rosData.torque.y = cisstData(4);
    rosData.torque.z = cisstData(5);
}

void mtsCISSTToROS(const prmVelocityCartesianGet & cisstData,
                   CISST_RAL_MSG(geometry_msgs, Twist)  & rosData,
                   const std::string & debugInfo)
{
    Eigen::Vector<double, 6> twist;
    cisstData.GetVelocity(twist);
    mtsCISSTToROS(twist, rosData, debugInfo);
}

void mtsCISSTToROS(const prmVelocityCartesianSet & cisstData,
                   CISST_RAL_MSG(geometry_msgs, Twist)  & rosData,
                   const std::string & debugInfo)
{
    Eigen::Vector<double, 6> twist;
    twist.head<3>() = cisstData.GetVelocity();
    twist.tail<3>() = cisstData.GetAngularVelocity();
    mtsCISSTToROS(twist, rosData, debugInfo);
}

void mtsCISSTToROS(const Eigen::Vector<double, 6> & cisstData,
                   CISST_RAL_MSG(geometry_msgs, Twist) & rosData,
                   const std::string &)
{
    rosData.linear.x = cisstData(0);
    rosData.linear.y = cisstData(1);
    rosData.linear.z = cisstData(2);

    rosData.angular.x = cisstData(3);
    rosData.angular.y = cisstData(4);
    rosData.angular.z = cisstData(5);
}

void mtsCISSTToROS(const prmPositionCartesianArrayGet & cisstData,
                   CISST_RAL_MSG(geometry_msgs, PoseArray) & rosData,
                   const std::string & debugInfo)
{
    rosData.poses.resize(cisstData.Positions().size());
    for (size_t idx = 0; idx < cisstData.Positions().size(); idx++) {
        mtsCISSTToROS(cisstData.Positions()[idx], rosData.poses[idx], debugInfo);
    }
}

// ---------------------------------------------
// sensor_msgs
// ---------------------------------------------

void mtsCISSTToROSSetJointNames(CISST_RAL_MSG(sensor_msgs, JointState) & rosData, const size_t & nb_joints)
{
    rosData.name.resize(nb_joints);
    for (size_t i = 0; i < nb_joints; ++i) {
        rosData.name[i] = std::to_string(i);
    }
}

void mtsCISSTToROS(const prmPositionJointGet & cisstData,
                   CISST_RAL_MSG(sensor_msgs, JointState) & rosData,
                   const std::string &)
{
    const size_t size = cisstData.Position().size();
    mtsCISSTToROSSetJointNames(rosData, size);
    rosData.velocity.resize(0);
    rosData.effort.resize(0);
    if (size != 0) {
        rosData.position.resize(size);
        std::copy(cisstData.Position().begin(), cisstData.Position().end(),
                  rosData.position.begin());
    }
}

void mtsCISSTToROS(const prmPositionJointSet & cisstData,
                   CISST_RAL_MSG(sensor_msgs, JointState) & rosData,
                   const std::string &)
{
    const size_t size = cisstData.Goal().size();
    mtsCISSTToROSSetJointNames(rosData, size);
    rosData.velocity.resize(0);
    rosData.effort.resize(0);
    if (size != 0) {
        rosData.position.resize(size);
        std::copy(cisstData.Goal().begin(), cisstData.Goal().end(),
                  rosData.position.begin());
    }
}

void mtsCISSTToROS(const prmVelocityJointGet & cisstData,
                   CISST_RAL_MSG(sensor_msgs, JointState) & rosData,
                   const std::string &)
{
    const size_t size = cisstData.Velocity().size();
    mtsCISSTToROSSetJointNames(rosData, size);
    rosData.position.resize(0);
    rosData.effort.resize(0);
    if (size != 0) {
        rosData.velocity.resize(size);
        std::copy(cisstData.Velocity().begin(), cisstData.Velocity().end(),
                  rosData.velocity.begin());
    }
}

void mtsCISSTToROS(const prmVelocityJointSet & cisstData,
                   CISST_RAL_MSG(sensor_msgs, JointState) & rosData,
                   const std::string &)
{
    const size_t size = cisstData.Goal().size();
    mtsCISSTToROSSetJointNames(rosData, size);
    rosData.position.resize(0);
    rosData.effort.resize(0);
    if (size != 0) {
        rosData.velocity.resize(size);
        std::copy(cisstData.Goal().begin(), cisstData.Goal().end(),
                  rosData.velocity.begin());
    }
}

void mtsCISSTToROS(const prmForceTorqueJointSet & cisstData,
                   CISST_RAL_MSG(sensor_msgs, JointState) & rosData,
                   const std::string &)
{
    const size_t size = cisstData.ForceTorque().size();
    mtsCISSTToROSSetJointNames(rosData, size);
    rosData.position.resize(0);
    rosData.velocity.resize(0);
    if (size != 0) {
        rosData.effort.resize(size);
        std::copy(cisstData.ForceTorque().begin(), cisstData.ForceTorque().end(),
                  rosData.effort.begin());
    }
}

void mtsCISSTToROS(const prmStateJoint & cisstData,
                   CISST_RAL_MSG(sensor_msgs, JointState) & rosData,
                   const std::string &)
{
    { // names
        const size_t size = cisstData.Name().size();
        if (size != 0) {
            rosData.name.resize(size);
            std::copy(cisstData.Name().begin(), cisstData.Name().end(),
                      rosData.name.begin());
        }
    }
    { // positions
        const size_t size = cisstData.Position().size();
        if (size != 0) {
            rosData.position.resize(size);
            std::copy(cisstData.Position().begin(), cisstData.Position().end(),
                      rosData.position.begin());
        }
    }
    { // velocities
        const size_t size = cisstData.Velocity().size();
        if (size != 0) {
            rosData.velocity.resize(size);
            std::copy(cisstData.Velocity().begin(), cisstData.Velocity().end(),
                      rosData.velocity.begin());
        }
    }
    { // efforts
        const size_t size = cisstData.Effort().size();
        if (size != 0) {
            rosData.effort.resize(size);
            std::copy(cisstData.Effort().begin(), cisstData.Effort().end(),
                      rosData.effort.begin());
        }
    }
}

void mtsCISSTToROS(const prmInputData & cisstData,
                   CISST_RAL_MSG(sensor_msgs, Joy) & rosData,
                   const std::string &)
{
    rosData.axes.resize(cisstData.AnalogInputs().size());
    rosData.buttons.resize(cisstData.DigitalInputs().size());
    std::copy(cisstData.AnalogInputs().begin(), cisstData.AnalogInputs().end(),
              rosData.axes.begin());
    std::copy(cisstData.DigitalInputs().begin(), cisstData.DigitalInputs().end(),
              rosData.buttons.begin());
}

void mtsCISSTToROS(const prmImageFrame & cisstData,
                   CISST_RAL_MSG(sensor_msgs, Image) & rosData,
                   const std::string &)
{
    rosData.width = cisstData.Width();
    rosData.height = cisstData.Height();
    rosData.step = rosData.width * cisstData.Channels();
    rosData.is_bigendian = false;

    if (cisstData.Channels() == 3) {
        rosData.encoding = sensor_msgs::image_encodings::RGB8;
    } else {
        rosData.encoding = sensor_msgs::image_encodings::MONO8;
    }

    rosData.data.resize(rosData.step * rosData.height);
    std::copy(cisstData.Data().begin(), cisstData.Data().end(), rosData.data.begin());
}

// Capitalization was changed in ROS2 :(
#if ROS1
#define ros_distortion(data) data.D
#define ros_intrinsic(data) data.K
#define ros_rectification(data) data.R
#define ros_projection(data) data.P
#elif ROS2
#define ros_distortion(data) data.d
#define ros_intrinsic(data) data.k
#define ros_rectification(data) data.r
#define ros_projection(data) data.p
#endif

void mtsCISSTToROS(const prmCameraInfo & cisstData,
                   CISST_RAL_MSG(sensor_msgs, CameraInfo) & rosData,
                   const std::string &)
{
    rosData.width = cisstData.Width();
    rosData.height = cisstData.Height();

    rosData.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    ros_distortion(rosData).resize(cisstData.DistortionParameters().size());

    Eigen::Map<Eigen::VectorXd> ros_mapped_distortion(ros_distortion(rosData).data(), ros_distortion(rosData).size());
    ros_mapped_distortion = cisstData.DistortionParameters();

    using ros_3x3_layout = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
    using ros_3x4_layout = Eigen::Matrix<double, 3, 4, Eigen::RowMajor>;
    Eigen::Map<ros_3x3_layout> ros_mapped_intrinsics(ros_intrinsic(rosData).data(), 3, 3);
    ros_mapped_intrinsics = cisstData.Intrinsic();

    Eigen::Map<ros_3x3_layout> ros_mapped_rectification(ros_rectification(rosData).data(), 3, 3);
    ros_mapped_rectification = cisstData.Rectification();

    Eigen::Map<ros_3x4_layout> ros_mapped_projection(ros_projection(rosData).data(), 3, 4);
    ros_mapped_projection = cisstData.Projection().matrix().topRows<3>();
}

void mtsCISSTToROS(const prmDepthMap & cisstData,
                   CISST_RAL_MSG(sensor_msgs, PointCloud2) & rosData,
                   const std::string &)
{
    rosData.width = cisstData.Width();
    rosData.height = cisstData.Height();
    rosData.is_bigendian = false;
    rosData.is_dense = false;

    bool has_color = cisstData.Color().size() > 0;

    sensor_msgs::PointCloud2Modifier modifier(rosData);
    if (has_color) {
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    } else {
        modifier.setPointCloud2FieldsByString(1, "xyz");
    }

    sensor_msgs::PointCloud2Iterator<float> iter_x(rosData, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(rosData, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(rosData, "z");

    float invalid = std::numeric_limits<float>::quiet_NaN();
    size_t size = cisstData.Width() * cisstData.Height();
    for (size_t i = 0; i < size; i++) {
        float x = cisstData.Points()(3*i + 0);
        float y = cisstData.Points()(3*i + 1);
        float z = cisstData.Points()(3*i + 2);

        if (!std::isinf(z)) {
            *iter_x = x;
            *iter_y = y;
            *iter_z = z;
        } else {
            *iter_x = *iter_y = *iter_z = invalid;
        }

        ++iter_x;
        ++iter_y;
        ++iter_z;
    }

    if (has_color) {
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(rosData, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(rosData, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(rosData, "b");

        for (size_t i = 0; i < size; i++) {
            float z = cisstData.Points()(3*i + 2);
            if (!std::isinf(z)) {
                *iter_r = cisstData.Color()(3*i + 0);
                *iter_g = cisstData.Color()(3*i + 1);
                *iter_b = cisstData.Color()(3*i + 2);
            }

            ++iter_r;
            ++iter_g;
            ++iter_b;
        }
    }
}

// ---------------------------------------------
// diagnostic_msgs
// ---------------------------------------------
void mtsCISSTToROS(const prmKeyValue & cisstData,
                   CISST_RAL_MSG(diagnostic_msgs, KeyValue) & rosData,
                   const std::string &)
{
    rosData.key = cisstData.Key;
    rosData.value = cisstData.Value;
}


// ---------------------------------------------
// std_srvs
// ---------------------------------------------
void mtsCISSTToROS(const bool & cisstData,
                   CISST_RAL_SRV_RES(std_srvs, Trigger) & rosData,
                   const std::string &)
{
    rosData.success = cisstData;
    rosData.message = "";
}

void mtsCISSTToROS(const std::string & cisstData,
                   CISST_RAL_SRV_RES(std_srvs, Trigger) & rosData,
                   const std::string &)
{
    rosData.success = true;
    rosData.message = cisstData;
}

// ---------------------------------------------
// cisst_msgs
// ---------------------------------------------
void mtsCISSTToROS(const prmPositionJointGet & cisstData,
                   CISST_RAL_MSG(cisst_msgs, DoubleVec) & rosData,
                   const std::string &)
{
    const size_t size = cisstData.Position().size();
    rosData.data.resize(size);
    for (size_t i = 0; i < size; ++i) {
        rosData.data[i] = cisstData.Position()(i);
    }
}

void mtsCISSTToROS(const Eigen::VectorXd& cisstData,
                   CISST_RAL_MSG(cisst_msgs, DoubleVec) & rosData,
                   const std::string &)
{
    const size_t size = cisstData.size();
    rosData.data.resize(size);
    for (size_t i = 0; i < size; ++i) {
        rosData.data[i] = cisstData(i);
    }
}

void mtsCISSTToROS(const mtsIntervalStatistics & cisstData,
                   CISST_RAL_MSG(cisst_msgs, IntervalStatistics) & rosData,
                   const std::string &)
{
    rosData.period_avg = cisstData.PeriodAvg();
    rosData.period_std_dev = cisstData.PeriodStdDev();
    rosData.period_min = cisstData.PeriodMin();
    rosData.period_max = cisstData.PeriodMax();
    rosData.compute_time_avg = cisstData.ComputeTimeAvg();
    rosData.compute_time_std_dev = cisstData.ComputeTimeStdDev();
    rosData.compute_time_min = cisstData.ComputeTimeMin();
    rosData.compute_time_max = cisstData.ComputeTimeMax();
    rosData.number_of_samples = cisstData.NumberOfSamples();
    rosData.number_of_overruns = cisstData.NumberOfOverruns();
    rosData.statistics_interval = cisstData.StatisticsInterval();
}

void mtsCISSTToROS(const Eigen::VectorXd& cisstData,
                   CISST_RAL_SRV_RES(cisst_msgs, ConvertFloat64Array) & rosData,
                   const std::string &)
{
    const size_t size = cisstData.size();
    if (size != 0) {
        rosData.output.resize(size);
        std::copy(cisstData.begin(), cisstData.end(),
                  rosData.output.begin());
    }
}
