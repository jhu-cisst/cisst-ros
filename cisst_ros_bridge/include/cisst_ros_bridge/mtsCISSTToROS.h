/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-21

  (C) Copyright 2013-2023 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

#ifndef _mtsCISSTToROS_h
#define _mtsCISSTToROS_h

// cisst include
#include <cisstVector/vctDynamicVectorTypes.h>

#include <cisstMultiTask/mtsManagerLocal.h>
#include <cisstMultiTask/mtsVector.h>
#include <cisstMultiTask/mtsTransformationTypes.h>
#include <cisstMultiTask/mtsIntervalStatistics.h>

#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmVelocityJointGet.h>
#include <cisstParameterTypes/prmVelocityJointSet.h>
#include <cisstParameterTypes/prmForceTorqueJointSet.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianArrayGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>
#include <cisstParameterTypes/prmForceCartesianGet.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <cisstParameterTypes/prmInputData.h>
#include <cisstParameterTypes/prmKeyValue.h>
#include <cisstParameterTypes/prmOperatingState.h>
#include <cisstParameterTypes/prmImageFrame.h>
#include <cisstParameterTypes/prmCameraInfo.h>
#include <cisstParameterTypes/prmDepthMap.h>

#include <cisst_ros_bridge/cisst_ral.h>

// ros includes
#if ROS1

#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <diagnostic_msgs/KeyValue.h>
#include <std_srvs/Trigger.h>

// non standard messages
#include <cisst_msgs/DoubleVec.h>
#include <cisst_msgs/IntervalStatistics.h>
#include <cisst_msgs/BoolStamped.h>
#include <cisst_msgs/ConvertFloat64Array.h>

#elif ROS2

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <std_srvs/srv/trigger.hpp>

// non standard messages
#include <cisst_msgs/msg/double_vec.hpp>
#include <cisst_msgs/msg/interval_statistics.hpp>
#include <cisst_msgs/msg/bool_stamped.hpp>
#include <cisst_msgs/srv/convert_float64_array.hpp>

#endif

namespace mts_cisst_to_ros {

    // cases for automatic header conversions
    // -4- ROS has header and child_frame_id, cisst has valid, timestamp, reference frame and moving frame
    // -3- ROS has header, cisst has valid, timestamp and reference frame
    // -2- ROS has header, cisst has valid, timestamp,
    // -1- ROS has header, cisst nothing
    // -0- ROS has no header, cisst has nothing

    template <typename _cisstType, typename _rosType>
    void cisst_header_to_ros_header(const _cisstType & cisstData, _rosType & rosData,
                                    cisst_ral::node_ptr_t node,
                                    const std::string & debugInfo)
    {
        if (!cisstData.Valid()) {
            CISST_RAL_TIME_SET_TO_ZERO(rosData.header.stamp);
            return;
        }
        try {
            const double cisstDataTime = cisstData.Timestamp();
            if (cisstDataTime > 0.0) {
                const double age =
                    mtsManagerLocal::GetInstance()->GetTimeServer().GetRelativeTime()
                    - cisstDataTime;
                if (age > 0.0) {
                    rosData.header.stamp = cisst_ral::now(node) - cisst_ral::duration_from_seconds(age);
                } else {
                    rosData.header.stamp = cisst_ral::now(node);
                }
            } else {
                rosData.header.stamp = cisst_ral::now(node);
            }
        } catch (std::exception & e) {
            CMN_LOG_RUN_ERROR << "cisst_header_to_ros_header caught exception \""
                              << e.what()
                              << "\"while computing timestamp for \"" << debugInfo
                              << "\"" << std::endl;
        }
    }

    // mts_ros_to_ros_header_choice<N> is preferred to
    // mts_ros_to_ros_header_choice<N-1>, but overload resolution will
    // fallback to mts_ros_to_ros_header_choice<N-1> (then
    // mts_ros_to_ros_header_choice<N-2> etc.) since it's a base class
    template<std::size_t _n>
    class header_choice: public header_choice<_n-1> {};
    template<>
    class header_choice<0> {};

    template <typename _cisstType, typename _rosType>
    auto header_impl(header_choice<4>,
                     const _cisstType & cisstData,
                     _rosType & rosData,
                     cisst_ral::node_ptr_t node,
                     const std::string & debugInfo)
        -> decltype(cisstData.Valid(),
                    cisstData.Timestamp(),
                    cisstData.ReferenceFrame(),
                    cisstData.MovingFrame(),
                    rosData.header,
                    rosData.child_frame_id,
                    false)
    {
        cisst_header_to_ros_header<_cisstType, _rosType>(cisstData, rosData, node, debugInfo);
        // set reference frame name
        rosData.header.frame_id = cisstData.ReferenceFrame();
        rosData.child_frame_id = cisstData.MovingFrame();
        return true;
    }

    template <typename _cisstType, typename _rosType>
    auto header_impl(header_choice<3>,
                     const _cisstType & cisstData,
                     _rosType & rosData,
                     cisst_ral::node_ptr_t node,
                     const std::string & debugInfo)
        -> decltype(cisstData.Valid(),
                    cisstData.Timestamp(),
                    cisstData.ReferenceFrame(),
                    rosData.header,
                    false)
    {
        cisst_header_to_ros_header<_cisstType, _rosType>(cisstData, rosData, node, debugInfo);
        // set reference frame name
        rosData.header.frame_id = cisstData.ReferenceFrame();
        return true;
    }

    template <typename _cisstType, typename _rosType>
    auto header_impl(header_choice<2>,
                     const _cisstType & cisstData,
                     _rosType & rosData,
                     cisst_ral::node_ptr_t node,
                     const std::string & debugInfo)
        -> decltype(cisstData.Valid(),
                    cisstData.Timestamp(),
                    rosData.header,
                    false)
    {
        cisst_header_to_ros_header<_cisstType, _rosType>(cisstData, rosData, node, debugInfo);
        return true;
    }

    template <typename _cisstType, typename _rosType>
    auto header_impl(header_choice<1>,
                     const _cisstType &,
                     _rosType & rosData,
                     cisst_ral::node_ptr_t node,
                     const std::string &)
        -> decltype(rosData.header,
                    false)
    {
        rosData.header.stamp = cisst_ral::now(node);
        return true;
    }

    // last case, nothing to do
    template <typename _cisstType, typename _rosType>
    bool header_impl(header_choice<0>,
                     const _cisstType &,
                     _rosType &,
                     cisst_ral::node_ptr_t,
                     const std::string &)
    {
        return true;
    }

    template <typename _cisstType, typename _rosType>
    bool header(const _cisstType & cisstData,
                _rosType & rosData,
                cisst_ral::node_ptr_t node,
                const std::string & debugInfo)
    {
        return header_impl<_cisstType, _rosType>(header_choice<4>(), cisstData, rosData, node, debugInfo);
    }
}


// helper functions
template <typename _cisstFrame, typename _rosPose>
void mtsCISSTToROSPose(const _cisstFrame & cisstFrame, _rosPose & rosPose)
{
    vctQuatRot3 quat(cisstFrame.Rotation(), VCT_NORMALIZE);
    rosPose.orientation.x = quat.X();
    rosPose.orientation.y = quat.Y();
    rosPose.orientation.z = quat.Z();
    rosPose.orientation.w = quat.W();
    rosPose.position.x = cisstFrame.Translation().X();
    rosPose.position.y = cisstFrame.Translation().Y();
    rosPose.position.z = cisstFrame.Translation().Z();
}

template <typename _cisstFrame, typename _rosTransform>
void mtsCISSTToROSTransform(const _cisstFrame & cisstFrame, _rosTransform & rosTransform)
{
    vctQuatRot3 quat(cisstFrame.Rotation(), VCT_NORMALIZE);
    rosTransform.rotation.x = quat.X();
    rosTransform.rotation.y = quat.Y();
    rosTransform.rotation.z = quat.Z();
    rosTransform.rotation.w = quat.W();
    rosTransform.translation.x = cisstFrame.Translation().X();
    rosTransform.translation.y = cisstFrame.Translation().Y();
    rosTransform.translation.z = cisstFrame.Translation().Z();
}

template <typename _cisstVector, typename _rosTwist>
void mtsCISSTToROSTwist(const _cisstVector & cisstVector, _rosTwist & rosTwist)
{
    rosTwist.linear.x = cisstVector.Element(0);
    rosTwist.linear.y = cisstVector.Element(1);
    rosTwist.linear.z = cisstVector.Element(2);
    rosTwist.angular.x = cisstVector.Element(3);
    rosTwist.angular.y = cisstVector.Element(4);
    rosTwist.angular.z = cisstVector.Element(5);
}

template <typename _cisstVector, typename _rosWrench>
void mtsCISSTToROSWrench(const _cisstVector & cisstVector, _rosWrench & rosWrench)
{
    rosWrench.force.x = cisstVector.Element(0);
    rosWrench.force.y = cisstVector.Element(1);
    rosWrench.force.z = cisstVector.Element(2);
    rosWrench.torque.x = cisstVector.Element(3);
    rosWrench.torque.y = cisstVector.Element(4);
    rosWrench.torque.z = cisstVector.Element(5);
}

// std_msgs
void mtsCISSTToROS(const double & cisstData,
                   CISST_RAL_MSG(std_msgs, Float32) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const double & cisstData,
                   CISST_RAL_MSG(std_msgs, Float64) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const int & cisstData,
                   CISST_RAL_MSG(std_msgs, Int32) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const bool & cisstData,
                   CISST_RAL_MSG(std_msgs, Bool) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const bool & cisstData,
                   CISST_RAL_MSG(cisst_msgs, BoolStamped) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const bool & cisstData,
                   CISST_RAL_MSG(sensor_msgs, Joy) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const std::string & cisstData,
                   CISST_RAL_MSG(std_msgs, String) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const mtsMessage & cisstData,
                   CISST_RAL_MSG(std_msgs, String) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmEventButton & cisstData,
                   CISST_RAL_MSG(std_msgs, Bool) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmEventButton & cisstData,
                   CISST_RAL_MSG(cisst_msgs, BoolStamped) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmEventButton & cisstData,
                   CISST_RAL_MSG(sensor_msgs, Joy) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const vctDoubleVec & cisstData,
                   CISST_RAL_MSG(std_msgs, Float64MultiArray) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const vctDoubleMat & cisstData,
                   CISST_RAL_MSG(std_msgs, Float64MultiArray) & rosData,
                   const std::string & debugInfo);

// geometry_msgs
void mtsCISSTToROS(const prmPositionCartesianGet & cisstData,
                   CISST_RAL_MSG(geometry_msgs, Transform) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmPositionCartesianGet & cisstData,
                   CISST_RAL_MSG(geometry_msgs, TransformStamped) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmPositionCartesianGet & cisstData,
                   CISST_RAL_MSG(geometry_msgs, Pose) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmPositionCartesianGet & cisstData,
                   CISST_RAL_MSG(geometry_msgs, PoseStamped) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmPositionCartesianArrayGet & cisstData,
                   CISST_RAL_MSG(geometry_msgs, PoseArray) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmPositionCartesianSet & cisstData,
                   CISST_RAL_MSG(geometry_msgs, Pose) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmPositionCartesianSet & cisstData,
                   CISST_RAL_MSG(geometry_msgs, PoseStamped) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const vctFrm4x4 & cisstData,
                   CISST_RAL_MSG(geometry_msgs, Pose) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const mtsFrm4x4 & cisstData,
                   CISST_RAL_MSG(geometry_msgs, Pose) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const vctFrm4x4 & cisstData,
                   CISST_RAL_MSG(geometry_msgs, Transform) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const mtsFrm4x4 & cisstData,
                   CISST_RAL_MSG(geometry_msgs, Transform) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmPositionCartesianSet & cisstData,
                   CISST_RAL_MSG(geometry_msgs, TransformStamped) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const mtsFrm4x4 & cisstData,
                   CISST_RAL_MSG(geometry_msgs, TransformStamped) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const vctFrm3 & cisstData,
                   CISST_RAL_MSG(geometry_msgs, Transform) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const vctFrm3 & cisstData,
                   CISST_RAL_MSG(geometry_msgs, PoseStamped) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const vctFrm3 & cisstData,
                   CISST_RAL_MSG(geometry_msgs, Pose) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const vct3 & cisstData,
                   CISST_RAL_MSG(geometry_msgs, Vector3) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const vctMatRot3 & cisstData,
                   CISST_RAL_MSG(geometry_msgs, Quaternion) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const vctMatRot3 & cisstData,
                   CISST_RAL_MSG(geometry_msgs, QuaternionStamped) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const vct6 & cisstData,
                   CISST_RAL_MSG(geometry_msgs, Wrench) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const vct6 & cisstData,
                   CISST_RAL_MSG(geometry_msgs, WrenchStamped) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const mtsDoubleVec & cisstData,
                   CISST_RAL_MSG(geometry_msgs, Wrench) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const mtsDoubleVec & cisstData,
                   CISST_RAL_MSG(geometry_msgs, WrenchStamped) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const mtsDoubleVec & cisstData,
                   CISST_RAL_MSG(geometry_msgs, Vector3Stamped) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmVelocityCartesianGet & cisstData,
                   CISST_RAL_MSG(geometry_msgs, Twist) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmVelocityCartesianGet & cisstData,
                   CISST_RAL_MSG(geometry_msgs, TwistStamped) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmForceCartesianGet & cisstData,
                   CISST_RAL_MSG(geometry_msgs, Wrench) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmForceCartesianGet & cisstData,
                   CISST_RAL_MSG(geometry_msgs, WrenchStamped) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmForceCartesianSet & cisstData,
                   CISST_RAL_MSG(geometry_msgs, Wrench) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmForceCartesianSet & cisstData,
                   CISST_RAL_MSG(geometry_msgs, WrenchStamped) & rosData,
                   const std::string & debugInfo);

// sensor_msgs
void mtsCISSTToROS(const vctDoubleVec & cisstData,
                   CISST_RAL_MSG(sensor_msgs, JointState) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmPositionJointGet & cisstData,
                   CISST_RAL_MSG(sensor_msgs, JointState) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmPositionJointSet & cisstData,
                   CISST_RAL_MSG(sensor_msgs, JointState) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmVelocityJointGet & cisstData,
                   CISST_RAL_MSG(sensor_msgs, JointState) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmVelocityJointSet & cisstData,
                   CISST_RAL_MSG(sensor_msgs, JointState) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmForceTorqueJointSet & cisstData,
                   CISST_RAL_MSG(sensor_msgs, JointState) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmStateJoint & cisstData,
                   CISST_RAL_MSG(sensor_msgs, JointState) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const vctDoubleMat & cisstData,
                   CISST_RAL_MSG(sensor_msgs, PointCloud) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const std::vector<vct3> & cisstData,
                   CISST_RAL_MSG(sensor_msgs, PointCloud) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmInputData & cisstData,
                   CISST_RAL_MSG(sensor_msgs, Joy) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmImageFrame & cisstData,
                   CISST_RAL_MSG(sensor_msgs, Image) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmCameraInfo & cisstData,
                   CISST_RAL_MSG(sensor_msgs, CameraInfo) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmDepthMap & cisstData,
                   CISST_RAL_MSG(sensor_msgs, PointCloud2) & rosData,
                   const std::string & debugInfo);

// diagnostic_msgs
void mtsCISSTToROS(const prmKeyValue & cisstData,
                   CISST_RAL_MSG(diagnostic_msgs, KeyValue) & rosData,
                   const std::string & debugInfo);

// std_srvs
void mtsCISSTToROS(const bool & cisstData,
                   CISST_RAL_SRV_RES(std_srvs, Trigger) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const std::string & cisstData,
                   CISST_RAL_SRV_RES(std_srvs, Trigger) & rosData,
                   const std::string & debugInfo);

// cisst_msgs
void mtsCISSTToROS(const prmPositionJointGet & cisstData,
                   CISST_RAL_MSG(cisst_msgs, DoubleVec) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const vctDoubleVec & cisstData,
                   CISST_RAL_MSG(cisst_msgs, DoubleVec) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const mtsIntervalStatistics & cisstData,
                   CISST_RAL_MSG(cisst_msgs, IntervalStatistics) & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const vctDoubleVec & cisstData,
                   CISST_RAL_SRV_RES(cisst_msgs, ConvertFloat64Array) & rosData,
                   const std::string & debugInfo);

#endif // _mtsCISSTToROS_h
