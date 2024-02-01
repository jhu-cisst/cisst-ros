/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-21

  (C) Copyright 2013-2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

#ifndef _mtsROSToCISST_h
#define _mtsROSToCISST_h

// cisst include
#include <cisstMultiTask/mtsManagerLocal.h>

#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstVector/vctDynamicMatrixTypes.h>

#include <cisstMultiTask/mtsVector.h>
#include <cisstMultiTask/mtsTransformationTypes.h>
#include <cisstMultiTask/mtsIntervalStatistics.h>

#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmForceTorqueJointSet.h>
#include <cisstParameterTypes/prmForceCartesianGet.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>
#include <cisstParameterTypes/prmVelocityJointSet.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>
#include <cisstParameterTypes/prmVelocityCartesianSet.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <cisstParameterTypes/prmInputData.h>
#include <cisstParameterTypes/prmKeyValue.h>
#include <cisstParameterTypes/prmStateJoint.h>

#include <cisst_ros_bridge/cisst_ral.h>

// ros includes
#if ROS1

#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <diagnostic_msgs/KeyValue.h>

// non standard messages
#include <cisst_msgs/DoubleVec.h>
#include <cisst_msgs/IntervalStatistics.h>
#include <cisst_msgs/ConvertFloat64Array.h>

#elif ROS2

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

// non standard messages
#include <cisst_msgs/msg/double_vec.hpp>
#include <cisst_msgs/msg/interval_statistics.hpp>
#include <cisst_msgs/srv/convert_float64_array.hpp>

#endif

namespace mts_ros_to_cisst {

    // cases for automatic header conversions
    // -4- ROS has header and child_frame_id, cisst has timestamp, valid, reference frame and moving frame
    // -3- ROS has header, cisst has timestamp, valid and reference frame
    // -2- ROS has header, cisst has timestamp, valid
    // -1- ROS has no header, cisst has timestamp, valid
    // -0- ROS has no header, cisst has nothing

    template <typename _rosType, typename _cisstType>
    void ros_header_to_cisst_header(const _rosType & rosData,
                                    _cisstType & cisstData,
                                    cisst_ral::node_ptr_t node)
    {
        const double cisstNow = mtsManagerLocal::GetInstance()->GetTimeServer().GetRelativeTime();
        // first check that header.stamp is not zero
        if (cisst_ral::time_is_zero(rosData.header.stamp)) {
            cisstData.SetTimestamp(cisstNow);
        } else {
            const double ageInSeconds = cisst_ral::age_in_seconds(node, rosData.header.stamp);
            if (ageInSeconds > 0.0) {
                cisstData.SetTimestamp(cisstNow - ageInSeconds);
            } else {
                cisstData.SetTimestamp(cisstNow);
            }
        }
        // always set as valid for now
        cisstData.SetValid(true);
    }

    // mts_ros_to_cisst_header_choice<N> is preferred to
    // mts_ros_to_cisst_header_choice<N-1>, but overload resolution will
    // fallback to mts_ros_to_cisst_header_choice<N-1> (then
    // mts_ros_to_cisst_header_choice<N-2> etc.) since it's a base class
    template<std::size_t _n>
    class header_choice: public header_choice<_n-1> {};
    template<>
    class header_choice<0> {};

    template <typename _rosType, typename _cisstType>
    auto header_impl(header_choice<4>,
                     const _rosType & rosData,
                     _cisstType & cisstData,
                     cisst_ral::node_ptr_t node)
        -> decltype(rosData.header,
                    rosData.child_frame_id,
                    cisstData.SetTimestamp(0.0),
                    cisstData.SetValid(true),
                    cisstData.SetReferenceFrame(""),
                    cisstData.SetMovingFrame(""),
                    void())
    {
        ros_header_to_cisst_header<_rosType, _cisstType>(rosData, cisstData, node);
        // set reference frame name
        cisstData.SetReferenceFrame(rosData.header.frame_id);
        cisstData.SetMovingFrame(rosData.child_frame_id);
    }

    template <typename _rosType, typename _cisstType>
    auto header_impl(header_choice<3>,
                     const _rosType & rosData,
                     _cisstType & cisstData,
                     cisst_ral::node_ptr_t node)
        -> decltype(rosData.header,
                    cisstData.SetTimestamp(0.0),
                    cisstData.SetValid(true),
                    cisstData.SetReferenceFrame(""),
                    void())
    {
        ros_header_to_cisst_header<_rosType, _cisstType>(rosData, cisstData, node);
        // set reference frame name
        cisstData.SetReferenceFrame(rosData.header.frame_id);
    }

    template <typename _rosType, typename _cisstType>
    auto header_impl(header_choice<2>,
                     const _rosType & rosData,
                     _cisstType & cisstData,
                     cisst_ral::node_ptr_t node)
        -> decltype(rosData.header,
                    cisstData.SetTimestamp(0.0),
                    cisstData.SetValid(true),
                    void())
    {
        ros_header_to_cisst_header<_rosType, _cisstType>(rosData, cisstData, node);
    }

    template <typename _rosType, typename _cisstType>
    auto header_impl(header_choice<1>,
                     const _rosType &,
                     _cisstType & cisstData,
                     cisst_ral::node_ptr_t)
        -> decltype(cisstData.SetTimestamp(0.0),
                    cisstData.SetValid(true),
                    void())
    {
        const double cisstNow = mtsManagerLocal::GetInstance()->GetTimeServer().GetRelativeTime();
        cisstData.SetTimestamp(cisstNow);
        // always set as valid for now
        cisstData.SetValid(true);
    }

    // last case, nothing to do
    template <typename _rosType, typename _cisstType>
    void header_impl(header_choice<0>,
                     const _rosType &, _cisstType &, cisst_ral::node_ptr_t)
    {
    }

    template <typename _rosType, typename _cisstType>
    void header(const _rosType & rosData, _cisstType & cisstData, cisst_ral::node_ptr_t node)
    {
        header_impl<_rosType, _cisstType>(header_choice<4>(), rosData, cisstData, node);
    }
}


// helper functions
template <typename _cisstFrame>
void mtsROSTransformToCISST(const CISST_RAL_MSG(geometry_msgs, Transform) & rosTransform, _cisstFrame & cisstFrame)
{
    cisstFrame.Translation().X() = rosTransform.translation.x;
    cisstFrame.Translation().Y() = rosTransform.translation.y;
    cisstFrame.Translation().Z() = rosTransform.translation.z;
    vctQuatRot3 quat;
    quat.X() = rosTransform.rotation.x;
    quat.Y() = rosTransform.rotation.y;
    quat.Z() = rosTransform.rotation.z;
    quat.W() = rosTransform.rotation.w;
    vctMatRot3 rotation(quat, VCT_NORMALIZE);
    cisstFrame.Rotation().Assign(rotation);
}

template <typename _cisstFrame>
void mtsROSPoseToCISST(const CISST_RAL_MSG(geometry_msgs, Pose) & rosPose, _cisstFrame & cisstFrame)
{
    cisstFrame.Translation().X() = rosPose.position.x;
    cisstFrame.Translation().Y() = rosPose.position.y;
    cisstFrame.Translation().Z() = rosPose.position.z;
    vctQuatRot3 quat;
    quat.X() = rosPose.orientation.x;
    quat.Y() = rosPose.orientation.y;
    quat.Z() = rosPose.orientation.z;
    quat.W() = rosPose.orientation.w;
    vctMatRot3 rotation(quat, VCT_NORMALIZE);
    cisstFrame.Rotation().Assign(rotation);
}


// std_msgs
void mtsROSToCISST(const CISST_RAL_MSG(std_msgs, Float32) & rosData,
                   double & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(std_msgs, Float64) & rosData,
                   double & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(std_msgs, Int32) & rosData,
                   int & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(std_msgs, Bool) & rosData,
                   bool & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(std_msgs, String) & rosData,
                   std::string & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(std_msgs, String) & rosData,
                   mtsMessage & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(std_msgs, Float64MultiArray) & rosData,
                   vctDoubleVec & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(std_msgs, Float64MultiArray) & rosData,
                   vctDoubleMat & cisstData);

// geometry_msgs
void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Vector3) & rosData,
                   vct3 & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Quaternion) & rosData,
                   vctMatRot3 & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Pose) & rosData,
                   prmPositionCartesianGet & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Pose) & rosData,
                   prmPositionCartesianSet & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, PoseStamped) & rosData,
                   prmPositionCartesianGet & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, PoseStamped) & rosData,
                   prmPositionCartesianSet & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, TransformStamped) & rosData,
                   prmPositionCartesianGet & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, TransformStamped) & rosData,
                   prmPositionCartesianSet & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Pose) & rosData,
                   vctFrm3 & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, PoseStamped) & rosData,
                   vctFrm3 & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, PoseStamped) & rosData,
                   vctFrm4x4 & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Pose) & rosData,
                   vctFrm4x4 & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Pose) & rosData,
                   mtsFrm4x4 & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Transform) & rosData,
                   prmPositionCartesianGet & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Transform) & rosData,
                   vctFrm3 & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Transform) & rosData,
                   vctFrm4x4 & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Transform) & rosData,
                   mtsFrm4x4 & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, TransformStamped) & rosData,
                   mtsFrm4x4 & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Wrench) & rosData,
                   prmForceCartesianGet & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, WrenchStamped) & rosData,
                   prmForceCartesianGet & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Wrench) & rosData,
                   prmForceCartesianSet & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, WrenchStamped) & rosData,
                   prmForceCartesianSet & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Wrench) & rosData,
                   mtsDoubleVec & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, WrenchStamped) & rosData,
                   mtsDoubleVec & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Twist) & rosData,
                   prmVelocityCartesianGet & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, TwistStamped) & rosData,
                   prmVelocityCartesianGet & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Twist) & rosData,
                   prmVelocityCartesianSet & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, TwistStamped) & rosData,
                   prmVelocityCartesianSet & cisstData);

// sensor_msgs
void mtsROSToCISST(const CISST_RAL_MSG(sensor_msgs, JointState) & rosData,
                   prmPositionJointSet & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(sensor_msgs, JointState) & rosData,
                   prmForceTorqueJointSet & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(sensor_msgs, JointState) & rosData,
                   prmVelocityJointSet & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(sensor_msgs, JointState) & rosData,
                   prmStateJoint & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(sensor_msgs, Joy) & rosData,
                   prmEventButton & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(sensor_msgs, Joy) & rosData,
                   prmInputData & cisstData);

// diagnostic_msgs
void mtsROSToCISST(const CISST_RAL_MSG(diagnostic_msgs, KeyValue) & rosData,
                   prmKeyValue & cisstData);

// cisst_msgs
void mtsROSToCISST(const CISST_RAL_MSG(cisst_msgs, DoubleVec) & rosData,
                   prmPositionJointSet & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(cisst_msgs, DoubleVec) & rosData,
                   vctDoubleVec & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(cisst_msgs, IntervalStatistics) & rosData,
                   mtsIntervalStatistics & cisstData);
void mtsROSToCISST(const CISST_RAL_SRV_REQ(cisst_msgs, ConvertFloat64Array) & rosData,
                   vctDoubleVec & cisstData);

#endif // _mtsROSToCISST_h
