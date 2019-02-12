/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-21

  (C) Copyright 2013-2019 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <cisstParameterTypes/prmVelocityJointGet.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>
#include <cisstParameterTypes/prmForceCartesianGet.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <cisstParameterTypes/prmCartesianImpedanceGains.h>
#include <cisstParameterTypes/prmInputData.h>
#include <cisstParameterTypes/prmKeyValue.h>

// ros include
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Joy.h>
#include <diagnostic_msgs/KeyValue.h>
#include <std_srvs/Trigger.h>

// non standard messages
#include <cisst_msgs/vctDoubleVec.h>
#include <cisst_msgs/prmCartesianImpedanceGains.h>
#include <cisst_msgs/mtsIntervalStatistics.h>
#include <cisst_msgs/BoolStamped.h>

// helper functions
template <typename _cisstFrame>
void mtsCISSTToROSPose(const _cisstFrame & cisstFrame, geometry_msgs::Pose & rosPose)
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

template <typename _cisstFrame>
void mtsCISSTToROSTransform(const _cisstFrame & cisstFrame, geometry_msgs::Transform & rosTransform)
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

template <typename _cisstType, typename _rosType>
bool mtsCISSTToROSHeader(const _cisstType & cisstData, _rosType & rosData, const std::string & debugInfo)
{
    if (!cisstData.Valid()) {
        return false;
    }
    try {
        const double cisstDataTime = cisstData.Timestamp();
        if (cisstDataTime > 0.0) {
            const double age =
                mtsManagerLocal::GetInstance()->GetTimeServer().GetRelativeTime()
                - cisstDataTime;
            if (age > 0.0) {
                rosData.header.stamp = ros::Time::now() - ros::Duration(age);
            } else {
                rosData.header.stamp = ros::Time::now();
            }
        } else {
            rosData.header.stamp = ros::Time::now();
        }
    } catch (std::exception e) {
        CMN_LOG_RUN_ERROR << "mtsCISSTToROSHeader caught exception \""
                          << e.what()
                          << "\"while computing timestamp for \"" << debugInfo
                          << "\"" << std::endl;
        return false;
    }
    return true;
}

template <typename _rosType>
bool mtsCISSTToROSHeader(_rosType & rosData, const std::string & CMN_UNUSED(debugInfo))
{
    rosData.header.stamp = ros::Time::now();
    return true;
}

// std_msgs
bool mtsCISSTToROS(const double & cisstData, std_msgs::Float32 & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const double & cisstData, std_msgs::Float64 & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const int & cisstData, std_msgs::Int32 & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const bool & cisstData, std_msgs::Bool & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const bool & cisstData, cisst_msgs::BoolStamped & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const bool & cisstData, sensor_msgs::Joy & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const std::string & cisstData, std_msgs::String & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const mtsMessage & cisstData, std_msgs::String & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const prmEventButton & cisstData, std_msgs::Bool & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const prmEventButton & cisstData, cisst_msgs::BoolStamped & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const prmEventButton & cisstData, sensor_msgs::Joy & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const vctDoubleVec & cisstData, std_msgs::Float64MultiArray & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const vctDoubleMat & cisstData, std_msgs::Float64MultiArray & rosData, const std::string & debugInfo);

// geometry_msgs
bool mtsCISSTToROS(const prmPositionCartesianGet & cisstData, geometry_msgs::Transform & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const prmPositionCartesianGet & cisstData, geometry_msgs::TransformStamped & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const prmPositionCartesianGet & cisstData, geometry_msgs::Pose & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const prmPositionCartesianGet & cisstData, geometry_msgs::PoseStamped & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const vctFrm4x4 & cisstData, geometry_msgs::Pose & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const mtsFrm4x4 & cisstData, geometry_msgs::Pose & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const vctFrm3 & cisstData, geometry_msgs::Pose & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const vctFrm4x4 & cisstData, geometry_msgs::Transform & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const mtsFrm4x4 & cisstData, geometry_msgs::Transform & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const mtsFrm4x4 & cisstData, geometry_msgs::TransformStamped & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const vctFrm3 & cisstData, geometry_msgs::Transform & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const vct3 & cisstData, geometry_msgs::Vector3 & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const vctMatRot3 & cisstData, geometry_msgs::Quaternion & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const mtsDoubleVec & cisstData, geometry_msgs::Wrench & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const mtsDoubleVec & cisstData, geometry_msgs::WrenchStamped & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const mtsDoubleVec & cisstData, geometry_msgs::Vector3Stamped & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const prmVelocityCartesianGet & cisstData, geometry_msgs::Twist & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const prmVelocityCartesianGet & cisstData, geometry_msgs::TwistStamped & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const prmForceCartesianGet & cisstData, geometry_msgs::Wrench & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const prmForceCartesianGet & cisstData, geometry_msgs::WrenchStamped & rosData, const std::string & debugInfo);

// sensor_msgs
bool mtsCISSTToROS(const vctDoubleVec & cisstData, sensor_msgs::JointState & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const prmPositionJointGet & cisstData, sensor_msgs::JointState & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const prmVelocityJointGet & cisstData, sensor_msgs::JointState & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const prmStateJoint & cisstData, sensor_msgs::JointState & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const vctDoubleMat & cisstData, sensor_msgs::PointCloud & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const std::vector<vct3> & cisstData, sensor_msgs::PointCloud & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const prmInputData & cisstData, sensor_msgs::Joy & rosData, const std::string & debugInfo);

// diagnostic_msgs
bool mtsCISSTToROS(const prmKeyValue & cisstData, diagnostic_msgs::KeyValue & rosData, const std::string & debugInfo);

// std_srvs
bool mtsCISSTToROS(const bool & cisstData, std_srvs::Trigger::Response & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const std::string & cisstData, std_srvs::Trigger::Response & rosData, const std::string & debugInfo);

// cisst_msgs
bool mtsCISSTToROS(const prmPositionJointGet & cisstData, cisst_msgs::vctDoubleVec & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const vctDoubleVec & cisstData, cisst_msgs::vctDoubleVec & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const prmCartesianImpedanceGains & cisstData,
                   cisst_msgs::prmCartesianImpedanceGains & rosData, const std::string & debugInfo);
bool mtsCISSTToROS(const mtsIntervalStatistics & cisstData,
                   cisst_msgs::mtsIntervalStatistics & rosData, const std::string & debugInfo);

#endif // _mtsCISSTToROS_h
