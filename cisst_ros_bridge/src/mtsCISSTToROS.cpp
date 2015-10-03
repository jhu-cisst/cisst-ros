/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-21

  (C) Copyright 2013-2015 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include "cisst_ros_bridge/mtsCISSTToROS.h"

void mtsCISSTToROS(const double & cisstData, std_msgs::Float32 & rosData)
{
    rosData.data = cisstData;
}

void mtsCISSTToROS(const int & cisstData, std_msgs::Int32 & rosData)
{
    rosData.data = cisstData;
}

void mtsCISSTToROS(const bool & cisstData, std_msgs::Bool & rosData)
{
    rosData.data = cisstData;
}

void mtsCISSTToROS(const std::string & cisstData, std_msgs::String & rosData)
{
    rosData.data = cisstData;
}

void mtsCISSTToROS(const prmEventButton & cisstData, std_msgs::Bool & rosData)
{
    if (cisstData.Type() == prmEventButton::PRESSED)
        rosData.data = true;
    else if (cisstData.Type() == prmEventButton::RELEASED)
        rosData.data = false;
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

void mtsCISSTToROS(const prmPositionCartesianGet & cisstData, geometry_msgs::Transform & rosData)
{
    mtsCISSTToROSTransform(cisstData.Position(), rosData);
}

void mtsCISSTToROS(const prmPositionCartesianGet & cisstData, geometry_msgs::Pose & rosData)
{
    mtsCISSTToROSPose(cisstData.Position(), rosData);
}

void mtsCISSTToROS(const prmPositionCartesianGet & cisstData, geometry_msgs::PoseStamped & rosData)
{
    rosData.header.stamp = ros::Time::now();
    mtsCISSTToROS(cisstData, rosData.pose);
}


void mtsCISSTToROS(const vctFrm4x4 & cisstData, geometry_msgs::Pose & rosData)
{
    mtsCISSTToROSPose(cisstData, rosData);
}

void mtsCISSTToROS(const mtsFrm4x4 & cisstData, geometry_msgs::Pose & rosData)
{
    mtsCISSTToROSPose(cisstData, rosData);
}


void mtsCISSTToROS(const vctFrm3 & cisstData, geometry_msgs::Pose & rosData)
{
    mtsCISSTToROSPose(cisstData, rosData);
}

void mtsCISSTToROS(const vctFrm4x4 & cisstData, geometry_msgs::Transform & rosData)
{
    mtsCISSTToROSTransform(cisstData, rosData);
}

void mtsCISSTToROS(const mtsFrm4x4 & cisstData, geometry_msgs::Transform & rosData)
{
    mtsCISSTToROSTransform(cisstData, rosData);
}

void mtsCISSTToROS(const vctFrm3 & cisstData, geometry_msgs::Transform & rosData)
{
    mtsCISSTToROSTransform(cisstData, rosData);
}

void mtsCISSTToROS(const vct3 & cisstData, geometry_msgs::Vector3 & rosData)
{
    rosData.x = cisstData[0];
    rosData.y = cisstData[1];
    rosData.z = cisstData[2];
}

void mtsCISSTToROS(const vctMatRot3 & cisstData, geometry_msgs::Quaternion & rosData)
{
    vctQuatRot3 quat(cisstData, VCT_NORMALIZE);
    rosData.x = quat.X();
    rosData.y = quat.Y();
    rosData.z = quat.Z();
    rosData.w = quat.W();
}

void mtsCISSTToROS(const mtsDoubleVec & cisstData, geometry_msgs::Wrench & rosData)
{
    if (cisstData.size() != 6) {
        ROS_ERROR("Wrench data size error, should be 6");
        return;
    }

    rosData.force.x = cisstData.Element(0);
    rosData.force.y = cisstData.Element(1);
    rosData.force.z = cisstData.Element(2);

    rosData.torque.x = cisstData.Element(3);
    rosData.torque.y = cisstData.Element(4);
    rosData.torque.z = cisstData.Element(5);
}

void mtsCISSTToROS(const mtsDoubleVec & cisstData, geometry_msgs::WrenchStamped & rosData)
{
    if (cisstData.size() != 6) {
        ROS_ERROR("Wrench data size error, should be 6");
        return;
    }

    rosData.header.stamp = ros::Time::now();
    //    rosData.header.frame_id = "/one_ati_force_finger_tip_link";
    rosData.header.frame_id = "/one_tool_wrist_caudier_link_shaft";

    rosData.wrench.force.x = cisstData.Element(0);
    rosData.wrench.force.y = cisstData.Element(1);
    rosData.wrench.force.z = cisstData.Element(2);

    rosData.wrench.torque.x = cisstData.Element(3);
    rosData.wrench.torque.y = cisstData.Element(4);
    rosData.wrench.torque.z = cisstData.Element(5);
}

void mtsCISSTToROS(const mtsDoubleVec & cisstData, geometry_msgs::Vector3Stamped & rosData)
{
    if (cisstData.size() != 4) {
        ROS_ERROR("Vector data size error, should be 3");
        return;
    }

    rosData.header.stamp.sec = cisstData.Element(3);
    rosData.header.frame_id = "/one_psm_base_link";

    rosData.vector.x = cisstData.Element(0);
    rosData.vector.y = cisstData.Element(1);
    rosData.vector.z = cisstData.Element(2);
}

void mtsCISSTToROS(const prmVelocityCartesianGet & cisstData, geometry_msgs::Twist & rosData)
{
    if (cisstData.Valid()) {
        rosData.linear.x = cisstData.VelocityLinear().X();
        rosData.linear.y = cisstData.VelocityLinear().Y();
        rosData.linear.z = cisstData.VelocityLinear().Z();

        rosData.angular.x = cisstData.VelocityAngular().X();
        rosData.angular.y = cisstData.VelocityAngular().Y();
        rosData.angular.z = cisstData.VelocityAngular().Z();
    }
    else {
        rosData.linear.x = 0.0;
        rosData.linear.y = 0.0;
        rosData.linear.z = 0.0;

        rosData.angular.x = 0.0;
        rosData.angular.y = 0.0;
        rosData.angular.z = 0.0;
    }
}

void mtsCISSTToROS(const prmVelocityCartesianGet & cisstData, geometry_msgs::TwistStamped & rosData)
{
    rosData.header.stamp = ros::Time::now();
    mtsCISSTToROS(cisstData, rosData.twist);
}


// ---------------------------------------------
// sensor_msgs
// ---------------------------------------------
void mtsCISSTToROS(const vctDoubleVec & cisstData, sensor_msgs::JointState & rosData)
{
    rosData.header.stamp = ros::Time::now();
    rosData.name.resize(0);
    rosData.velocity.resize(0);
    rosData.effort.resize(0);
    const size_t size = cisstData.size();
    if (size != 0) {
        rosData.position.resize(size);
        std::copy(cisstData.begin(), cisstData.end(),
                  rosData.position.begin());
    }
}

void mtsCISSTToROS(const prmPositionJointGet & cisstData, sensor_msgs::JointState & rosData)
{
    rosData.header.stamp = ros::Time::now();
    rosData.name.resize(0);
    rosData.velocity.resize(0);
    rosData.effort.resize(0);
    const size_t size = cisstData.Position().size();
    if (size != 0) {
        rosData.position.resize(size);
        std::copy(cisstData.Position().begin(), cisstData.Position().end(),
                  rosData.position.begin());
    }
}

void mtsCISSTToROS(const prmVelocityJointGet & cisstData, sensor_msgs::JointState & rosData)
{
    rosData.header.stamp = ros::Time::now();
    rosData.name.resize(0);
    rosData.position.resize(0);
    rosData.effort.resize(0);
    const size_t size = cisstData.Velocity().size();
    if (size != 0) {
        rosData.velocity.resize(size);
        std::copy(cisstData.Velocity().begin(), cisstData.Velocity().end(),
                  rosData.velocity.begin());
    }
}

void mtsCISSTToROS(const prmStateJoint & cisstData, sensor_msgs::JointState & rosData)
{
    rosData.header.stamp = ros::Time::now();
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

void mtsCISSTToROS(const vctDoubleMat & cisstData, sensor_msgs::PointCloud & rosData)
{
    rosData.header.stamp = ros::Time::now();
    rosData.points.resize(cisstData.rows());

    for (size_t i = 0; i < cisstData.rows(); ++i) {
        rosData.points[i].x = cisstData.at(i, 0);
        rosData.points[i].y = cisstData.at(i, 1);
        rosData.points[i].z = cisstData.at(i, 2);
    }
}

// ---------------------------------------------
// cisst_msgs
// ---------------------------------------------
void mtsCISSTToROS(const prmPositionJointGet & cisstData, cisst_msgs::vctDoubleVec & rosData)
{
    const size_t size = cisstData.Position().size();
    rosData.data.resize(size);
    for (size_t i = 0; i < size; ++i) {
        rosData.data[i] = cisstData.Position().Element(i);
    }
}

void mtsCISSTToROS(const vctDoubleVec & cisstData, cisst_msgs::vctDoubleVec & rosData)
{
    const size_t size = cisstData.size();
    rosData.data.resize(size);
    for (size_t i = 0; i < size; ++i) {
        rosData.data[i] = cisstData.Element(i);
    }
}

void mtsCISSTToROS(const prmFixtureGainCartesianSet & cisstData, cisst_msgs::prmFixtureGainCartesianSet & rosData)
{
    geometry_msgs::Vector3 vector;    // holder for vector data
    geometry_msgs::Quaternion quaternion;   // holder for rotation data
    geometry_msgs::Pose pose;   // holder for pose data

    // vf pos/rot
    mtsCISSTToROS(cisstData.ForcePosition(), vector);
    rosData.ForcePosition = vector;
    mtsCISSTToROS(cisstData.ForceOrientation(), quaternion);
    rosData.ForceOrientation = quaternion;
    mtsCISSTToROS(cisstData.TorqueOrientation(), quaternion);
    rosData.TorqueOrientation = quaternion;

    // force gains
    mtsCISSTToROS(cisstData.PositionStiffnessPos(), vector);
    rosData.PosStiffPos = vector;

    mtsCISSTToROS(cisstData.PositionStiffnessNeg(), vector);
    rosData.PosStiffNeg = vector;

    mtsCISSTToROS(cisstData.PositionDampingPos(), vector);
    rosData.PosDampingPos = vector;

    mtsCISSTToROS(cisstData.PositionDampingNeg(), vector);
    rosData.PosDampingNeg = vector;

    mtsCISSTToROS(cisstData.ForceBiasPos(), vector);
    rosData.ForceBiasPos = vector;

    mtsCISSTToROS(cisstData.ForceBiasNeg(), vector);
    rosData.ForceBiasNeg = vector;

    // torque gains
    mtsCISSTToROS(cisstData.OrientationStiffnessPos(), vector);
    rosData.OriStiffPos = vector;

    mtsCISSTToROS(cisstData.OrientationStiffnessNeg(), vector);
    rosData.OriStiffNeg = vector;

    mtsCISSTToROS(cisstData.OrientationDampingPos(), vector);
    rosData.OriDampingPos = vector;

    mtsCISSTToROS(cisstData.OrientationDampingNeg(), vector);
    rosData.OriDampingNeg = vector;

    mtsCISSTToROS(cisstData.TorqueBiasPos(), vector);
    rosData.TorqueBiasPos = vector;

    mtsCISSTToROS(cisstData.TorqueBiasNeg(), vector);
    rosData.TorqueBiasNeg = vector;
}
