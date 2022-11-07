/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-21

  (C) Copyright 2013-2022 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include "cisst_ros_bridge/mtsCISSTToROS.h"

void mtsCISSTToROS(const double & cisstData, std_msgs::Float32 & rosData,
                   const std::string &)
{
    rosData.data = cisstData;
}

void mtsCISSTToROS(const double & cisstData, std_msgs::Float64 & rosData,
                   const std::string &)
{
    rosData.data = cisstData;
}

void mtsCISSTToROS(const int & cisstData, std_msgs::Int32 & rosData,
                   const std::string &)
{
    rosData.data = cisstData;
}

void mtsCISSTToROS(const bool & cisstData, std_msgs::Bool & rosData,
                   const std::string &)
{
    rosData.data = cisstData;
}

void mtsCISSTToROS(const bool & cisstData, cisst_msgs::BoolStamped & rosData,
                   const std::string &)
{
    rosData.data = cisstData;
}

void mtsCISSTToROS(const bool & cisstData, sensor_msgs::Joy & rosData,
                   const std::string &)
{
    rosData.axes.resize(0);
    rosData.buttons.resize(1);
    rosData.buttons[0] = cisstData;
}

void mtsCISSTToROS(const std::string & cisstData, std_msgs::String & rosData,
                   const std::string &)
{
    rosData.data = cisstData;
}

void mtsCISSTToROS(const mtsMessage & cisstData, std_msgs::String & rosData,
                   const std::string &)
{
    rosData.data = cisstData.Message;
}

void mtsCISSTToROS(const prmEventButton & cisstData, std_msgs::Bool & rosData,
                   const std::string &)
{
    if (cisstData.Type() == prmEventButton::PRESSED) {
        rosData.data = true;
    } else if (cisstData.Type() == prmEventButton::RELEASED) {
        rosData.data = false;
    }
}

void mtsCISSTToROS(const prmEventButton & cisstData, cisst_msgs::BoolStamped & rosData,
                   const std::string &)
{
    if (cisstData.Type() == prmEventButton::PRESSED) {
        rosData.data = true;
    } else if (cisstData.Type() == prmEventButton::RELEASED) {
        rosData.data = false;
    }
}

void mtsCISSTToROS(const prmEventButton & cisstData, sensor_msgs::Joy & rosData,
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

void mtsCISSTToROS(const vctDoubleMat & cisstData, std_msgs::Float64MultiArray & rosData,
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
        std::copy(cisstData.begin(), cisstData.end(),
                  rosData.data.begin());
    }
}

void mtsCISSTToROS(const vctDoubleVec & cisstData, std_msgs::Float64MultiArray & rosData,
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

void mtsCISSTToROS(const prmPositionCartesianGet & cisstData, geometry_msgs::Transform & rosData,
                   const std::string &)
{
    mtsCISSTToROSTransform(cisstData.Position(), rosData);
}

void mtsCISSTToROS(const prmPositionCartesianGet & cisstData, geometry_msgs::TransformStamped & rosData,
                   const std::string &)
{
    mtsCISSTToROSTransform(cisstData.Position(), rosData.transform);
}

void mtsCISSTToROS(const prmPositionCartesianGet & cisstData, geometry_msgs::Pose & rosData,
                   const std::string &)
{
    mtsCISSTToROSPose(cisstData.Position(), rosData);
}

void mtsCISSTToROS(const prmPositionCartesianGet & cisstData, geometry_msgs::PoseStamped & rosData,
                   const std::string &)
{
    mtsCISSTToROSPose(cisstData.Position(), rosData.pose);
}

void mtsCISSTToROS(const prmPositionCartesianArrayGet & cisstData, geometry_msgs::PoseArray & rosData,
                   const std::string &)
{
    typedef std::vector<vctFrm3>::const_iterator IteratorType;
    const IteratorType end = cisstData.Positions().end();
    IteratorType iter;
    size_t index = 0;
    rosData.poses.resize(cisstData.Positions().size());
    for (iter = cisstData.Positions().begin();
         iter != end;
         ++iter, ++index) {
        mtsCISSTToROSPose(*iter, rosData.poses[index]);
    }
}

void mtsCISSTToROS(const prmPositionCartesianSet & cisstData, geometry_msgs::Pose & rosData,
                   const std::string &)
{
    mtsCISSTToROSPose(cisstData.Goal(), rosData);
}

void mtsCISSTToROS(const prmPositionCartesianSet & cisstData, geometry_msgs::PoseStamped & rosData,
                   const std::string &)
{
    mtsCISSTToROSPose(cisstData.Goal(), rosData.pose);
}

void mtsCISSTToROS(const vctFrm4x4 & cisstData, geometry_msgs::Pose & rosData,
                   const std::string &)
{
    mtsCISSTToROSPose(cisstData, rosData);
}

void mtsCISSTToROS(const mtsFrm4x4 & cisstData, geometry_msgs::Pose & rosData,
                   const std::string &)
{
    mtsCISSTToROSPose(cisstData, rosData);
}

void mtsCISSTToROS(const vctFrm4x4 & cisstData, geometry_msgs::Transform & rosData,
                   const std::string &)
{
    mtsCISSTToROSTransform(cisstData, rosData);
}

void mtsCISSTToROS(const mtsFrm4x4 & cisstData, geometry_msgs::Transform & rosData,
                   const std::string &)
{
    mtsCISSTToROSTransform(cisstData, rosData);
}

void mtsCISSTToROS(const prmPositionCartesianSet & cisstData, geometry_msgs::TransformStamped & rosData,
                   const std::string &)
{
    mtsCISSTToROSTransform(cisstData.Goal(), rosData.transform);
}

void mtsCISSTToROS(const mtsFrm4x4 & cisstData, geometry_msgs::TransformStamped & rosData,
                   const std::string &)
{
    mtsCISSTToROSTransform(cisstData, rosData.transform);
}

void mtsCISSTToROS(const vctFrm3 & cisstData, geometry_msgs::Transform & rosData,
                   const std::string &)
{
    mtsCISSTToROSTransform(cisstData, rosData);
}

void mtsCISSTToROS(const vctFrm3 & cisstData, geometry_msgs::PoseStamped & rosData,
                   const std::string &)
{
    mtsCISSTToROSPose(cisstData, rosData.pose);
}

void mtsCISSTToROS(const vctFrm3 & cisstData, geometry_msgs::Pose & rosData,
                   const std::string &)
{
    mtsCISSTToROSPose(cisstData, rosData);
}

void mtsCISSTToROS(const vct3 & cisstData, geometry_msgs::Vector3 & rosData,
                   const std::string &)
{
    rosData.x = cisstData[0];
    rosData.y = cisstData[1];
    rosData.z = cisstData[2];
}

void mtsCISSTToROS(const vctMatRot3 & cisstData, geometry_msgs::Quaternion & rosData,
                   const std::string &)
{
    vctQuatRot3 quat(cisstData, VCT_NORMALIZE);
    rosData.x = quat.X();
    rosData.y = quat.Y();
    rosData.z = quat.Z();
    rosData.w = quat.W();
}

void mtsCISSTToROS(const vctMatRot3 & cisstData, geometry_msgs::QuaternionStamped & rosData,
                   const std::string &)
{
    vctQuatRot3 quat(cisstData, VCT_NORMALIZE);
    rosData.quaternion.x = quat.X();
    rosData.quaternion.y = quat.Y();
    rosData.quaternion.z = quat.Z();
    rosData.quaternion.w = quat.W();
}

void mtsCISSTToROS(const vct6 & cisstData, geometry_msgs::Wrench & rosData,
                   const std::string &)
{
    mtsCISSTToROSWrench(cisstData, rosData);
}

void mtsCISSTToROS(const vct6 & cisstData, geometry_msgs::WrenchStamped & rosData,
                   const std::string &)
{
    mtsCISSTToROSWrench(cisstData, rosData.wrench);
}

void mtsCISSTToROS(const mtsDoubleVec & cisstData, geometry_msgs::Wrench & rosData,
                   const std::string & debugInfo)
{
    if (cisstData.size() != 6) {
        CMN_LOG_RUN_ERROR << "mtsCISSTToROS: wrench data size error, should be 6, not "
                          << cisstData.size()
                          << "\" for \"" << debugInfo
                          << "\"" << std::endl;
        return;
    }
    mtsCISSTToROSWrench(cisstData, rosData);
}

void mtsCISSTToROS(const mtsDoubleVec & cisstData, geometry_msgs::WrenchStamped & rosData,
                   const std::string & debugInfo)
{
    if (cisstData.size() != 6) {
        CMN_LOG_RUN_ERROR << "mtsCISSTToROS: wrench data size error, should be 6, not "
                          << cisstData.size()
                          << "\" for \"" << debugInfo
                          << "\"" << std::endl;
        return;
    }
    mtsCISSTToROSWrench(cisstData, rosData.wrench);
}

void mtsCISSTToROS(const mtsDoubleVec & cisstData, geometry_msgs::Vector3Stamped & rosData,
                   const std::string & debugInfo)
{
    if (cisstData.size() != 3) {
        CMN_LOG_RUN_ERROR << "mtsCISSTToROS: vector data size error, should be 3, not "
                          << cisstData.size()
                          << "\" for \"" << debugInfo
                          << "\"" << std::endl;
        return;
    }
    rosData.vector.x = cisstData.Element(0);
    rosData.vector.y = cisstData.Element(1);
    rosData.vector.z = cisstData.Element(2);
}

void mtsCISSTToROS(const prmVelocityCartesianGet & cisstData, geometry_msgs::Twist & rosData,
                   const std::string &)
{
    rosData.linear.x = cisstData.VelocityLinear().X();
    rosData.linear.y = cisstData.VelocityLinear().Y();
    rosData.linear.z = cisstData.VelocityLinear().Z();
    rosData.angular.x = cisstData.VelocityAngular().X();
    rosData.angular.y = cisstData.VelocityAngular().Y();
    rosData.angular.z = cisstData.VelocityAngular().Z();
}

void mtsCISSTToROS(const prmVelocityCartesianGet & cisstData, geometry_msgs::TwistStamped & rosData,
                   const std::string & debugInfo)
{
    mtsCISSTToROS(cisstData, rosData.twist, debugInfo);
}

void mtsCISSTToROS(const prmForceCartesianGet & cisstData, geometry_msgs::Wrench & rosData,
                   const std::string &)
{
    mtsCISSTToROSWrench(cisstData.Force(), rosData);
}

void mtsCISSTToROS(const prmForceCartesianGet & cisstData, geometry_msgs::WrenchStamped & rosData,
                   const std::string & debugInfo)
{
    mtsCISSTToROS(cisstData, rosData.wrench, debugInfo);
}

void mtsCISSTToROS(const prmForceCartesianSet & cisstData, geometry_msgs::Wrench & rosData,
                   const std::string &)
{
    mtsCISSTToROSWrench(cisstData.Force(), rosData);
}

void mtsCISSTToROS(const prmForceCartesianSet & cisstData, geometry_msgs::WrenchStamped & rosData,
                   const std::string & debugInfo)
{
    mtsCISSTToROS(cisstData, rosData.wrench, debugInfo);
}

// ---------------------------------------------
// sensor_msgs
// ---------------------------------------------
void mtsCISSTToROS(const vctDoubleVec & cisstData, sensor_msgs::JointState & rosData,
                   const std::string &)
{
    rosData.velocity.resize(0);
    rosData.effort.resize(0);
    const size_t size = cisstData.size();
    if (size != 0) {
        rosData.position.resize(size);
        std::copy(cisstData.begin(), cisstData.end(),
                  rosData.position.begin());
    }
}

void mtsCISSTToROS(const prmPositionJointGet & cisstData, sensor_msgs::JointState & rosData,
                   const std::string &)
{
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

void mtsCISSTToROS(const prmPositionJointSet & cisstData, sensor_msgs::JointState & rosData,
                   const std::string &)
{
    rosData.name.resize(0);
    rosData.velocity.resize(0);
    rosData.effort.resize(0);
    const size_t size = cisstData.Goal().size();
    if (size != 0) {
        rosData.position.resize(size);
        std::copy(cisstData.Goal().begin(), cisstData.Goal().end(),
                  rosData.position.begin());
    }
}

void mtsCISSTToROS(const prmVelocityJointGet & cisstData, sensor_msgs::JointState & rosData,
                   const std::string &)
{
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

void mtsCISSTToROS(const prmVelocityJointSet & cisstData, sensor_msgs::JointState & rosData,
                   const std::string &)
{
    rosData.name.resize(0);
    rosData.effort.resize(0);
    const size_t size = cisstData.Goal().size();
    if (size != 0) {
        rosData.velocity.resize(size);
        std::copy(cisstData.Goal().begin(), cisstData.Goal().end(),
                  rosData.velocity.begin());
    }
}

void mtsCISSTToROS(const prmForceTorqueJointSet & cisstData, sensor_msgs::JointState & rosData,
                   const std::string &)
{
    rosData.name.resize(0);
    rosData.position.resize(0);
    rosData.velocity.resize(0);
    const size_t size = cisstData.ForceTorque().size();
    if (size != 0) {
        rosData.effort.resize(size);
        std::copy(cisstData.ForceTorque().begin(), cisstData.ForceTorque().end(),
                  rosData.effort.begin());
    }
}

void mtsCISSTToROS(const prmStateJoint & cisstData, sensor_msgs::JointState & rosData,
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

void mtsCISSTToROS(const vctDoubleMat & cisstData, sensor_msgs::PointCloud & rosData,
                   const std::string &)
{
    rosData.points.resize(cisstData.rows());
    for (size_t i = 0; i < cisstData.rows(); ++i) {
        rosData.points[i].x = cisstData.at(i, 0);
        rosData.points[i].y = cisstData.at(i, 1);
        rosData.points[i].z = cisstData.at(i, 2);
    }
}

void mtsCISSTToROS(const std::vector<vct3> & cisstData, sensor_msgs::PointCloud & rosData,
                   const std::string &)
{
    rosData.points.resize(cisstData.size());
    typedef std::vector<vct3>::const_iterator IteratorType;
    const IteratorType end = cisstData.end();
    IteratorType iter;
    size_t index = 0;
    for (iter = cisstData.begin();
         iter != end;
         ++iter, ++index) {
        rosData.points[index].x = iter->X();
        rosData.points[index].y = iter->Y();
        rosData.points[index].z = iter->Z();
    }
}

void mtsCISSTToROS(const prmInputData & cisstData, sensor_msgs::Joy & rosData,
                   const std::string &)
{
    rosData.axes.resize(cisstData.AnalogInputs().size());
    rosData.buttons.resize(cisstData.DigitalInputs().size());
    std::copy(cisstData.AnalogInputs().begin(), cisstData.AnalogInputs().end(),
              rosData.axes.begin());
    std::copy(cisstData.DigitalInputs().begin(), cisstData.DigitalInputs().end(),
              rosData.buttons.begin());
}

// ---------------------------------------------
// diagnostic_msgs
// ---------------------------------------------
void mtsCISSTToROS(const prmKeyValue & cisstData, diagnostic_msgs::KeyValue & rosData,
                   const std::string &)
{
    rosData.key = cisstData.Key;
    rosData.value = cisstData.Value;
}


// ---------------------------------------------
// std_srvs
// ---------------------------------------------
void mtsCISSTToROS(const bool & cisstData, std_srvs::Trigger::Response & rosData,
                   const std::string &)
{
    rosData.success = cisstData;
    rosData.message = "";
}

void mtsCISSTToROS(const std::string & cisstData, std_srvs::Trigger::Response & rosData,
                   const std::string &)
{
    rosData.success = true;
    rosData.message = cisstData;
}

// ---------------------------------------------
// cisst_msgs
// ---------------------------------------------
void mtsCISSTToROS(const prmPositionJointGet & cisstData, cisst_msgs::vctDoubleVec & rosData,
                   const std::string &)
{
    const size_t size = cisstData.Position().size();
    rosData.data.resize(size);
    for (size_t i = 0; i < size; ++i) {
        rosData.data[i] = cisstData.Position().Element(i);
    }
}

void mtsCISSTToROS(const vctDoubleVec & cisstData, cisst_msgs::vctDoubleVec & rosData,
                   const std::string &)
{
    const size_t size = cisstData.size();
    rosData.data.resize(size);
    for (size_t i = 0; i < size; ++i) {
        rosData.data[i] = cisstData.Element(i);
    }
}

void mtsCISSTToROS(const prmCartesianImpedanceGains & cisstData,
                   cisst_msgs::prmCartesianImpedanceGains & rosData,
                   const std::string & debugInfo)
{
    // vf pos/rot
    mtsCISSTToROS(cisstData.ForcePosition(),
                  rosData.ForcePosition, debugInfo);
    mtsCISSTToROS(cisstData.ForceOrientation(),
                  rosData.ForceOrientation, debugInfo);
    mtsCISSTToROS(cisstData.TorqueOrientation(),
                  rosData.TorqueOrientation, debugInfo);

    // force gains
    mtsCISSTToROS(cisstData.PositionDeadbandPos(),
                  rosData.PosDeadbandPos, debugInfo);
    mtsCISSTToROS(cisstData.PositionDeadbandNeg(),
                  rosData.PosDeadbandNeg, debugInfo);
    mtsCISSTToROS(cisstData.PositionStiffnessPos(),
                  rosData.PosStiffPos, debugInfo);
    mtsCISSTToROS(cisstData.PositionStiffnessNeg(),
                  rosData.PosStiffNeg, debugInfo);
    mtsCISSTToROS(cisstData.PositionDampingPos(),
                  rosData.PosDampingPos, debugInfo);
    mtsCISSTToROS(cisstData.PositionDampingNeg(),
                  rosData.PosDampingNeg, debugInfo);
    mtsCISSTToROS(cisstData.ForceBiasPos(),
                  rosData.ForceBiasPos, debugInfo);
    mtsCISSTToROS(cisstData.ForceBiasNeg(),
                  rosData.ForceBiasNeg, debugInfo);

    // torque gains
    mtsCISSTToROS(cisstData.OrientationDeadbandPos(),
                  rosData.OriDeadbandPos, debugInfo);
    mtsCISSTToROS(cisstData.OrientationDeadbandNeg(),
                  rosData.OriDeadbandNeg, debugInfo);
    mtsCISSTToROS(cisstData.OrientationStiffnessPos(),
                  rosData.OriStiffPos, debugInfo);
    mtsCISSTToROS(cisstData.OrientationStiffnessNeg(),
                  rosData.OriStiffNeg, debugInfo);
    mtsCISSTToROS(cisstData.OrientationDampingPos(),
                  rosData.OriDampingPos, debugInfo);
    mtsCISSTToROS(cisstData.OrientationDampingNeg(),
                  rosData.OriDampingNeg, debugInfo);
    mtsCISSTToROS(cisstData.TorqueBiasPos(),
                  rosData.TorqueBiasPos, debugInfo);
    mtsCISSTToROS(cisstData.TorqueBiasNeg(),
                  rosData.TorqueBiasNeg, debugInfo);
}

void mtsCISSTToROS(const mtsIntervalStatistics & cisstData,
                   cisst_msgs::mtsIntervalStatistics & rosData,
                   const std::string &)
{
    rosData.PeriodAvg = cisstData.PeriodAvg();
    rosData.PeriodStdDev = cisstData.PeriodStdDev();
    rosData.PeriodMin = cisstData.PeriodMin();
    rosData.PeriodMax = cisstData.PeriodMax();
    rosData.ComputeTimeAvg = cisstData.ComputeTimeAvg();
    rosData.ComputeTimeStdDev = cisstData.ComputeTimeStdDev();
    rosData.ComputeTimeMin = cisstData.ComputeTimeMin();
    rosData.ComputeTimeMax = cisstData.ComputeTimeMax();
    rosData.NumberOfSamples = cisstData.NumberOfSamples();
    rosData.NumberOfOverruns = cisstData.NumberOfOverruns();
    rosData.StatisticsInterval = cisstData.StatisticsInterval();
}

void mtsCISSTToROS(const vctFrm4x4 & cisstData,
                   cisst_msgs::QueryForwardKinematics::Response & rosData,
                   const std::string &)
{
    mtsCISSTToROSPose(cisstData, rosData.cp.pose);
}
