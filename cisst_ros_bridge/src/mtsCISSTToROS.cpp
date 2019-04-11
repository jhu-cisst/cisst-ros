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

#include "cisst_ros_bridge/mtsCISSTToROS.h"

bool mtsCISSTToROS(const double & cisstData, std_msgs::Float32 & rosData,
                   const std::string &)
{
    rosData.data = cisstData;
    return true;
}

bool mtsCISSTToROS(const double & cisstData, std_msgs::Float64 & rosData,
                   const std::string &)
{
    rosData.data = cisstData;
    return true;
}

bool mtsCISSTToROS(const int & cisstData, std_msgs::Int32 & rosData,
                   const std::string &)
{
    rosData.data = cisstData;
    return true;
}

bool mtsCISSTToROS(const bool & cisstData, std_msgs::Bool & rosData,
                   const std::string &)
{
    rosData.data = cisstData;
    return true;
}

bool mtsCISSTToROS(const bool & cisstData, cisst_msgs::BoolStamped & rosData,
                   const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(rosData, debugInfo)) {
        rosData.data = cisstData;
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const bool & cisstData, sensor_msgs::Joy & rosData,
                   const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(rosData, debugInfo)) {
        rosData.axes.resize(0);
        rosData.buttons.resize(1);
        rosData.buttons[0] = cisstData;
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const std::string & cisstData, std_msgs::String & rosData,
                   const std::string &)
{
    rosData.data = cisstData;
    return true;
}

bool mtsCISSTToROS(const mtsMessage & cisstData, std_msgs::String & rosData,
                   const std::string &)
{
    rosData.data = cisstData.Message;
    return true;
}

bool mtsCISSTToROS(const prmEventButton & cisstData, std_msgs::Bool & rosData,
                   const std::string &)
{
    if (cisstData.Valid()) {
        if (cisstData.Type() == prmEventButton::PRESSED) {
            rosData.data = true;
        } else if (cisstData.Type() == prmEventButton::RELEASED) {
            rosData.data = false;
        }
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const prmEventButton & cisstData, cisst_msgs::BoolStamped & rosData,
                   const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(cisstData, rosData, debugInfo)) {
        if (cisstData.Type() == prmEventButton::PRESSED) {
            rosData.data = true;
        } else if (cisstData.Type() == prmEventButton::RELEASED) {
            rosData.data = false;
        }
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const prmEventButton & cisstData, sensor_msgs::Joy & rosData,
                   const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(cisstData, rosData, debugInfo)) {
        rosData.axes.resize(0);
        rosData.buttons.resize(1);
        if (cisstData.Type() == prmEventButton::PRESSED) {
            rosData.buttons[0] = 1;
        } else if (cisstData.Type() == prmEventButton::RELEASED) {
            rosData.buttons[0] = 0;
        }
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const vctDoubleMat & cisstData, std_msgs::Float64MultiArray & rosData,
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
    return true;
}

bool mtsCISSTToROS(const vctDoubleVec & cisstData, std_msgs::Float64MultiArray & rosData,
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
    return true;
}

bool mtsCISSTToROS(const prmPositionCartesianGet & cisstData, geometry_msgs::Transform & rosData,
                   const std::string &)
{
    mtsCISSTToROSTransform(cisstData.Position(), rosData);
    return true;
}

bool mtsCISSTToROS(const prmPositionCartesianGet & cisstData, geometry_msgs::TransformStamped & rosData,
                   const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(cisstData, rosData, debugInfo)) {
        rosData.header.frame_id = cisstData.ReferenceFrame();
        rosData.child_frame_id = cisstData.MovingFrame();
        mtsCISSTToROSTransform(cisstData.Position(), rosData.transform);
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const prmPositionCartesianGet & cisstData, geometry_msgs::Pose & rosData,
                   const std::string &)
{
    mtsCISSTToROSPose(cisstData.Position(), rosData);
    return true;
}

bool mtsCISSTToROS(const prmPositionCartesianGet & cisstData, geometry_msgs::PoseStamped & rosData,
                   const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(cisstData, rosData, debugInfo)) {
        rosData.header.frame_id = cisstData.MovingFrame();
        mtsCISSTToROSPose(cisstData.Position(), rosData.pose);
        return true;
    }
    return false;
}


bool mtsCISSTToROS(const vctFrm4x4 & cisstData, geometry_msgs::Pose & rosData,
                   const std::string &)
{
    mtsCISSTToROSPose(cisstData, rosData);
    return true;
}

bool mtsCISSTToROS(const mtsFrm4x4 & cisstData, geometry_msgs::Pose & rosData,
                   const std::string &)
{
    mtsCISSTToROSPose(cisstData, rosData);
    return true;
}

bool mtsCISSTToROS(const vctFrm3 & cisstData, geometry_msgs::Pose & rosData,
                   const std::string &)
{
    mtsCISSTToROSPose(cisstData, rosData);
    return true;
}

bool mtsCISSTToROS(const vctFrm4x4 & cisstData, geometry_msgs::Transform & rosData,
                   const std::string &)
{
    mtsCISSTToROSTransform(cisstData, rosData);
    return true;
}

bool mtsCISSTToROS(const mtsFrm4x4 & cisstData, geometry_msgs::Transform & rosData,
                   const std::string &)
{
    mtsCISSTToROSTransform(cisstData, rosData);
    return true;
}

bool mtsCISSTToROS(const mtsFrm4x4 & cisstData, geometry_msgs::TransformStamped & rosData,
                   const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(cisstData, rosData, debugInfo)) {
        mtsCISSTToROSTransform(cisstData, rosData.transform);
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const vctFrm3 & cisstData, geometry_msgs::Transform & rosData,
                   const std::string &)
{
    mtsCISSTToROSTransform(cisstData, rosData);
    return true;
}

bool mtsCISSTToROS(const vct3 & cisstData, geometry_msgs::Vector3 & rosData,
                   const std::string &)
{
    rosData.x = cisstData[0];
    rosData.y = cisstData[1];
    rosData.z = cisstData[2];
    return true;
}

bool mtsCISSTToROS(const vctMatRot3 & cisstData, geometry_msgs::Quaternion & rosData,
                   const std::string &)
{
    vctQuatRot3 quat(cisstData, VCT_NORMALIZE);
    rosData.x = quat.X();
    rosData.y = quat.Y();
    rosData.z = quat.Z();
    rosData.w = quat.W();
    return true;
}

bool mtsCISSTToROS(const mtsDoubleVec & cisstData, geometry_msgs::Wrench & rosData,
                   const std::string & debugInfo)
{
    if (cisstData.size() != 6) {
        CMN_LOG_RUN_ERROR << "mtsCISSTToROS: wrench data size error, should be 6, not "
                          << cisstData.size()
                          << "\" for \"" << debugInfo
                          << "\"" << std::endl;
        return false;
    }
    rosData.force.x = cisstData.Element(0);
    rosData.force.y = cisstData.Element(1);
    rosData.force.z = cisstData.Element(2);
    rosData.torque.x = cisstData.Element(3);
    rosData.torque.y = cisstData.Element(4);
    rosData.torque.z = cisstData.Element(5);
    return true;
}

bool mtsCISSTToROS(const mtsDoubleVec & cisstData, geometry_msgs::WrenchStamped & rosData,
                   const std::string & debugInfo)
{
    if (cisstData.size() != 6) {
        CMN_LOG_RUN_ERROR << "mtsCISSTToROS: wrench data size error, should be 6, not "
                          << cisstData.size()
                          << "\" for \"" << debugInfo
                          << "\"" << std::endl;
        return false;
    }
    mtsCISSTToROSHeader(cisstData, rosData, debugInfo);
    rosData.wrench.force.x = cisstData.Element(0);
    rosData.wrench.force.y = cisstData.Element(1);
    rosData.wrench.force.z = cisstData.Element(2);
    rosData.wrench.torque.x = cisstData.Element(3);
    rosData.wrench.torque.y = cisstData.Element(4);
    rosData.wrench.torque.z = cisstData.Element(5);
    return true;
}

bool mtsCISSTToROS(const mtsDoubleVec & cisstData, geometry_msgs::Vector3Stamped & rosData,
                   const std::string & debugInfo)
{
    if (cisstData.size() != 3) {
        CMN_LOG_RUN_ERROR << "mtsCISSTToROS: vector data size error, should be 3, not "
                          << cisstData.size()
                          << "\" for \"" << debugInfo
                          << "\"" << std::endl;
        return false;
    }
    if (mtsCISSTToROSHeader(cisstData, rosData, debugInfo)) {
        rosData.vector.x = cisstData.Element(0);
        rosData.vector.y = cisstData.Element(1);
        rosData.vector.z = cisstData.Element(2);
    }
    return true;
}

bool mtsCISSTToROS(const prmVelocityCartesianGet & cisstData, geometry_msgs::Twist & rosData,
                   const std::string &)
{
    if (cisstData.Valid()) {
        rosData.linear.x = cisstData.VelocityLinear().X();
        rosData.linear.y = cisstData.VelocityLinear().Y();
        rosData.linear.z = cisstData.VelocityLinear().Z();
        rosData.angular.x = cisstData.VelocityAngular().X();
        rosData.angular.y = cisstData.VelocityAngular().Y();
        rosData.angular.z = cisstData.VelocityAngular().Z();
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const prmVelocityCartesianGet & cisstData, geometry_msgs::TwistStamped & rosData,
                   const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(cisstData, rosData, debugInfo)) {
        rosData.header.frame_id = cisstData.MovingFrame();
        mtsCISSTToROS(cisstData, rosData.twist, debugInfo);
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const prmForceCartesianGet & cisstData, geometry_msgs::Wrench & rosData,
                   const std::string &)
{
    if (cisstData.Valid()) {
        rosData.force.x = cisstData.Force().Element(0);
        rosData.force.y = cisstData.Force().Element(1);
        rosData.force.z = cisstData.Force().Element(2);
        rosData.torque.x = cisstData.Force().Element(3);
        rosData.torque.y = cisstData.Force().Element(4);
        rosData.torque.z = cisstData.Force().Element(5);
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const prmForceCartesianGet & cisstData, geometry_msgs::WrenchStamped & rosData,
                   const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(cisstData, rosData, debugInfo)) {
        rosData.header.frame_id = cisstData.MovingFrame();
        mtsCISSTToROS(cisstData, rosData.wrench, debugInfo);
        return true;
    }
    return false;
}

// ---------------------------------------------
// sensor_msgs
// ---------------------------------------------
bool mtsCISSTToROS(const vctDoubleVec & cisstData, sensor_msgs::JointState & rosData, const std::string & debugInfo)
{
    mtsCISSTToROSHeader(rosData, debugInfo);
    rosData.velocity.resize(0);
    rosData.effort.resize(0);
    const size_t size = cisstData.size();
    if (size != 0) {
        rosData.position.resize(size);
        std::copy(cisstData.begin(), cisstData.end(),
                  rosData.position.begin());
    }
    return true;
}

bool mtsCISSTToROS(const prmPositionJointGet & cisstData, sensor_msgs::JointState & rosData,
                   const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(cisstData, rosData, debugInfo)) {
        rosData.name.resize(0);
        rosData.velocity.resize(0);
        rosData.effort.resize(0);
        const size_t size = cisstData.Position().size();
        if (size != 0) {
            rosData.position.resize(size);
            std::copy(cisstData.Position().begin(), cisstData.Position().end(),
                      rosData.position.begin());
        }
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const prmVelocityJointGet & cisstData, sensor_msgs::JointState & rosData,
                   const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(cisstData, rosData, debugInfo)) {
        rosData.name.resize(0);
        rosData.position.resize(0);
        rosData.effort.resize(0);
        const size_t size = cisstData.Velocity().size();
        if (size != 0) {
            rosData.velocity.resize(size);
            std::copy(cisstData.Velocity().begin(), cisstData.Velocity().end(),
                      rosData.velocity.begin());
        }
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const prmStateJoint & cisstData, sensor_msgs::JointState & rosData,
                   const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(cisstData, rosData, debugInfo)) {
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
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const vctDoubleMat & cisstData, sensor_msgs::PointCloud & rosData,
                   const std::string & debugInfo)
{
    mtsCISSTToROSHeader(rosData, debugInfo);
    rosData.points.resize(cisstData.rows());
    for (size_t i = 0; i < cisstData.rows(); ++i) {
        rosData.points[i].x = cisstData.at(i, 0);
        rosData.points[i].y = cisstData.at(i, 1);
        rosData.points[i].z = cisstData.at(i, 2);
    }
    return true;
}

bool mtsCISSTToROS(const std::vector<vct3> & cisstData, sensor_msgs::PointCloud & rosData,
                   const std::string & debugInfo)
{
    mtsCISSTToROSHeader(rosData, debugInfo);
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
    return true;
}

bool mtsCISSTToROS(const prmInputData & cisstData, sensor_msgs::Joy & rosData,
                   const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(cisstData, rosData, debugInfo)) {
        rosData.axes.resize(cisstData.AnalogInputs().size());
        rosData.buttons.resize(cisstData.DigitalInputs().size());
        std::copy(cisstData.AnalogInputs().begin(), cisstData.AnalogInputs().end(),
                  rosData.axes.begin());
        std::copy(cisstData.DigitalInputs().begin(), cisstData.DigitalInputs().end(),
                  rosData.buttons.begin());
        return true;
    }
    return false;
}

// ---------------------------------------------
// diagnostic_msgs
// ---------------------------------------------
bool mtsCISSTToROS(const prmKeyValue & cisstData, diagnostic_msgs::KeyValue & rosData,
                   const std::string &)
{
    rosData.key = cisstData.Key;
    rosData.value = cisstData.Value;
    return true;
}


// ---------------------------------------------
// std_srvs
// ---------------------------------------------
bool mtsCISSTToROS(const bool & cisstData, std_srvs::Trigger::Response & rosData,
                   const std::string &)
{
    rosData.success = cisstData;
    rosData.message = "";
    return true;
}

bool mtsCISSTToROS(const std::string & cisstData, std_srvs::Trigger::Response & rosData,
                   const std::string &)
{
    rosData.success = true;
    rosData.message = cisstData;
    return true;
}

// ---------------------------------------------
// cisst_msgs
// ---------------------------------------------
bool mtsCISSTToROS(const prmPositionJointGet & cisstData, cisst_msgs::vctDoubleVec & rosData,
                   const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(cisstData, rosData, debugInfo)) {
        const size_t size = cisstData.Position().size();
        rosData.data.resize(size);
        for (size_t i = 0; i < size; ++i) {
            rosData.data[i] = cisstData.Position().Element(i);
        }
        return true;
    }
    return false;
}

bool mtsCISSTToROS(const vctDoubleVec & cisstData, cisst_msgs::vctDoubleVec & rosData,
                   const std::string & debugInfo)
{
    mtsCISSTToROSHeader(rosData, debugInfo);
    const size_t size = cisstData.size();
    rosData.data.resize(size);
    for (size_t i = 0; i < size; ++i) {
        rosData.data[i] = cisstData.Element(i);
    }
    return true;
}

bool mtsCISSTToROS(const prmCartesianImpedanceGains & cisstData,
                   cisst_msgs::prmCartesianImpedanceGains & rosData,
                   const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(cisstData, rosData, debugInfo)) {
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

        return true;
    }
    return false;
}

bool mtsCISSTToROS(const mtsIntervalStatistics & cisstData,
                   cisst_msgs::mtsIntervalStatistics & rosData,
                   const std::string & debugInfo)
{
    if (mtsCISSTToROSHeader(cisstData, rosData, debugInfo)) {
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
        return true;
    }
    return false;
}
