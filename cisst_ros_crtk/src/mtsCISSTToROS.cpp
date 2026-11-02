/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2020-03-24

  (C) Copyright 2020-2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisst_ros_crtk/mtsCISSTToROS.h>
#include <cisst_ros_bridge/mtsCISSTToROS.h>

template <typename _ros_operating_state>
void mtsCISSTToROSOperatingState(const prmOperatingState & cisstData,
                                 _ros_operating_state & rosData,
                                 const std::string &)
{
    try {
        rosData.state = prmOperatingState::StateTypeToString(cisstData.State());
    } catch (...) {
        rosData.state = "UNDEFINED";
    }
    rosData.is_homed = cisstData.IsHomed();
    rosData.is_busy = cisstData.IsBusy();
}

void mtsCISSTToROS(const prmOperatingState & cisstData,
                   CISST_RAL_MSG(crtk_msgs, OperatingState) & rosData,
                   const std::string & debugInfo)
{
    mtsCISSTToROSOperatingState(cisstData, rosData, debugInfo);
}

void mtsCISSTToROS(const std::string & cisstData,
                   CISST_RAL_MSG(crtk_msgs, StringStamped) & rosData,
                   const std::string &)
{
    rosData.string = cisstData;
}

void mtsCISSTToROS(const prmStateCartesian & cisstData,
                   CISST_RAL_MSG(crtk_msgs, CartesianState) & rosData,
                   const std::string & debugInfo)
{
    mtsCISSTToROS(cisstData.Position(), rosData.pose, debugInfo);
    rosData.pose_is_valid.data = cisstData.PositionIsValid();

    mtsCISSTToROS(cisstData.Velocity(), rosData.twist, debugInfo);
    rosData.twist_is_valid.data = cisstData.VelocityIsValid();

    mtsCISSTToROS(cisstData.Force(), rosData.wrench, debugInfo);
    rosData.wrench_is_valid.data = cisstData.ForceIsValid();
}

void mtsCISSTToROS(const prmServoCartesian & cisstData,
    CISST_RAL_MSG(crtk_msgs, CartesianServo) & rosData,
    const std::string & debugInfo)
{
    mtsCISSTToROS(cisstData.TaskFrame(), rosData.task_frame, debugInfo);

    mtsCISSTToROS(cisstData.Position(), rosData.pose, debugInfo);
    mtsCISSTToROS(cisstData.Velocity(), rosData.twist, debugInfo);
    mtsCISSTToROS(cisstData.Force(), rosData.wrench, debugInfo);

    for (size_t i = 0; i < 6; i++) {
        rosData.axis_mode[i] = CISST_RAL_MSG(crtk_msgs, SetpointMode)();
        rosData.axis_mode[i].value = static_cast<uint8_t>(cisstData.AxisMode()[i]);
    }
}

void mtsCISSTToROS(const prmServoJoint & cisstData,
    CISST_RAL_MSG(crtk_msgs, JointServo) & rosData,
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
    { // modes
        const size_t size = cisstData.Mode().size();
        if (size != 0) {
            rosData.mode.resize(size);
            for (size_t i = 0; i < size; i++) {
                rosData.mode[i] = CISST_RAL_MSG(crtk_msgs, SetpointMode)();
                rosData.mode[i].value = static_cast<uint8_t>(cisstData.Mode()[i]);
            }
        }
    }
}

void mtsCISSTToROS(const prmOperatingState & cisstData,
                   CISST_RAL_SRV_RES(crtk_msgs, TriggerOperatingState) & rosData,
                   const std::string & debugInfo)
{
    mtsCISSTToROSOperatingState(cisstData, rosData.operating_state, debugInfo);
}

void mtsCISSTToROS(const prmForwardKinematicsResponse & cisstData,
                   CISST_RAL_SRV_RES(crtk_msgs, QueryForwardKinematics) & rosData,
                   const std::string & debugInfo)
{
    mtsCISSTToROS(cisstData.cp(), rosData.cp, debugInfo);
    rosData.result = cisstData.result();
    rosData.message = cisstData.message();
}

void mtsCISSTToROS(const prmInverseKinematicsResponse & cisstData,
                   CISST_RAL_SRV_RES(crtk_msgs, QueryInverseKinematics) & rosData,
                   const std::string &)
{
    rosData.jp.resize(cisstData.jp().size());
    std::copy(cisstData.jp().begin(), cisstData.jp().end(),
              rosData.jp.begin());
    rosData.result = cisstData.result();
    rosData.message = cisstData.message();
}
