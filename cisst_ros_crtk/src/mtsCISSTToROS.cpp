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

void mtsCISSTToROS(const prmOperatingState & cisstData,
                   CISST_RAL_SRV_RES(crtk_msgs, TriggerOperatingState) & rosData,
                   const std::string & debugInfo)
{
    mtsCISSTToROSOperatingState(cisstData, rosData.operating_state, debugInfo);
}

void mtsCISSTToROS(const prmForwardKinematicsResponse & cisstData,
                   CISST_RAL_SRV_RES(crtk_msgs, QueryForwardKinematics) & rosData,
                   const std::string &)
{
    mtsCISSTToROSPose(cisstData.cp(), rosData.cp);
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
