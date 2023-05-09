/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2020-03-24

  (C) Copyright 2020-2023 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsCISSTToROS_CRTK_h
#define _mtsCISSTToROS_CRTK_h

// cisst include
#include <cisstParameterTypes/prmOperatingState.h>
#include <cisstParameterTypes/prmForwardKinematicsResponse.h>
#include <cisstParameterTypes/prmInverseKinematicsResponse.h>

// crtk
#include <crtk_msgs/OperatingState.h>
#include <crtk_msgs/StringStamped.h>
#include <crtk_msgs/TriggerOperatingState.h>
#include <crtk_msgs/QueryForwardKinematics.h>
#include <crtk_msgs/QueryInverseKinematics.h>

// crtk_msgs
void mtsCISSTToROS(const prmOperatingState & cisstData,
                   crtk_msgs::OperatingState & rosData,
                   const std::string & debugInfo);

void mtsCISSTToROS(const std::string & cisstData,
                   crtk_msgs::StringStamped & rosData,
                   const std::string & debugInfo);

// crtk_srvs
void mtsCISSTToROS(const prmOperatingState & cisstData,
                   crtk_msgs::TriggerOperatingState::Response & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmForwardKinematicsResponse & cisstData,
                   crtk_msgs::QueryForwardKinematics::Response & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmInverseKinematicsResponse & cisstData,
                   crtk_msgs::QueryInverseKinematics::Response & rosData,
                   const std::string & debugInfo);

#endif // _mtsCISSTToROS_CRTK_h
