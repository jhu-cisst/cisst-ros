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

#include <cisst_ros_crtk/mtsROSToCISST.h>
#include <cisst_ros_bridge/mtsROSToCISST.h>

void mtsROSToCISST(const crtk_msgs::OperatingState & rosData,
                   prmOperatingState & cisstData)
{
    try {
        cisstData.State() = prmOperatingState::StateTypeFromString(rosData.state);
    } catch (...) {
        cisstData.State() = prmOperatingState::UNDEFINED;
    }
    cisstData.IsHomed() = rosData.is_homed;
    cisstData.IsBusy() = rosData.is_busy;
}

void mtsROSToCISST(const crtk_msgs::StringStamped & rosData,
                   std::string & cisstData)
{
    cisstData = rosData.string;
}

void mtsROSToCISST(const crtk_msgs::CartesianImpedanceHalfPlaneGains & rosData,
                   prmCartesianImpedanceHalfPlaneGains & cisstData)
{
    mtsROSToCISST(rosData.deadband,
                  cisstData.Deadband);
    mtsROSToCISST(rosData.p,
                  cisstData.P);
    mtsROSToCISST(rosData.d,
                  cisstData.D);
    mtsROSToCISST(rosData.bias,
                  cisstData.Bias);
}

void mtsROSToCISST(const crtk_msgs::CartesianImpedance & rosData, prmCartesianImpedance & cisstData)
{
    // vf pos/rot
    mtsROSToCISST(rosData.force_position,
                  cisstData.ForcePosition);
    mtsROSToCISST(rosData.force_orientation,
                  cisstData.ForceOrientation);
    mtsROSToCISST(rosData.torque_orientation,
                  cisstData.TorqueOrientation);

    // force gains
    mtsROSToCISST(rosData.position_positive,
                  cisstData.PositionPositive);
    mtsROSToCISST(rosData.position_negative,
                  cisstData.PositionNegative);

    // torque gains
    mtsROSToCISST(rosData.orientation_positive,
                  cisstData.OrientationPositive);
    mtsROSToCISST(rosData.orientation_negative,
                  cisstData.OrientationNegative);
}

void mtsROSToCISST(const crtk_msgs::QueryForwardKinematics::Request & rosData,
                   prmForwardKinematicsRequest & cisstData)
{
    cisstData.jp().SetSize(rosData.jp.size());
    std::copy(rosData.jp.begin(), rosData.jp.end(),
              cisstData.jp().begin());
}

void mtsROSToCISST(const crtk_msgs::QueryInverseKinematics::Request & rosData,
                   prmInverseKinematicsRequest & cisstData)
{
    cisstData.jp().SetSize(rosData.jp.size());
    std::copy(rosData.jp.begin(), rosData.jp.end(),
              cisstData.jp().begin());
    mtsROSPoseToCISST(rosData.cp, cisstData.cp());
}
