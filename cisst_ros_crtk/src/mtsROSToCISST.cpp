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

#include <cisst_ros_crtk/mtsROSToCISST.h>
#include <cisst_ros_bridge/mtsROSToCISST.h>

void mtsROSToCISST(const CISST_RAL_MSG(crtk_msgs, OperatingState) & rosData,
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

void mtsROSToCISST(const CISST_RAL_MSG(crtk_msgs, StringStamped) & rosData,
                   std::string & cisstData)
{
    cisstData = rosData.string;
}

void mtsROSToCISST(const CISST_RAL_MSG(crtk_msgs, CartesianImpedanceHalfPlaneGains) & rosData,
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

void mtsROSToCISST(const CISST_RAL_MSG(crtk_msgs, CartesianImpedance) & rosData,
                   prmCartesianImpedance & cisstData)
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

void mtsROSToCISST(const CISST_RAL_MSG(crtk_msgs, CartesianState) & rosData,
                   prmStateCartesian & cisstData)
{
    mtsROSPoseToCISST(rosData.pose, cisstData.Position());
    cisstData.PositionIsValid() = rosData.pose_is_valid.data;

    vct6 velocity(rosData.twist.linear.x,  rosData.twist.linear.y,  rosData.twist.linear.z,
               rosData.twist.angular.x, rosData.twist.angular.y, rosData.twist.angular.z);
    cisstData.SetVelocity(velocity);
    cisstData.VelocityIsValid() = rosData.twist_is_valid.data;

    vct6 force(rosData.wrench.force.x,  rosData.wrench.force.y,  rosData.wrench.force.z,
                rosData.wrench.torque.x, rosData.wrench.torque.y, rosData.wrench.torque.z);
    cisstData.SetForce(force);
    cisstData.ForceIsValid() = rosData.wrench_is_valid.data;
}

void mtsROSToCISST(const CISST_RAL_SRV_REQ(crtk_msgs, QueryForwardKinematics) & rosData,
                   prmForwardKinematicsRequest & cisstData)
{
    cisstData.jp().SetSize(rosData.jp.size());
    std::copy(rosData.jp.begin(), rosData.jp.end(),
              cisstData.jp().begin());
}

void mtsROSToCISST(const CISST_RAL_SRV_REQ(crtk_msgs, QueryInverseKinematics) & rosData,
                   prmInverseKinematicsRequest & cisstData)
{
    cisstData.jp().SetSize(rosData.jp.size());
    std::copy(rosData.jp.begin(), rosData.jp.end(),
              cisstData.jp().begin());
    mtsROSPoseToCISST(rosData.cp, cisstData.cp());
}
