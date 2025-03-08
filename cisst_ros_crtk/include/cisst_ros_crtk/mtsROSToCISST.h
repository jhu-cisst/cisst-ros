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

#ifndef _mtsROSToCISST_CRTK_h
#define _mtsROSToCISST_CRTK_h

// cisst include
#include <cisstParameterTypes/prmOperatingState.h>
#include <cisstParameterTypes/prmCartesianImpedance.h>
#include <cisstParameterTypes/prmStateCartesian.h>
#include <cisstParameterTypes/prmServoCartesian.h>
#include <cisstParameterTypes/prmServoJoint.h>
#include <cisstParameterTypes/prmForwardKinematicsRequest.h>
#include <cisstParameterTypes/prmInverseKinematicsRequest.h>

// ral
#include <cisst_ros_bridge/cisst_ral.h>

// crtk
#if ROS1
#include <crtk_msgs/OperatingState.h>
#include <crtk_msgs/StringStamped.h>
#include <crtk_msgs/CartesianImpedance.h>
#include <crtk_msgs/CartesianState.h>
#include <crtk_msgs/CartesianServo.h>
#include <crtk_msgs/JointServo.h>
#include <crtk_msgs/QueryForwardKinematics.h>
#include <crtk_msgs/QueryInverseKinematics.h>
#elif ROS2
#include <crtk_msgs/msg/operating_state.hpp>
#include <crtk_msgs/msg/string_stamped.hpp>
#include <crtk_msgs/msg/cartesian_impedance.hpp>
#include <crtk_msgs/msg/cartesian_state.hpp>
#include <crtk_msgs/msg/cartesian_servo.hpp>
#include <crtk_msgs/msg/joint_servo.hpp>
#include <crtk_msgs/srv/query_forward_kinematics.hpp>
#include <crtk_msgs/srv/query_inverse_kinematics.hpp>
#endif

// crtk_msgs
void mtsROSToCISST(const CISST_RAL_MSG(crtk_msgs, OperatingState) & rosData,
                   prmOperatingState & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(crtk_msgs, StringStamped) & rosData,
                   std::string & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(crtk_msgs, CartesianImpedance) & rosData,
                   prmCartesianImpedance & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(crtk_msgs, CartesianState) & rosData,
                   prmStateCartesian & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(crtk_msgs, CartesianServo) & rosData,
                   prmServoCartesian & cisstData);
void mtsROSToCISST(const CISST_RAL_MSG(crtk_msgs, JointServo) & rosData,
                   prmStateCartesian & cisstData);

// crtk_srvs
void mtsROSToCISST(const CISST_RAL_SRV_REQ(crtk_msgs, QueryForwardKinematics) & rosData,
                   prmForwardKinematicsRequest & cisstData);
void mtsROSToCISST(const CISST_RAL_SRV_REQ(crtk_msgs, QueryInverseKinematics) & rosData,
                   prmInverseKinematicsRequest & cisstData);

#endif // _mtsROSToCISST_CRTK_h
