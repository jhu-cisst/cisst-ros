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

#ifndef _mtsROSToCISST_h
#define _mtsROSToCISST_h

// cisst include
#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstMultiTask/mtsTransformationTypes.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmForceTorqueJointSet.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>
#include <cisstParameterTypes/prmFixtureGainCartesianSet.h>

// ros include
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>

#include <sensor_msgs/JointState.h>

// non standard messages
#include <cisst_msgs/vctDoubleVec.h>
#include <cisst_msgs/prmFixtureGainCartesianSet.h>

// std_msgs
void mtsROSToCISST(const std_msgs::Float32 & rosData, double & cisstData);
void mtsROSToCISST(const std_msgs::Int32 & rosData, int & cisstData);
void mtsROSToCISST(const std_msgs::Bool & rosData, bool & cisstData);
void mtsROSToCISST(const std_msgs::String & rosData, std::string & cisstData);

// geometry_msgs
void mtsROSToCISST(const geometry_msgs::Vector3 & rosData, vct3 & cisstData);
void mtsROSToCISST(const geometry_msgs::Quaternion & rosData, vctMatRot3 & cisstData);
void mtsROSToCISST(const geometry_msgs::Pose & rosData, prmPositionCartesianGet & cisstData);
void mtsROSToCISST(const geometry_msgs::Pose & rosData, prmPositionCartesianSet & cisstData);
void mtsROSToCISST(const geometry_msgs::PoseStamped & rosData, prmPositionCartesianSet & cisstData);
void mtsROSToCISST(const geometry_msgs::Pose & rosData, vctFrm3 & cisstData);
void mtsROSToCISST(const geometry_msgs::Pose & rosData, vctFrm4x4 & cisstData);
void mtsROSToCISST(const geometry_msgs::Pose & rosData, mtsFrm4x4 & cisstData);
void mtsROSToCISST(const geometry_msgs::Transform & rosData, prmPositionCartesianGet & cisstData);
void mtsROSToCISST(const geometry_msgs::Transform & rosData, vctFrm3 & cisstData);
void mtsROSToCISST(const geometry_msgs::Transform & rosData, vctFrm4x4 & cisstData);
void mtsROSToCISST(const geometry_msgs::Transform & rosData, mtsFrm4x4 & cisstData);
void mtsROSToCISST(const geometry_msgs::Wrench & rosData, prmForceCartesianSet & cisstData);
void mtsROSToCISST(const geometry_msgs::WrenchStamped & rosData, prmForceCartesianSet & cisstData);

// sensor_msgs
void mtsROSToCISST(const sensor_msgs::JointState & rosData, prmPositionJointSet & cisstData);
void mtsROSToCISST(const sensor_msgs::JointState & rosData, prmForceTorqueJointSet & cisstData);

// cisst_msgs
void mtsROSToCISST(const cisst_msgs::vctDoubleVec & rosData, prmPositionJointSet & cisstData);
void mtsROSToCISST(const cisst_msgs::vctDoubleVec & rosData, vctDoubleVec & cisstData);
void mtsROSToCISST(const cisst_msgs::prmFixtureGainCartesianSet & rosData, prmFixtureGainCartesianSet & cisstData);

#endif // _mtsROSToCISST_h
