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

#ifndef _mtsCISSTToROS_h
#define _mtsCISSTToROS_h

// cisst include
#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <cisstParameterTypes/prmFixtureGainCartesianSet.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>
#include <cisstMultiTask/mtsVector.h>

// ros include
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud.h>

// non standard messages
#include <cisst_msgs/vctDoubleVec.h>
#include <cisst_msgs/prmFixtureGainCartesianSet.h>


// std_msgs
void mtsCISSTToROS(const double & cisstData, std_msgs::Float32 & rosData);
void mtsCISSTToROS(const bool & cisstData, std_msgs::Bool & rosData);
void mtsCISSTToROS(const std::string & cisstData, std_msgs::String & rosData);
void mtsCISSTToROS(const prmEventButton & cisstData, std_msgs::Bool & rosData);

// geometry_msgs
void mtsCISSTToROS(const prmPositionCartesianGet & cisstData, geometry_msgs::Transform & rosData);
void mtsCISSTToROS(const prmPositionCartesianGet & cisstData, geometry_msgs::Pose & rosData);
void mtsCISSTToROS(const vctFrm4x4 & cisstData, geometry_msgs::Pose & rosData);
void mtsCISSTToROS(const vctFrm3 & cisstData, geometry_msgs::Pose & rosData);
void mtsCISSTToROS(const vct3 & cisstData, geometry_msgs::Vector3 & rosData);
void mtsCISSTToROS(const vctMatRot3 & cisstData, geometry_msgs::Quaternion & rosData);
void mtsCISSTToROS(const mtsDoubleVec & cisstData, geometry_msgs::Wrench & rosData);
void mtsCISSTToROS(const mtsDoubleVec & cisstData, geometry_msgs::WrenchStamped & rosData);
void mtsCISSTToROS(const mtsDoubleVec & cisstData, geometry_msgs::Vector3Stamped & rosData);
void mtsCISSTToROS(const prmVelocityCartesianGet &cisstData, geometry_msgs::Twist &rosData);

// sensor_msgs
void mtsCISSTToROS(const prmPositionJointGet & cisstData, sensor_msgs::JointState & rosData);
void mtsCISSTToROS(const vctDoubleMat & cisstData, sensor_msgs::PointCloud & rosData);

// cisst_msgs
void mtsCISSTToROS(const prmPositionJointGet & cisstData, cisst_msgs::vctDoubleVec & rosData);
void mtsCISSTToROS(const vctDoubleVec & cisstData, cisst_msgs::vctDoubleVec & rosData);
void mtsCISSTToROS(const prmFixtureGainCartesianSet & cisstData, cisst_msgs::prmFixtureGainCartesianSet & rosData);


#endif // _mtsCISSTToROS_h
