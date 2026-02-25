/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen, Adnan Munawar
  Created on: 2013-05-21

  (C) Copyright 2013-2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

#include "cisstParameterTypes/prmPositionCartesianGet.h"
#include <cisst_ros_bridge/mtsROSToCISST.h>
#include <Eigen/Geometry>


void mtsROSToCISST(const CISST_RAL_MSG(std_msgs, Float32) & rosData,
                   double & cisstData)
{
    cisstData = rosData.data;
}

void mtsROSToCISST(const CISST_RAL_MSG(std_msgs, Float64) & rosData,
                   double & cisstData)
{
    cisstData = rosData.data;
}

void mtsROSToCISST(const CISST_RAL_MSG(std_msgs, Int32) & rosData,
                   int & cisstData)
{
    cisstData = rosData.data;
}

void mtsROSToCISST(const CISST_RAL_MSG(std_msgs, Bool) & rosData,
                   bool & cisstData)
{
    cisstData = rosData.data;
}

void mtsROSToCISST(const CISST_RAL_MSG(std_msgs, String) & rosData,
                   std::string & cisstData)
{
    cisstData = rosData.data;
}

void mtsROSToCISST(const CISST_RAL_MSG(std_msgs, String) & rosData,
                   mtsMessage & cisstData)
{
    cisstData.Message = rosData.data;
}

void mtsROSToCISST(const CISST_RAL_MSG(std_msgs, Float64MultiArray) & rosData,
                   Eigen::VectorXd & cisstData)
{
    const size_t size = rosData.data.size();
    if (size != 0) {
        cisstData.resize(size);
        std::copy(rosData.data.begin(), rosData.data.end(),
                  cisstData.begin());
    }
}

void mtsROSToCISST(const CISST_RAL_MSG(std_msgs, Float64MultiArray) & rosData,
                   Eigen::MatrixXd & cisstData)
{
    if (rosData.layout.dim.size() != 2) {
        cmnThrow("mtsROSToCISST(std::msgs::Float64MultiArray, vctDoubleMat): incoming array dimension is not 2");
    }
    // assuming rows/cols for data storage.  In mtsCISSTToROS we
    // labelled the dimensions using "rows" and "cols" so we're making
    // sure names are the same.  This might be annoying for data
    // coming from other packages and we might have to change this
    // check.  We could also add support for cols/rows storage order
    if ((rosData.layout.dim[0].label != "rows")
        || (rosData.layout.dim[1].label != "cols")) {
        cmnThrow("mtsROSToCISST(std::msgs::Float64MultiArray, vctDoubleMat): dimensions must be labelled \"rows\" and \"cols\"");
    }
    // now check the strides and sizes
    const size_t rows = rosData.layout.dim[0].size;
    const size_t cols = rosData.layout.dim[1].size;
    if (rosData.layout.dim[0].stride != 1) {
        cmnThrow("mtsROSToCISST(std::msgs::Float64MultiArray, vctDoubleMat): dim[0].stride must be equal to 1");
    }
    if (rosData.layout.dim[1].stride != rows) {
        cmnThrow("mtsROSToCISST(std::msgs::Float64MultiArray, vctDoubleMat): dim[1].stride must be equal to number of rows");
    }
    // now allocate and copy data
    cisstData.resize(rows, cols);
    auto row_major_linear_view = cisstData.reshaped<Eigen::RowMajor>();
    std::copy(rosData.data.begin(), rosData.data.end(),
              row_major_linear_view.begin());
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Vector3) & rosData,
                   Eigen::Vector3d& cisstData)
{
    cisstData[0] = rosData.x;
    cisstData[1] = rosData.y;
    cisstData[2] = rosData.z;
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Quaternion) & rosData,
                   Eigen::Quaterniond& cisstData)
{
    cisstData.x() = rosData.x;
    cisstData.y() = rosData.y;
    cisstData.z() = rosData.z;
    cisstData.w() = rosData.w;
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Pose) & rosData,
                   prmPositionCartesianGet& cisstData)
{
    mtsROSToCISST(rosData, cisstData.Position());
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Pose) & rosData,
                   prmPositionCartesianSet& cisstData)
{
    mtsROSToCISST(rosData, cisstData.Goal());
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Pose) & rosData,
                   Eigen::Isometry3d& cisstData)
{
    cisstData.translation().x() = rosData.position.x;
    cisstData.translation().y() = rosData.position.y;
    cisstData.translation().z() = rosData.position.z;
    Eigen::Quaterniond q(
        rosData.orientation.w,
        rosData.orientation.x,
        rosData.orientation.y,
        rosData.orientation.z
    );
    cisstData.linear() = q.toRotationMatrix();
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Transform) & rosData,
                   prmPositionCartesianGet& cisstData)
{
    mtsROSToCISST(rosData, cisstData.Position());
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Transform) & rosData,
                   prmPositionCartesianSet& cisstData)
{
    mtsROSToCISST(rosData, cisstData.Goal());
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Transform) & rosData,
                   Eigen::Isometry3d& cisstData)
{
    cisstData.translation().x() = rosData.translation.x;
    cisstData.translation().y() = rosData.translation.y;
    cisstData.translation().z() = rosData.translation.z;
    Eigen::Quaterniond q(
        rosData.rotation.w,
        rosData.rotation.x,
        rosData.rotation.y,
        rosData.rotation.z
    );
    cisstData.linear() = q.toRotationMatrix();
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Wrench) & rosData,
                   prmForceCartesianGet & cisstData)
{
    mtsROSToCISST(rosData, cisstData.Force());
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Wrench) & rosData,
                   prmForceCartesianSet & cisstData)
{
    mtsROSToCISST(rosData, cisstData.Force());
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Wrench) & rosData,
                   Eigen::Vector<double, 6>& cisstData)
{
    cisstData[0] = rosData.force.x;
    cisstData[1] = rosData.force.y;
    cisstData[2] = rosData.force.z;
    cisstData[3] = rosData.torque.x;
    cisstData[4] = rosData.torque.y;
    cisstData[5] = rosData.torque.z;
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Twist) & rosData,
                   prmVelocityCartesianGet & cisstData)
{
    cisstData.SetVelocityLinear(Eigen::Vector3d(rosData.linear.x, rosData.linear.y, rosData.linear.z));
    cisstData.SetVelocityAngular((Eigen::Vector3d(rosData.angular.x, rosData.angular.y, rosData.angular.z)));
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Twist) & rosData,
                   prmVelocityCartesianSet & cisstData)
{
    cisstData.SetVelocity(Eigen::Vector3d(rosData.linear.x, rosData.linear.y, rosData.linear.z));
    cisstData.SetAngularVelocity((Eigen::Vector3d(rosData.angular.x, rosData.angular.y, rosData.angular.z)));
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Twist) & rosData,
                   Eigen::Vector<double, 6>& cisstData)
{
    cisstData[0] = rosData.angular.x;
    cisstData[1] = rosData.angular.y;
    cisstData[2] = rosData.angular.z;
    cisstData[3] = rosData.linear.x;
    cisstData[4] = rosData.linear.y;
    cisstData[5] = rosData.linear.z;
}

void mtsROSToCISST(const CISST_RAL_MSG(sensor_msgs, JointState) & rosData,
                   prmPositionJointSet & cisstData)
{
    cisstData.Goal().resize(rosData.position.size());
    std::copy(rosData.position.begin(), rosData.position.end(),
              cisstData.Goal().begin());
    const auto velocitySize = rosData.velocity.size();
    if (velocitySize > 0) {
        cisstData.Velocity().resize(velocitySize);
        std::copy(rosData.velocity.begin(), rosData.velocity.end(),
                  cisstData.Velocity().begin());
    }
}

void mtsROSToCISST(const CISST_RAL_MSG(sensor_msgs, JointState) & rosData,
                   prmForceTorqueJointSet & cisstData)
{
    cisstData.ForceTorque().resize(rosData.effort.size());
    std::copy(rosData.effort.begin(), rosData.effort.end(),
              cisstData.ForceTorque().begin());
}

void mtsROSToCISST(const CISST_RAL_MSG(sensor_msgs, JointState) & rosData,
                   prmVelocityJointSet & cisstData)
{
    cisstData.Goal().resize(rosData.velocity.size());
    std::copy(rosData.velocity.begin(), rosData.velocity.end(),
              cisstData.Goal().begin());
}

void mtsROSToCISST(const CISST_RAL_MSG(sensor_msgs, JointState) & rosData,
                   prmStateJoint & cisstData)
{
    cisstData.Name().resize(rosData.name.size());
    std::copy(rosData.name.begin(), rosData.name.end(),
              cisstData.Name().begin());
    cisstData.Position().resize(rosData.position.size());
    std::copy(rosData.position.begin(), rosData.position.end(),
              cisstData.Position().begin());
    cisstData.Velocity().resize(rosData.velocity.size());
    std::copy(rosData.velocity.begin(), rosData.velocity.end(),
              cisstData.Velocity().begin());
    cisstData.Effort().resize(rosData.effort.size());
    std::copy(rosData.effort.begin(), rosData.effort.end(),
              cisstData.Effort().begin());
}

void mtsROSToCISST(const CISST_RAL_MSG(sensor_msgs, Joy) & rosData,
                   prmEventButton & cisstData)
{
    if (rosData.buttons.size() < 1) {
        cisstData.Type() = prmEventButton::UNDEFINED;
        return;
    }
    if (rosData.buttons[0] == 1) {
        cisstData.Type() = prmEventButton::PRESSED;
    } else if (rosData.buttons[0] == 0) {
        cisstData.Type() = prmEventButton::RELEASED;
    } else if (rosData.buttons[0] == 2) {
        cisstData.Type() = prmEventButton::CLICKED;
    } else {
        cisstData.Type() = prmEventButton::UNDEFINED;
    }
}

void mtsROSToCISST(const CISST_RAL_MSG(sensor_msgs, Joy) & rosData,
                   prmInputData & cisstData)
{
    cisstData.AnalogInputs().resize(rosData.axes.size());
    cisstData.DigitalInputs().resize(rosData.buttons.size());
    std::copy(rosData.axes.begin(), rosData.axes.end(),
              cisstData.AnalogInputs().begin());
    std::copy(rosData.buttons.begin(), rosData.buttons.end(),
              cisstData.DigitalInputs().begin());
}

void mtsROSToCISST(const CISST_RAL_MSG(diagnostic_msgs, KeyValue) & rosData,
                   prmKeyValue & cisstData)
{
    cisstData.Key = rosData.key;
    cisstData.Value = rosData.value;
}

void mtsROSToCISST(const CISST_RAL_MSG(cisst_msgs, DoubleVec) & rosData,
                   prmPositionJointSet & cisstData)
{
    const size_t size = rosData.data.size();
    cisstData.Goal().resize(size);
    for (size_t i = 0; i < size; ++i) {
        cisstData.Goal()(i) = rosData.data[i];
    }
}

void mtsROSToCISST(const CISST_RAL_MSG(cisst_msgs, DoubleVec) & rosData,
                   Eigen::VectorXd& cisstData)
{
    const size_t size = rosData.data.size();
    cisstData.resize(size);
    for (size_t i = 0; i < size; ++i) {
        cisstData(i) = rosData.data[i];
    }
}

void mtsROSToCISST(const CISST_RAL_MSG(cisst_msgs, IntervalStatistics) & rosData,
                   mtsIntervalStatistics & cisstData)
{
    cisstData.SetFromExisting(rosData.period_avg,
                              rosData.period_std_dev,
                              rosData.period_min,
                              rosData.period_max,
                              rosData.compute_time_avg,
                              rosData.compute_time_std_dev,
                              rosData.compute_time_min,
                              rosData.compute_time_max,
                              rosData.number_of_samples,
                              rosData.number_of_overruns,
                              rosData.statistics_interval);
}

void mtsROSToCISST(const CISST_RAL_SRV_REQ(cisst_msgs, ConvertFloat64Array) & rosData,
                   Eigen::VectorXd& cisstData)
{
    cisstData.resize(rosData.input.size());
    std::copy(rosData.input.begin(), rosData.input.end(),
              cisstData.begin());
}
