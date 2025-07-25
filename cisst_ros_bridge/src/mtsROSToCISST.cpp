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

#include <cisst_ros_bridge/mtsROSToCISST.h>

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
                   vctDoubleVec & cisstData)
{
    const size_t size = rosData.data.size();
    if (size != 0) {
        cisstData.SetSize(size);
        std::copy(rosData.data.begin(), rosData.data.end(),
                  cisstData.begin());
    }
}

void mtsROSToCISST(const CISST_RAL_MSG(std_msgs, Float64MultiArray) & rosData,
                   vctDoubleMat & cisstData)
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
    cisstData.SetSize(rows, cols);
    std::copy(rosData.data.begin(), rosData.data.end(),
              cisstData.begin());
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Vector3) & rosData,
                   vct3 & cisstData)
{
    cisstData[0] = rosData.x;
    cisstData[1] = rosData.y;
    cisstData[2] = rosData.z;
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Quaternion) & rosData,
                   vctMatRot3 & cisstData)
{
    vctQuatRot3 quat;
    quat.X() = rosData.x;
    quat.Y() = rosData.y;
    quat.Z() = rosData.z;
    quat.W() = rosData.w;
    cisstData.FromNormalized(quat);
}


void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Pose) & rosData,
                   prmPositionCartesianGet & cisstData)
{
    mtsROSPoseToCISST(rosData, cisstData.Position());
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Pose) & rosData,
                   prmPositionCartesianSet & cisstData)
{
    mtsROSPoseToCISST(rosData, cisstData.Goal());
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, PoseStamped) & rosData,
                   prmPositionCartesianGet & cisstData)
{
    mtsROSToCISST(rosData.pose, cisstData);
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, PoseStamped) & rosData,
                   prmPositionCartesianSet & cisstData)
{
    mtsROSToCISST(rosData.pose, cisstData);
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, TransformStamped) & rosData,
                   prmPositionCartesianGet & cisstData)
{
    mtsROSToCISST(rosData.transform, cisstData.Position());
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, TransformStamped) & rosData,
                   prmPositionCartesianSet & cisstData)
{
    mtsROSToCISST(rosData.transform, cisstData.Goal());
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, PoseStamped) & rosData,
                   vctFrm3 & cisstData)
{
    mtsROSPoseToCISST(rosData.pose, cisstData);
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, PoseStamped) & rosData,
                   vctFrm4x4 & cisstData)
{
    mtsROSPoseToCISST(rosData.pose, cisstData);
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Pose) & rosData,
                   vctFrm3 & cisstData)
{
    mtsROSPoseToCISST(rosData, cisstData);
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Pose) & rosData,
                   vctFrm4x4 & cisstData)
{
    mtsROSPoseToCISST(rosData, cisstData);
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Pose) & rosData,
                   mtsFrm4x4 & cisstData)
{
    mtsROSPoseToCISST(rosData, cisstData);
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Transform) & rosData,
                   prmPositionCartesianGet & cisstData)
{
    mtsROSTransformToCISST(rosData, cisstData.Position());
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Transform) & rosData,
                   vctFrm3 & cisstData)
{
    mtsROSTransformToCISST(rosData, cisstData);
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Transform) & rosData,
                   vctFrm4x4 & cisstData)
{
    mtsROSTransformToCISST(rosData, cisstData);
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Transform) & rosData,
                   mtsFrm4x4 & cisstData)
{
    mtsROSTransformToCISST(rosData, cisstData);
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, TransformStamped) & rosData,
                   mtsFrm4x4 & cisstData)
{
    mtsROSTransformToCISST(rosData.transform, cisstData);
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Wrench) & rosData,
                   prmForceCartesianGet & cisstData)
{
    vctFixedSizeVector<double, 6>
        vctFT(rosData.force.x, rosData.force.y, rosData.force.z,
              rosData.torque.x, rosData.torque.y, rosData.torque.z);
    cisstData.SetForce(vctFT);
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, WrenchStamped) & rosData,
                   prmForceCartesianGet & cisstData)
{
    mtsROSToCISST(rosData.wrench, cisstData);
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Wrench) & rosData,
                   prmForceCartesianSet & cisstData)
{
    vctFixedSizeVector<double, 6>
        vctFT(rosData.force.x, rosData.force.y, rosData.force.z,
              rosData.torque.x, rosData.torque.y, rosData.torque.z);
    cisstData.SetForce(vctFT);
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, WrenchStamped) & rosData,
                   prmForceCartesianSet & cisstData)
{
    mtsROSToCISST(rosData.wrench, cisstData);
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Wrench) & rosData,
                   mtsDoubleVec & cisstData)
{
    cisstData.SetSize(6);
    cisstData.Element(0) = rosData.force.x;
    cisstData.Element(1) = rosData.force.y;
    cisstData.Element(2) = rosData.force.z;
    cisstData.Element(3) = rosData.torque.x;
    cisstData.Element(4) = rosData.torque.y;
    cisstData.Element(5) = rosData.torque.z;
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, WrenchStamped) & rosData,
                   mtsDoubleVec & cisstData)
{
    mtsROSToCISST(rosData.wrench, cisstData);
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Twist) & rosData,
                   prmVelocityCartesianSet & cisstData)
{
    cisstData.SetVelocity(vct3(rosData.linear.x, rosData.linear.y, rosData.linear.z));
    cisstData.SetAngularVelocity(vct3(rosData.angular.x, rosData.angular.y, rosData.angular.z));
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, TwistStamped) & rosData,
                   prmVelocityCartesianSet & cisstData)
{
    mtsROSToCISST(rosData.twist, cisstData);
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, Twist) & rosData,
                   prmVelocityCartesianGet & cisstData)
{
    cisstData.SetVelocityLinear(vct3(rosData.linear.x, rosData.linear.y, rosData.linear.z));
    cisstData.SetVelocityAngular(vct3(rosData.angular.x, rosData.angular.y, rosData.angular.z));
}

void mtsROSToCISST(const CISST_RAL_MSG(geometry_msgs, TwistStamped) & rosData,
                   prmVelocityCartesianGet & cisstData)
{
    mtsROSToCISST(rosData.twist, cisstData);
}

void mtsROSToCISST(const CISST_RAL_MSG(sensor_msgs, JointState) & rosData,
                   prmPositionJointSet & cisstData)
{
    cisstData.Goal().SetSize(rosData.position.size());
    std::copy(rosData.position.begin(), rosData.position.end(),
              cisstData.Goal().begin());
    const auto velocitySize = rosData.velocity.size();
    if (velocitySize > 0) {
        cisstData.Velocity().SetSize(velocitySize);
        std::copy(rosData.velocity.begin(), rosData.velocity.end(),
                  cisstData.Velocity().begin());
    }
}

void mtsROSToCISST(const CISST_RAL_MSG(sensor_msgs, JointState) & rosData,
                   prmForceTorqueJointSet & cisstData)
{
    cisstData.ForceTorque().SetSize(rosData.effort.size());
    std::copy(rosData.effort.begin(), rosData.effort.end(),
              cisstData.ForceTorque().begin());
}

void mtsROSToCISST(const CISST_RAL_MSG(sensor_msgs, JointState) & rosData,
                   prmVelocityJointSet & cisstData)
{
    cisstData.Goal().SetSize(rosData.velocity.size());
    std::copy(rosData.velocity.begin(), rosData.velocity.end(),
              cisstData.Goal().begin());
}

void mtsROSToCISST(const CISST_RAL_MSG(sensor_msgs, JointState) & rosData,
                   prmStateJoint & cisstData)
{
    cisstData.Name().resize(rosData.name.size());
    std::copy(rosData.name.begin(), rosData.name.end(),
              cisstData.Name().begin());
    cisstData.Position().SetSize(rosData.position.size());
    std::copy(rosData.position.begin(), rosData.position.end(),
              cisstData.Position().begin());
    cisstData.Velocity().SetSize(rosData.velocity.size());
    std::copy(rosData.velocity.begin(), rosData.velocity.end(),
              cisstData.Velocity().begin());
    cisstData.Effort().SetSize(rosData.effort.size());
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
    cisstData.AnalogInputs().SetSize(rosData.axes.size());
    cisstData.DigitalInputs().SetSize(rosData.buttons.size());
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
        cisstData.Goal().Element(i) = rosData.data[i];
    }
}

void mtsROSToCISST(const CISST_RAL_MSG(cisst_msgs, DoubleVec) & rosData,
                   vctDoubleVec & cisstData)
{
    const size_t size = rosData.data.size();
    cisstData.resize(size);
    for (size_t i = 0; i < size; ++i) {
        cisstData.Element(i) = rosData.data[i];
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
                   vctDoubleVec & cisstData)
{
    cisstData.SetSize(rosData.input.size());
    std::copy(rosData.input.begin(), rosData.input.end(),
              cisstData.begin());
}
