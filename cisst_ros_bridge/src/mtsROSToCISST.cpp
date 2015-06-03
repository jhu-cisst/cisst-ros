/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen, Adnan Munawar
  Created on: 2013-05-21

  (C) Copyright 2013-2015 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

#include "cisst_ros_bridge/mtsROSToCISST.h"

void mtsROSToCISST(const std_msgs::Float32 & rosData, double & cisstData)
{
    cisstData = rosData.data;
}

void mtsROSToCISST(const std_msgs::Bool & rosData, bool & cisstData)
{
    cisstData = rosData.data;
}

void mtsROSToCISST(const std_msgs::String & rosData, std::string & cisstData)
{
    cisstData = rosData.data;
}

void mtsROSToCISST(const geometry_msgs::Vector3 & rosData, vct3 & cisstData)
{
  cisstData[0] = rosData.x;
  cisstData[1] = rosData.y;
  cisstData[2] = rosData.z;
}

void mtsROSToCISST(const geometry_msgs::Quaternion & rosData, vctMatRot3 & cisstData)
{
  vctQuatRot3 quat;
  quat.X() = rosData.x;
  quat.Y() = rosData.y;
  quat.Z() = rosData.z;
  quat.W() = rosData.w;
  cisstData.FromNormalized(quat);
}


void mtsROSToCISST(const geometry_msgs::Pose & rosData, prmPositionCartesianGet & cisstData)
{
    cisstData.Position().Translation().X() = rosData.position.x;
    cisstData.Position().Translation().Y() = rosData.position.y;
    cisstData.Position().Translation().Z() = rosData.position.z;
    vctQuatRot3 quat;
    quat.X() = rosData.orientation.x;
    quat.Y() = rosData.orientation.y;
    quat.Z() = rosData.orientation.z;
    quat.W() = rosData.orientation.w;
    vctMatRot3 rotation(quat, VCT_NORMALIZE);
    cisstData.Position().Rotation().Assign(rotation);
}

void mtsROSToCISST(const geometry_msgs::Pose & rosData, prmPositionCartesianSet & cisstData)
{
    cisstData.Goal().Translation().X() = rosData.position.x;
    cisstData.Goal().Translation().Y() = rosData.position.y;
    cisstData.Goal().Translation().Z() = rosData.position.z;
    vctQuatRot3 quat;
    quat.X() = rosData.orientation.x;
    quat.Y() = rosData.orientation.y;
    quat.Z() = rosData.orientation.z;
    quat.W() = rosData.orientation.w;
    vctMatRot3 rotation(quat, VCT_NORMALIZE);
    cisstData.Goal().Rotation().FromNormalized(rotation);
}

void mtsROSToCISST(const geometry_msgs::PoseStamped & rosData, prmPositionCartesianSet & cisstData)
{
    mtsROSToCISST(rosData.pose, cisstData);
}

void mtsROSToCISST(const geometry_msgs::Pose & rosData, vctFrm4x4 & cisstData)
{
    cisstData.Translation().X() = rosData.position.x;
    cisstData.Translation().Y() = rosData.position.y;
    cisstData.Translation().Z() = rosData.position.z;
    vctQuatRot3 quat;
    quat.X() = rosData.orientation.x;
    quat.Y() = rosData.orientation.y;
    quat.Z() = rosData.orientation.z;
    quat.W() = rosData.orientation.w;
    vctMatRot3 rotation(quat, VCT_NORMALIZE);
    cisstData.Rotation().Assign(rotation);
}

void mtsROSToCISST(const geometry_msgs::Transform & rosData, prmPositionCartesianGet & cisstData)
{
    cisstData.Position().Translation().X() = rosData.translation.x;
    cisstData.Position().Translation().Y() = rosData.translation.y;
    cisstData.Position().Translation().Z() = rosData.translation.z;
    vctQuatRot3 quat;
    quat.X() = rosData.rotation.x;
    quat.Y() = rosData.rotation.y;
    quat.Z() = rosData.rotation.z;
    quat.W() = rosData.rotation.w;
    vctMatRot3 rotation(quat, VCT_NORMALIZE);
    cisstData.Position().Rotation().Assign(rotation);
}

void mtsROSToCISST(const geometry_msgs::Transform & rosData, vctFrm4x4 & cisstData)
{
    cisstData.Translation().X() = rosData.translation.x;
    cisstData.Translation().Y() = rosData.translation.y;
    cisstData.Translation().Z() = rosData.translation.z;
    vctQuatRot3 quat;
    quat.X() = rosData.rotation.x;
    quat.Y() = rosData.rotation.y;
    quat.Z() = rosData.rotation.z;
    quat.W() = rosData.rotation.w;
    vctMatRot3 rotation(quat, VCT_NORMALIZE);
    cisstData.Rotation().FromNormalized(rotation);
}

void mtsROSToCISST(const geometry_msgs::Wrench & rosData, prmForceCartesianSet & cisstData)
{
    vctFixedSizeVector<double,6> vctFT(
                rosData.force.x, rosData.force.y, rosData.force.z,
                rosData.torque.x, rosData.torque.y, rosData.torque.z);
    cisstData.SetForce(vctFT);
    cisstData.SetValid(true);
}

void mtsROSToCISST(const geometry_msgs::WrenchStamped & rosData, prmForceCartesianSet & cisstData)
{
    mtsROSToCISST(rosData.wrench, cisstData);
}

void mtsROSToCISST(const sensor_msgs::JointState & rosData, prmPositionJointSet & cisstData)
{
    vctDoubleVec DesiredPosition;
    DesiredPosition.SetSize(rosData.position.size());
    DesiredPosition.SetAll(0.0);
    for (unsigned int i = 0; i < rosData.position.size(); ++i) {
        DesiredPosition.at(i) = rosData.position[i];
    }
    cisstData.SetGoal(DesiredPosition);
}

void mtsROSToCISST(const sensor_msgs::JointState & rosData, prmForceTorqueJointSet & cisstData)
{
    vctDoubleVec newVec;
    newVec.SetSize(rosData.effort.size());
    for (std::size_t i = 0; i < rosData.effort.size(); ++i) {
        newVec.at(i) = rosData.effort[i];
    }
    cisstData.SetForceTorque(newVec);
}

void mtsROSToCISST(const cisst_msgs::vctDoubleVec & rosData, prmPositionJointSet & cisstData)
{
    const size_t size = rosData.data.size();
    cisstData.Goal().resize(size);
    for (size_t i = 0; i < size; ++i) {
        cisstData.Goal().Element(i) = rosData.data[i];
    }
}

void mtsROSToCISST(const cisst_msgs::vctDoubleVec & rosData, vctDoubleVec & cisstData)
{
    const size_t size = rosData.data.size();
    cisstData.resize(size);
    for (size_t i = 0; i < size; ++i) {
        cisstData.Element(i) = rosData.data[i];
    }
}

void mtsROSToCISST(const cisst_msgs::prmFixtureGainCartesianSet & rosData, prmFixtureGainCartesianSet & cisstData)
{
  vct3 vct3Data;   // holder for vct3 data
  vctMatRot3 rotationData;   // holder for rotation data

  // reference frame/point
  mtsROSToCISST(rosData.RefPoint, vct3Data);
  cisstData.SetRefPoint(vct3Data);
  mtsROSToCISST(rosData.RotationRefToMaster, rotationData);
  cisstData.SetRotationRefToMaster(rotationData);

  // vf pos/rot
  mtsROSToCISST(rosData.ForcePosition, vct3Data);
  cisstData.SetForcePosition(vct3Data);
  mtsROSToCISST(rosData.ForceOrientation, rotationData);
  cisstData.SetForceOrientation(rotationData);
  mtsROSToCISST(rosData.TorqueOrientation, rotationData);
  cisstData.SetTorqueOrientation(rotationData);

  // force gains
  mtsROSToCISST(rosData.PosStiffPos, vct3Data);
  cisstData.SetPositionStiffnessPos(vct3Data);

  mtsROSToCISST(rosData.PosStiffNeg, vct3Data);
  cisstData.SetPositionStiffnessNeg(vct3Data);

  mtsROSToCISST(rosData.PosDampingPos, vct3Data);
  cisstData.SetPositionDampingPos(vct3Data);

  mtsROSToCISST(rosData.PosDampingNeg, vct3Data);
  cisstData.SetPositionDampingNeg(vct3Data);

  mtsROSToCISST(rosData.ForceBiasPos, vct3Data);
  cisstData.SetForceBiasPos(vct3Data);

  mtsROSToCISST(rosData.ForceBiasNeg, vct3Data);
  cisstData.SetForceBiasNeg(vct3Data);

  // toqrue gains
  mtsROSToCISST(rosData.OriStiffPos, vct3Data);
  cisstData.SetOrientationStiffnessPos(vct3Data);

  mtsROSToCISST(rosData.OriStiffNeg, vct3Data);
  cisstData.SetOrientationStiffnessNeg(vct3Data);

  mtsROSToCISST(rosData.OriDampingPos, vct3Data);
  cisstData.SetOrientationDampingPos(vct3Data);

  mtsROSToCISST(rosData.OriDampingNeg, vct3Data);
  cisstData.SetOrientationDampingNeg(vct3Data);

  mtsROSToCISST(rosData.TorqueBiasPos, vct3Data);
  cisstData.SetTorqueBiasPos(vct3Data);

  mtsROSToCISST(rosData.TorqueBiasNeg, vct3Data);
  cisstData.SetTorqueBiasNeg(vct3Data);
}

