/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen, Adnan Munawar
  Created on: 2013-05-21

  (C) Copyright 2013-2019 Johns Hopkins University (JHU), All Rights Reserved.

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

void mtsROSToCISST(const std_msgs::Float64 & rosData, double & cisstData)
{
    cisstData = rosData.data;
}

void mtsROSToCISST(const std_msgs::Int32 & rosData, int & cisstData)
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
    mtsROSToCISSTNoHeader(cisstData);
    mtsROSPoseToCISST(rosData, cisstData.Position());
}

void mtsROSToCISST(const geometry_msgs::Pose & rosData, prmPositionCartesianSet & cisstData)
{
    mtsROSToCISSTNoHeader(cisstData);
    mtsROSPoseToCISST(rosData, cisstData.Goal());
}

void mtsROSToCISST(const geometry_msgs::PoseStamped & rosData, prmPositionCartesianGet & cisstData)
{
    mtsROSToCISSTHeader(rosData, cisstData);
    mtsROSToCISST(rosData.pose, cisstData);
}

void mtsROSToCISST(const geometry_msgs::PoseStamped & rosData, prmPositionCartesianSet & cisstData)
{
    mtsROSToCISSTHeader(rosData, cisstData);
    mtsROSToCISST(rosData.pose, cisstData);
}

void mtsROSToCISST(const geometry_msgs::TransformStamped & rosData, prmPositionCartesianSet & cisstData)
{
    mtsROSToCISSTHeader(rosData, cisstData);
    mtsROSToCISST(rosData.transform, cisstData.Goal());
}

void mtsROSToCISST(const geometry_msgs::PoseStamped & rosData, vctFrm3 & cisstData)
{
    mtsROSPoseToCISST(rosData.pose, cisstData);
}

void mtsROSToCISST(const geometry_msgs::Pose & rosData, vctFrm3 & cisstData)
{
    mtsROSPoseToCISST(rosData, cisstData);
}

void mtsROSToCISST(const geometry_msgs::Pose & rosData, vctFrm4x4 & cisstData)
{
    mtsROSPoseToCISST(rosData, cisstData);
}

void mtsROSToCISST(const geometry_msgs::Pose & rosData, mtsFrm4x4 & cisstData)
{
    mtsROSToCISSTNoHeader(cisstData);
    mtsROSPoseToCISST(rosData, cisstData);
}

void mtsROSToCISST(const geometry_msgs::Transform & rosData, prmPositionCartesianGet & cisstData)
{
    mtsROSToCISSTNoHeader(cisstData);
    mtsROSTransformToCISST(rosData, cisstData.Position());
}

void mtsROSToCISST(const geometry_msgs::Transform & rosData, vctFrm3 & cisstData)
{
    mtsROSTransformToCISST(rosData, cisstData);
}

void mtsROSToCISST(const geometry_msgs::Transform & rosData, vctFrm4x4 & cisstData)
{
    mtsROSTransformToCISST(rosData, cisstData);
}

void mtsROSToCISST(const geometry_msgs::Transform & rosData, mtsFrm4x4 & cisstData)
{
    mtsROSToCISSTNoHeader(cisstData);
    mtsROSTransformToCISST(rosData, cisstData);
}

void mtsROSToCISST(const geometry_msgs::TransformStamped & rosData, mtsFrm4x4 & cisstData)
{
    mtsROSToCISSTHeader(rosData, cisstData);
    mtsROSTransformToCISST(rosData.transform, cisstData);
}

void mtsROSToCISST(const geometry_msgs::Wrench & rosData, prmForceCartesianSet & cisstData)
{
    mtsROSToCISSTNoHeader(cisstData);
    vctFixedSizeVector<double, 6>
        vctFT(rosData.force.x, rosData.force.y, rosData.force.z,
              rosData.torque.x, rosData.torque.y, rosData.torque.z);
    cisstData.SetForce(vctFT);
}

void mtsROSToCISST(const geometry_msgs::WrenchStamped & rosData, prmForceCartesianSet & cisstData)
{
    mtsROSToCISSTHeader(rosData, cisstData);
    mtsROSToCISST(rosData.wrench, cisstData);
}

void mtsROSToCISST(const geometry_msgs::Wrench & rosData, mtsDoubleVec & cisstData)
{
    mtsROSToCISSTNoHeader(cisstData);
    cisstData.SetSize(6);
    cisstData.Element(0) = rosData.force.x;
    cisstData.Element(1) = rosData.force.y;
    cisstData.Element(2) = rosData.force.z;
    cisstData.Element(3) = rosData.torque.x;
    cisstData.Element(4) = rosData.torque.y;
    cisstData.Element(5) = rosData.torque.z;
}

void mtsROSToCISST(const geometry_msgs::WrenchStamped & rosData, mtsDoubleVec & cisstData)
{
    mtsROSToCISSTHeader(rosData, cisstData);
    mtsROSToCISST(rosData.wrench, cisstData);
}

void mtsROSToCISST(const geometry_msgs::Twist & rosData, prmVelocityCartesianSet & cisstData)
{
    mtsROSToCISSTNoHeader(cisstData);
    cisstData.SetVelocity(vct3(rosData.linear.x, rosData.linear.y, rosData.linear.z));
    cisstData.SetAngularVelocity(vct3(rosData.angular.x, rosData.angular.y, rosData.angular.z));
}

void mtsROSToCISST(const geometry_msgs::TwistStamped & rosData, prmVelocityCartesianSet & cisstData)
{
    mtsROSToCISSTHeader(rosData, cisstData);
    cisstData.SetTimestamp(rosData.header.stamp.toSec());
    mtsROSToCISST(rosData.twist, cisstData);
}


void mtsROSToCISST(const sensor_msgs::JointState & rosData, prmPositionJointSet & cisstData)
{
    mtsROSToCISSTHeader(rosData, cisstData);
    cisstData.Goal().SetSize(rosData.position.size());
    std::copy(rosData.position.begin(), rosData.position.end(),
              cisstData.Goal().begin());
}

void mtsROSToCISST(const sensor_msgs::JointState & rosData, prmForceTorqueJointSet & cisstData)
{
    mtsROSToCISSTHeader(rosData, cisstData);
    cisstData.ForceTorque().SetSize(rosData.effort.size());
    std::copy(rosData.effort.begin(), rosData.effort.end(),
              cisstData.ForceTorque().begin());
}

void mtsROSToCISST(const sensor_msgs::JointState & rosData, prmVelocityJointSet & cisstData)
{
    mtsROSToCISSTHeader(rosData, cisstData);
    cisstData.Goal().SetSize(rosData.velocity.size());
    std::copy(rosData.velocity.begin(), rosData.velocity.end(),
              cisstData.Goal().begin());
}

void mtsROSToCISST(const diagnostic_msgs::KeyValue & rosData, prmKeyValue & cisstData)
{
    cisstData.Key = rosData.key;
    cisstData.Value = rosData.value;
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

void mtsROSToCISST(const cisst_msgs::prmCartesianImpedanceGains & rosData,
                   prmCartesianImpedanceGains & cisstData)
{
    mtsROSToCISSTHeader(rosData, cisstData);
    // vf pos/rot
    mtsROSToCISST(rosData.ForcePosition,
                  cisstData.ForcePosition());
    mtsROSToCISST(rosData.ForceOrientation,
                  cisstData.ForceOrientation());
    mtsROSToCISST(rosData.TorqueOrientation,
                  cisstData.TorqueOrientation());

    // force gains
    mtsROSToCISST(rosData.PosDeadbandPos,
                  cisstData.PositionDeadbandPos());
    mtsROSToCISST(rosData.PosDeadbandNeg,
                  cisstData.PositionDeadbandNeg());
    mtsROSToCISST(rosData.PosStiffPos,
                  cisstData.PositionStiffnessPos());
    mtsROSToCISST(rosData.PosStiffNeg,
                  cisstData.PositionStiffnessNeg());
    mtsROSToCISST(rosData.PosDampingPos,
                  cisstData.PositionDampingPos());
    mtsROSToCISST(rosData.PosDampingNeg,
                  cisstData.PositionDampingNeg());
    mtsROSToCISST(rosData.ForceBiasPos,
                  cisstData.ForceBiasPos());
    mtsROSToCISST(rosData.ForceBiasNeg,
                  cisstData.ForceBiasNeg());

    // torque gains
    mtsROSToCISST(rosData.OriDeadbandPos,
                  cisstData.OrientationDeadbandPos());
    mtsROSToCISST(rosData.OriDeadbandNeg,
                  cisstData.OrientationDeadbandNeg());
    mtsROSToCISST(rosData.OriStiffPos,
                  cisstData.OrientationStiffnessPos());
    mtsROSToCISST(rosData.OriStiffNeg,
                  cisstData.OrientationStiffnessNeg());
    mtsROSToCISST(rosData.OriDampingPos,
                  cisstData.OrientationDampingPos());
    mtsROSToCISST(rosData.OriDampingNeg,
                  cisstData.OrientationDampingNeg());
    mtsROSToCISST(rosData.TorqueBiasPos,
                  cisstData.TorqueBiasPos());
    mtsROSToCISST(rosData.TorqueBiasNeg,
                  cisstData.TorqueBiasNeg());
}
