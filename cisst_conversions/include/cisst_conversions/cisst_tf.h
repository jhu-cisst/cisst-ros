
#include <tf_conversions/tf_kdl.h>
#include <cisstVector.h>

namespace tf {


//tf::quaternionKDLToTF(const KDL::Rotation &k, Quaternion &t)


void quaternionCISSTToTF(const vctQuaternionRotation3<double> &c, Quaternion &t)
{
  double x, y, z, w;
  x = c.X();
  y = c.Y();
  z = c.Z();
  w = c.W();
  t = tf::Quaternion(x, y, z, w);
}

void quaternionCISSTToMsg(const vctQuaternionRotation3<double> &c, geometry_msgs::Quaternion &m)
{
  m.x = c.X();
  m.y = c.Y();
  m.z = c.Z();
  m.w = c.W();
}

void quaternionKDLToCISST(const KDL::Rotation &k, vctQuaternionRotation3<double> &c)
{
  double x,y, z, w;
  k.GetQuaternion(x, y, z, w);
  c.X() = x;
  c.Y() = y;
  c.Z() = z;
  c.W() = w;
}

void poseCISSTToMsg(const vctFrame4x4<double> &c, geometry_msgs::Pose &m)
{
  m.position.x = c.Translation().X();
  m.position.y = c.Translation().Y();
  m.position.z = c.Translation().Z();

  vctQuaternionRotation3<double> crot;
  crot.FromNormalized(c.Rotation());

  m.orientation.x = crot.X();
  m.orientation.y = crot.Y();
  m.orientation.z = crot.Z();
  m.orientation.w = crot.W();
}

void poseKDLToCISST(const KDL::Frame &k, vctFrame4x4<double> &c)
{
  c.Translation() = vct3(k.p.x(), k.p.y(), k.p.z());
  vctQuatRot3 cRot;
  quaternionKDLToCISST(k.M, cRot);
  c.Rotation().FromNormalized(cRot);
}

void poseCISSTToKDL(const vctFrame4x4<double> &c, KDL::Frame &k)
{
  k.p = KDL::Vector(c.Translation().X(),
                    c.Translation().Y(),
                    c.Translation().Z());
  vctQuatRot3 crot;
  crot.FromNormalized(c.Rotation());
  k.M = KDL::Rotation::Quaternion(crot.X(), crot.Y(), crot.Z(), crot.W());
}

}  // namespace tf
