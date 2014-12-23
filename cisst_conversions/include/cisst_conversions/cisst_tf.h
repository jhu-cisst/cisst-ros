
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


}  // namespace tf
