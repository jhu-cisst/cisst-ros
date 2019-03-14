Header header

# vf pos & ori
geometry_msgs/Quaternion  ForceOrientation
geometry_msgs/Vector3     ForcePosition
geometry_msgs/Quaternion  TorqueOrientation

# force gains
geometry_msgs/Vector3     PosDeadbandPos
geometry_msgs/Vector3     PosDeadbandNeg
geometry_msgs/Vector3     PosStiffPos
geometry_msgs/Vector3     PosStiffNeg
geometry_msgs/Vector3     PosDampingPos
geometry_msgs/Vector3     PosDampingNeg
geometry_msgs/Vector3     ForceBiasPos
geometry_msgs/Vector3     ForceBiasNeg

# torque gains
geometry_msgs/Vector3     OriDeadbandPos
geometry_msgs/Vector3     OriDeadbandNeg
geometry_msgs/Vector3     OriStiffPos
geometry_msgs/Vector3     OriStiffNeg
geometry_msgs/Vector3     OriDampingPos
geometry_msgs/Vector3     OriDampingNeg
geometry_msgs/Vector3     TorqueBiasPos
geometry_msgs/Vector3     TorqueBiasNeg
