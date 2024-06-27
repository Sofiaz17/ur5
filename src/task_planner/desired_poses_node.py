#!/usr/bin/env python


import rospy as ros

from ur5.srv import GetDesiredPoses,GetDesiredPosesRequest,GetDesiredPosesResponse
from ur5.msg import BlockParams

from tf.transformations import quaternion_from_euler


desired_poses = []
PI = 3.1415926535
z_offset = 0.87

info_name = "[desired_poses_node]:"
debug_mode = False

class DesiredPose3D:
  def __init__(self, label, x, y, z, theta):
    self.label = label
    self.x = x
    self.y = y
    self.z = z
    self.theta = theta

def define_desired_poses():
  global desired_poses
  desired_poses.append(DesiredPose3D( label = "X1-Y2-Z2-TWINFILLET", x = 0.8, y = 0.4, z = z_offset, theta = PI ))
  desired_poses.append(DesiredPose3D( label = "X1-Y4-Z2", x = 0.3, y = 0.5, z = z_offset, theta = PI/6 ))
  desired_poses.append(DesiredPose3D( label = "X1-Y1-Z2", x = -0.1, y = 0.4, z = z_offset, theta = PI/6 ))
  desired_poses.append(DesiredPose3D( label = "X2-Y2-Z2", x = -0.1, y = 0.5, z = z_offset, theta = PI/6 ))
  desired_poses.append(DesiredPose3D( label = "X1-Y2-Z2-CHAMFER", x = -0.1, y = 0.6, z = z_offset, theta = PI/3 ))
  desired_poses.append(DesiredPose3D( label = "X2-Y2-Z2-FILLET", x = -0.1, y = 0.5, z = z_offset, theta = PI/2 ))
  desired_poses.append(DesiredPose3D( label = "X1-Y3-Z2", x = 0.1, y = 0.35, z = z_offset, theta = PI/3 ))
  desired_poses.append(DesiredPose3D( label = "X1-Y2-Z1", x = 0.1, y = 0.35, z = z_offset, theta = PI/3 ))
  desired_poses.append(DesiredPose3D( label = "X1-Y4-Z1", x = 0.8, y = 0.7, z = z_offset, theta = PI/4 ))

define_desired_poses()

def get_desired_poses_handler(req):
  """! handler of get_desired_poses service, type ur5.srv.GetDesiredPoses
  It initializes a vector with block desired poses and populates the GetDesiredPoses with them.
  @param req: empty
  @return res: the list of desired poses
  """
  ros.loginfo("%s desired_poses_node contacted", info_name)
  res = GetDesiredPosesResponse()

  res.dim = len(desired_poses)
  for i in range(res.dim):
    res.poses.append(BlockParams())

    res.poses[i].label = desired_poses[i].label
    res.poses[i].pose.position.x = desired_poses[i].x
    res.poses[i].pose.position.y = desired_poses[i].y
    res.poses[i].pose.position.z = z_offset

    q = quaternion_from_euler(0.0, 0.0, desired_poses[i].theta, 'sxyz')
    res.poses[i].pose.orientation.x = q[0]
    res.poses[i].pose.orientation.y = q[1]
    res.poses[i].pose.orientation.z = q[2]
    res.poses[i].pose.orientation.w = q[3]


  if debug_mode:
    ros.loginfo("%s I have %d desired poses:", info_name, res.dim)
    for i in range(res.dim):
      pose = res.poses[i]
      ros.loginfo("%s   %d: label=\"%s\", position=[%f,%f,%f], quaternion=%f+[%f,%f,%f]", info_name, i, pose.label, pose.position.x, pose.position.y, pose.position.z, pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)

  return res


if __name__ == "__main__":
  ros.init_node("desired_poses_node")
  service = ros.Service("get_desired_poses", GetDesiredPoses, get_desired_poses_handler)
  debug_mode = ros.get_param("/debug_mode")
  ros.loginfo("%s desired_poses_node is ready!", info_name)
  ros.spin()