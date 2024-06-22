#!/usr/bin/env python
import os
import sys
from pathlib import Path 

# ROOT = Path(__file__).resolve().parents[1]  # vision folder
# if str(ROOT) not in sys.path:
#     sys.path.append(str(ROOT))
# ROOT = Path(os.path.abspath(ROOT))
#sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

class DesiredPose3D:
  def __init__(self, label, x, y, z, theta):
    self.label = label
    self.x = x
    self.y = y
    self.z = z
    self.theta = theta

desired_poses = []
PI = 3.1415926535
x_offset = 0.5
y_offset = 0.2
z_offset = 0.87

# Set here your actual poses
#desired_poses.append(DesiredPose2D( label = "X1-Y1-Z2", x = -0.1, y = 0.2, theta = 0 ))
#desired_poses.append(DesiredPose2D( label = "X1-Y2-Z1", x = -0.1, y = 0.3, theta = PI/6 ))
#desired_poses.append(DesiredPose2D( label = "X1-Y2-Z2", x = -0.1, y = 0.4, theta = PI/4 ))
#desired_poses.append(DesiredPose2D( label = "X1-Y2-Z2-CHAMFER", x = -0.2, y = 0.2, theta = PI/3 ))
#desired_poses.append(DesiredPose2D( label = "X1-Y2-Z2-TWINFILLET", x = -0.2, y = 0.3, theta = PI/2 ))
#desired_poses.append(DesiredPose2D( label = "X1-Y3-Z2", x = -0.2, y = 0.4, theta = -PI/6 ))
#desired_poses.append(DesiredPose2D( label = "X1-Y3-Z2-FILLET", x = -0.3, y = 0.2, theta = -PI/4 ))
#desired_poses.append(DesiredPose2D( label = "X1-Y4-Z1", x = -0.3, y = 0.3, theta = -PI/3 ))
#desired_poses.append(DesiredPose2D( label = "X1-Y4-Z2", x = -0.3, y = 0.4, theta = -PI/2 ))
#desired_poses.append(DesiredPose2D( label = "X2-Y2-Z2", x = -0.4, y = 0.2, theta = PI ))
#desired_poses.append(DesiredPose2D( label = "X2-Y2-Z2-FILLET", x = -0.4, y = 0.3, theta = 0 ))

desired_poses.append(DesiredPose3D( label = "X1-Y2-Z2-TWINFILLET", x = -0.1, y = 0.2, z = z_offset, theta = PI ))
desired_poses.append(DesiredPose3D( label = "X1-Y4-Z2", x = -0.1, y = 0.35, z = z_offset, theta = PI/6 ))

# In absolute coordinates the table plane has Z = 0.87 and its limits are X in [0.0, 1.0] and Y in [0.2, 0.8].
# However, to simplify this file it will consider a frame centered in [0.5, 0.2, 0.87] with same orientation.
# Like this, in the following lines we can set only three parameters (as we reduced to a 2D domain), in the range:
# X in [-0.5, 0.5]
# Y in [0.0, 0.6]
# theta in [-PI, PI]

# Other scripts know of this convention and will easily reconvert the values to 3D
# It is required to have actual poses on the left of the table, i.e. positive X, while desired poses will be on the right, i.e. negative X.



# Lables of the blocks:
# "X1-Y1-Z2"
# "X1-Y2-Z1"
# "X1-Y2-Z2"
# "X1-Y2-Z2-CHAMFER"
# "X1-Y2-Z2-TWINFILLET"
# "X1-Y3-Z2"
# "X1-Y3-Z2-FILLET"
# "X1-Y4-Z1"
# "X1-Y4-Z2"
# "X2-Y2-Z2"
# "X2-Y2-Z2-FILLET"
