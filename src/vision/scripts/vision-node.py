#!/usr/bin/env python3

# @file vision-node.py
# @brief This script integrates block detection and pose estimation using ROS, OpenCV, Open3D, and other libraries.

# @defgroup Vision VISION
# @addtogroup Vision

# @mainpage Block Detection and Pose Estimation

# @section Overview
# This script integrates ROS (Robot Operating System) with computer vision techniques for block detection using the ZED camera.
# It performs:
# - Image processing to find regions of interest (ROIs) using OpenCV.
# - Block detection using a custom Block Detection module (`Block_detection.py`).
# - Point cloud processing and pose estimation using Open3D.

# @section Details
# The script subscribes to ROS topics for images (`/ur5/zed_node/left_raw/image_raw_color`) and point clouds
# (`/ur5/zed_node/point_cloud/cloud_registered`). Upon receiving an image, it:
# - Finds ROIs and detects blocks within the ROIs using `Block_detection.py`.
# - Generates a point cloud from detected blocks and estimates their poses relative to a fixed coordinate system.

# The script also uses Open3D for point cloud processing, including downsampling, feature extraction, and registration
# algorithms (RANSAC and ICP) to refine pose estimation.

# @section Dependencies
# - ROS (Kinetic or newer)
# - OpenCV
# - Open3D
# - NumPy
# - SciPy
# - CV Bridge (ROS)
# - Sensor_msgs (ROS)

# @section Directories
# - `images/`: Contains images used for ROI detection and visualization.

# @section Parameters
# - `R_cloud_to_world`: Transformation matrix from cloud to world coordinates.
# - `x_camera`: Camera position offset.
# - `base_offset`: Base offset of the block.
# - `block_offset`: Offset specific to each block.
# - `voxel_size`: Size parameter for downsampling point clouds.

# @section Flags
# - `debug`: Flag to enable debug visualizations.

# @section Classes
# - `Point`: Stores point coordinates and pixel locations.
# - `block`: ROS message type for block information.

# @section Functions
# - `get_img(img)`: Callback to process incoming images, detect ROIs, and detect blocks.
# - `find_pose(point_cloud)`: Callback to process point clouds, estimate block poses, and publish results.
# - Various utility functions for point cloud operations, mesh loading, transformation, and visualization.

# @section Main
# - `get_brick_pose_server()`: Main function to start the ROS node and service.



import rospy
from vision.msg import block
from pathlib import Path
import sys
import os
import rospkg
import copy
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import sensor_msgs.point_cloud2 as point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from vision.scripts import Block_detection as Lego
from  vision.scripts import Region_of_interest as roi
import math
from tf.transformations import quaternion_from_euler
import open3d as o3d
from scipy.spatial.transform import Rotation as R
import time
from ur5.srv import GetBrickPose, GetBrickPoseResponse
from geometry_msgs.msg import Pose
from ur5.msg import BlockParams
import threading

# --------------- DIRECTORIES ---------------
#ROOT = Path(__file__).resolve().parents[1]  # vision directory
ROOT = '/home/sofia_unix/catkin_ws/src/ur5/src/vision'
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))
ROOT = Path(os.path.abspath(ROOT))

ZED_IMG = str(ROOT) + '/images/img_ZED_cam.png'
ROI_IMG = str(ROOT) + '/images/ROI_table.png'
LINE_IMG = str(ROOT) + '/images/line-height.png'
bridge = CvBridge()

# --------------- WORLD PARAMS ---------------

R_cloud_to_world = np.matrix([[0, -0.49948, 0.86632], [-1., 0., 0.], [0., -0.86632, -0.49948]])
x_camera = np.array([-0.9, 0.24, -0.35])
base_offset = np.array([0.5, 0.35, 1.75])
block_offset = np.array([-0.011453, -0.005865, -0.019507])
voxel_size = 0.003


TABLE_OFFSET = 0.86 + 0.1

# --------------- FLAGS e GLOBALS ---------------
can_acquire_img = True
can_take_point_cloud = False
send_next_msg = True
block_list = []
measures = 0        # num of measures for world position of blocks
icp_threshold = 0.004

img_received = threading.Event()
pose_received = threading.Event()


# ---------------- DEBUG --------------------
debug = False
info_name = "[vision_node]:"

# print('REGION OF INTEREST DEBUG')
# print(dir(roi))
# print('REGION __FILE__')
# print(roi.__file__)
# print('BLOCK DETECTION DEBUG')
# print(dir(Lego))



class Point:
    # @Description Class to store points in the world space useful to estimate block pose
    # @Fields: coordinates x, y, z and pixels coordinates in ROI image

    def __init__(self):
        self.x = None
        self.y = None
        self.z = None
        self.px = ()

    def set(self, x, y, z, px):
        # @Description  Method to all parameters in once

        self.x = x
        self.y = y
        self.z = z
        self.px = px

    def info(self):
        # @Description Prints Point info
        print("x,y,z, PX: ", self.x, self.y, self.z, self.px)


def block_info(block):
    # @Description Prints Block info

    print("MESSAGE: ")
    print("\tlabel: " + str(block.label))
    print("\tx: " + str(block.pose.position.x))
    print("\ty: " + str(block.pose.position.y))
    print("\tz: " + str(block.pose.position.z))
    print("\tquaternion x: " + str(block.pose.orientation.x))
    print("\tquaternion y: " + str(block.pose.orientation.y))
    print("\tquaternion z: " + str(block.pose.orientation.z))
    print("\tquaternion w: " + str(block.pose.orientation.w))
    # print("\troll: " + str(block.roll))
    # print("\tpitch: " + str(block.pitch))
    # print("\tyaw: " + str(block.yaw))


def get_img(img):
    # @Description Callback function to store image from zed node and start detection process
    # @Parameters Image from Zed camera

    global can_acquire_img
    global can_take_point_cloud
    global block_list
    global img_received

    if can_acquire_img:
        print("GETTING IMAGE...")
        
        try:
            cv_obj = bridge.imgmsg_to_cv2(img, "bgr8")      # convert image to cv2 obj
        except CvBridgeError as e:
            print(e)

        cv.imwrite(ZED_IMG, cv_obj)

        # FIND ROI
        roi.find_roi(ZED_IMG)           # find ROI

        # Detection phase
        block_list = Lego.detection(ROI_IMG)    # get list of detected blocks

        # Copying ROI image to draw line for pose estimations
        img = cv.imread(ROI_IMG, cv.IMREAD_COLOR)
        cv.imwrite(LINE_IMG, img)

        # FLAG TO CHANGE
        can_acquire_img = False
        can_take_point_cloud = True
        img_received.set()
    #else:
        #print("Witing for img or not acquired yet [vision_node]")
        
        
def create_open3d_point_cloud(point_cloud_box):


    # Creating an Open3D point cloud object
    pcd = o3d.geometry.PointCloud()

    # Converting the list parameter in numpy array (Nx3)
    points = np.array(point_cloud_box)

    # Add points in list to point cloud
    pcd.points = o3d.utility.Vector3dVector(points)

    return pcd


# Visualization of the point cloud
def visualize_point_cloud(pcd):
    o3d.visualization.draw_geometries([pcd])


def load_mesh_model(block_label):
    # Adjust this path to the correct directory where your STL files are stored
    script_dir = os.path.dirname(os.path.abspath(__file__))
    stl_file_path = os.path.join(script_dir, 'models/'+block_label+'/mesh/'+block_label+'.stl')
    print(f"STL FILE PATH: {stl_file_path}")
    
    if not os.path.isfile(stl_file_path):
        raise FileNotFoundError(f"STL file not found: {stl_file_path}")
    
    mesh = o3d.io.read_triangle_mesh(stl_file_path)
    
    if mesh.is_empty():
        raise ValueError(f"Loaded mesh is empty: {stl_file_path}")
    
    return mesh.sample_points_poisson_disk(6138)



def execute_icp(source, target, threshold):
    # Setting convergence criteria
    icp_criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=5000)

    # ICP algorithm
    transformation_icp = o3d.pipelines.registration.registration_icp(
        source, target, threshold, np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        icp_criteria)

    return transformation_icp.transformation


def extract_rpy_from_transformation(transformation):
    # Extracting 3x3 rotation matix from the 4x4 one
    rotation_matrix = transformation[:3, :3].copy()

    # Use scipy to convert rotation matrix to RPY angles
    r = R.from_matrix(rotation_matrix)
    roll, pitch, yaw = r.as_euler('xyz')

    if roll > 3.14:
        roll = roll - 3.14
    if pitch > 3.14:
        pitch = pitch - 3.14
    if yaw > 3.14:
        yaw = yaw - 3.14

    return roll, pitch, yaw


def euler_to_rotation_matrix(euler_angles):
    """
    Calcola la matrice di rotazione a partire da angoli di Eulero.

    Args:
    - euler_angles (list or np.ndarray): Angoli di Eulero [x, y, z] in radianti.

    Returns:
    - numpy.ndarray: Matrice di rotazione 3x3.
    """
    # Estrai gli angoli di Eulero lungo gli assi x, y, z
    angle_x, angle_y, angle_z = euler_angles

    # Calcola le matrici di rotazione per ciascun asse
    rotation_x = np.array([[1, 0, 0],
                           [0, np.cos(angle_x), -np.sin(angle_x)],
                           [0, np.sin(angle_x), np.cos(angle_x)]])

    rotation_y = np.array([[np.cos(angle_y), 0, np.sin(angle_y)],
                           [0, 1, 0],
                           [-np.sin(angle_y), 0, np.cos(angle_y)]])

    rotation_z = np.array([[np.cos(angle_z), -np.sin(angle_z), 0],
                           [np.sin(angle_z), np.cos(angle_z), 0],
                           [0, 0, 1]])

    # Calcola la matrice di rotazione totale (rotazione intorno a z, poi y, poi x)
    rotation_matrix = rotation_z @ rotation_y @ rotation_x

    return rotation_matrix


def translate_point_cloud(pcd, vector):
    # Definisci l'asse di rotazione (per esempio attorno all'asse y)

    global base_offset

    translation_vector = vector

    # Inizializza una matrice identità 4x4
    translation_matrix = np.eye(4)

    # Imposta la parte di traslazione della matrice
    translation_matrix[0:3, 3] = translation_vector

    # Applica la trasformazione alla nuvola di punti
    pcd.transform(translation_matrix)

    return pcd


def draw_registration_result(source, target, transformation):
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.25, origin=[0, 0, 0])
    source_tmp = copy.deepcopy(source)
    target_tmp = copy.deepcopy(target)
    source_tmp.paint_uniform_color([1, 0.706, 0])
    target_tmp.paint_uniform_color([0, 0.651, 0.929])
    source_tmp.transform(transformation)
    o3d.visualization.draw_geometries([source_tmp, target_tmp, coordinate_frame])


def prepare_point_clouds(source, target, voxel_size):
    trans_init = np.array([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
                           [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    source.transform(trans_init)

    #target = correct_pc_orientation(target)
    target.transform(trans_init)

    #draw_registration_result(source, target, np.identity(4))
    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)

    return source, target, source_down, target_down, source_fpfh, target_fpfh


def preprocess_point_cloud(pcd, voxel_size):
    #downsampling with voxel size
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    pcd_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5

    #Computing FPFH feature with search radius feature
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(pcd_down,
                                                               o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature,
                                                                                                    max_nn=100))
    return pcd_down, pcd_fpfh


def execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print("EXECUTING RANSAC on downsampled point clouds")

    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True, distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 0.9999)
    )

    return result


def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size, result_ransac):
    distance_threshold = voxel_size * 0.4
    print("PERFORMING Point-to-plane ICP to refine the pose estimation")

    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, result_ransac.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane()
    )

    return result


def rotate_point_cloud(pcd, rotation_vector):
    """
    Ruota la nuvola di punti secondo il vettore di rotazione specificato attorno al proprio centro.

    Args:
        pcd (open3d.geometry.PointCloud): La nuvola di punti da ruotare.
        rotation_vector (tuple): Un vettore di 3 angoli in radianti (roll, pitch, yaw).

    Returns:
        open3d.geometry.PointCloud: La nuvola di punti ruotata.
    """
    # Calcola il centro della nuvola di punti
    center = pcd.get_center()

    # Crea la matrice di rotazione dalle componenti del vettore
    rotation_matrix = o3d.geometry.get_rotation_matrix_from_xyz(rotation_vector)

    # Trasla la nuvola di punti in modo che il centro coincida con l'origine
    pcd.translate(-center)

    # Applica la rotazione attorno all'origine (che ora è il centro della nuvola di punti)
    pcd.rotate(rotation_matrix)

    # Riporta la nuvola di punti alla posizione originale
    pcd.translate(center)

    return pcd


def rotate_point_cloud_about_axis(pcd, rotation_vector):
    # Crea la matrice di rotazione dalle componenti del vettore
    rotation_matrix = o3d.geometry.get_rotation_matrix_from_xyz(rotation_vector)

    # Applica la rotazione alla nuvola di punti
    pcd.rotate(rotation_matrix, center=(0, 0, 0))
    return pcd



# Operation to get and elaborate point cloud of blocks
def find_pose(point_cloud):
    # @Description Callback function to collect point cloud and estimate center and pose of each block detected
    # @Parameters point cloud from zed node

    global can_take_point_cloud
    global block_list
    global measures
    global icp_threshold
    global voxel_size
    global debug
    global pose_received

    if can_take_point_cloud:

        print("FINDING POSE...")

        if len(block_list) == 0:
            print("NO BLOCK DETECTED")
            sys.exit(1)

        # STEP 1 - Taking point clouds for each block
        for block in block_list:
            point_cloud_box = []
            for x in range(int(block.x1), int(block.x2)):
                for y in range(int(block.y1), int(block.y2)):
                        points = point_cloud2.read_points(point_cloud, field_names=['x', 'y', 'z'],
                                                          uvs=[(int(x), int(y))], skip_nans=True)
                        for point in points:
                            point_cloud_box.append(np.array(point))


            # STEP 2 - storing block point cloud
            target = create_open3d_point_cloud(point_cloud_box)

            # STEP 3 - Calculating center of the block leveraging center of the bounding box
            # computing world coordinates through a transformation matrix and correcting result adding offsets
            block.world_coord = R_cloud_to_world.dot(target.get_center()) + x_camera + base_offset + block_offset
            block.world_coord[0, 2] = 0.86999  # z of the block is a constant
            #print("WORLD:", block.world_coord)

            print(f"\033[34mExecuting block {block.label} \033[0m")

            # STEP 4 - Loading mesh model
            source = load_mesh_model(block.label)

            # STEP 5 - Applying a series transformation to match simulation origin frame (starting from camera POV)
            #[-0.02216, -0.40428 , 0.0211]
            target = rotate_point_cloud_about_axis(target, [np.pi - 0.02216, np.pi/2 - 0.40428, np.pi/2 + 0.0211])
            target = translate_point_cloud(target, [-0.4, 0.52, 1.4])
            #source = translate_point_cloud(source, [-0.4, 0.52, 1.4])

            #print("TARGET center: ", target.get_center())
            #draw_registration_result(source, target, np.identity(4))

            # STEP 6 - Preparing point clouds (downsampling)
            source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_point_clouds(source, target, voxel_size)

            # STEP 7 - Executing a global registration through RANSAC algorithm
            result_ransac = execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size)

            #print("RANSAC result: ", result_ransac)

            # STEP 8 - Refining registration using ICP algorithm
            result_icp = refine_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size, result_ransac)

            #print("ICP result: ", result_icp)

            if debug:       # show 3D situation after registration (useful to undestand results)
               draw_registration_result(source_down, target_down, result_icp.transformation)


            #draw_registration_result(source, target, transformation)

            #print("Transformation matrix:\n", transformation)

            # extarcting RPY angles from transformation matrx
            yaw, pitch, roll = extract_rpy_from_transformation(result_icp.transformation)


            # print("Roll: ", roll + 0.11)
            # print("Pitch: ", pitch )
            # print("Yaw: ", yaw )

            # Assigning block pose
            block.pose = [roll + 0.11, pitch, yaw]

        can_take_point_cloud = False

        # publishing messages
        #msg_pub(block_list)
        pose_received.set()
    #else:
        #print("Waiting for point cloud or not acquired yet [vision_node]")

def to_quaternions(r, p ,y):
    # @Description function transform RPY angles to Quaternions
    # @Parameters RPY angles
    # @Returns Quaternion

    return quaternion_from_euler(r, p, y)


def msg_pub(request):
    print('VISION in msg_pub')
    # @Description function that prepares and sends a message to motion node
    # @Parameters list of detected blocks

    #global send_next_msg
    #if send_next_msg:
    global block_list

    
        # Preparing msg
    response = GetBrickPoseResponse()
    for current_block in block_list:
        bp = BlockParams()
        q = to_quaternions(current_block.pose[0], current_block.pose[1],  current_block.pose[2])
        #labels.append(current_block.label)
        #bp.labels = labels
        bp.label = current_block.label
        print(f"VISION label:{bp.label}")
        bp.pose.position.x = round(current_block.world_coord[0, 0], 6)
        bp.pose.position.y = round(current_block.world_coord[0, 1], 6)
        bp.pose.position.z = round(current_block.world_coord[0, 2], 6)
        
        bp.pose.orientation.x = q[0]
        bp.pose.orientation.y = q[1]
        bp.pose.orientation.z = q[2]
        bp.pose.orientation.w = q[3]

        block_info(bp)    # print bp info
        #poses.append(pose)
        response.poses.append(bp)

  
    #response.poses.append(bp)
    #response.label = labels
    response.numBricks = len(block_list)

    
    return response


def get_brick_pose_server():
    
    rospy.init_node('vision_node')

    # Subscribers
    img_sub = rospy.Subscriber("/ur5/zed_node/left_raw/image_raw_color", Image, get_img)
    point_cloud_sub = rospy.Subscriber("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2, find_pose, queue_size=1)

    rospy.loginfo("Waiting for image and pose to be received...")
    img_received.wait()  # Aspetta che l'immagine venga ricevuta
    pose_received.wait()  # Aspetta che la pose venga trovata

    rospy.loginfo("Both image and pose have been received. Starting service...")

    service = rospy.Service("get_brick_pose", GetBrickPose, msg_pub)
    rospy.spin()


# def msg_pub(req):
#     # Funzione per il servizio
#     # ...
#     return GetBrickPoseResponse()

# ----------------------------------------------- MAIN -----------------------------------------------


if __name__ == '__main__':
    get_brick_pose_server()