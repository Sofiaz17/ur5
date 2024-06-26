/*!
    @file motion_planner.h
    @brief Functions declaration for ur5 motion planning
    @date   26/11/2023
    @author Federica Lorenzini
*/
#ifndef __MOTION_PLANNER_H__
#define __MOTION_PLANNER_H__
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry> //to use quaternions
#include <string.h>
#include <cmath>
#include "ros/ros.h"
using namespace Eigen;
using namespace std;

/*!
    @defgroup Motion_module MOTION
    @{
*/

typedef Matrix<double, 6, 6> Matrix6d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 8, 1> Vector8d;
typedef Matrix<double, 3, 1> Vector3d;
typedef Matrix<double, Dynamic, 8> Path;

const VectorXd A = (VectorXd(6) << 0, -0.425, -0.3922, 0, 0, 0).finished();                //!< Vector of A distance of standard UR5
const VectorXd D = (VectorXd(6) << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996).finished();       //!< Vector of D distance of standard UR5
const VectorXd ALPHA = (VectorXd(6) << M_PI / 2, 0, 0, M_PI / 2, -M_PI / 2, 0).finished(); //!< Vector of ALPHA angles of standard UR5
const VectorXd a = (VectorXd(6) << 0, -0.425, -0.3922, 0, 0, 0).finished();
const VectorXd d = (VectorXd(6) << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996).finished();
const VectorXd alpha = (VectorXd(6) << M_PI / 2, 0, 0, M_PI / 2, -M_PI / 2, 0).finished();

const double dt = 0.1;       //!< Time delta used in a single path unit
const double path_dt = 20.0; //!< Time duration of a single path unit

/*!
    @brief Function to check if a number is very close to zero.
    @details If the abs(number) is minor than 10^(-7), then it is approximated to 0-.
    @param[in] x: Number whose value needs to be checked.
    @return True if the number is minor than 10^(-7), false otherwise.
*/
bool almostZero(double x);

/*!
    @brief Function to print the matrix.
    @param[in] str: String to identify the matrix(label).
    @param[in] m: matrix to print.
    @return True if the number is minor than 10^(-7), false otherwise.
*/
int displayMatrix(string str, MatrixXd m);

/*!
    @brief Compute transformation matrix from i to j frame.
    @param[in] th: non-constant value. It's the angle about previous z from old x to new x.
    @param[in] alpha: constant value. It's the  angle about common normal, from old z axis to new z axis.
    @param[in] distance: constant value. Offset along previous z to the common normal
    @param[in] a: constant value. It's the length of the common normal.
    @return The transformation matrix from joint i to joint j. It's a 4x4 matrix.
*/
Matrix4d calculateTransMatrix(double th, double alpha, double distance, double a);

/*!
    @brief Compute transformation matrix from base to world frame.
    @details The transformation is given by a rotation of PI around x axis and a traslation of (0.5, 0.35, 1.75).
    @return The transformation matrix, it's a 4x4 matrix.
*/
Matrix4d base_to_world();

/*!
    @brief Compute transformation matrix to align gripper to world frame.
    @details The transformation is given by a rotation of PI/2 around z axis.
    @return The transformation matrix, it's a 4x4 matrix.
*/
Matrix4d alignGripper();

/*!
    @brief Function to compute Jacobian matrix
    @param[in] th: are the joint variables. Since we have six joints, th will be a 6 dimensions vector.
    @return the Jacobian matrix. It's a 6x6 matrix.
*/
Matrix6d calculateJacobian(Vector6d th);

/*!
    @brief Function to compute Jacobian matrix using cross product
    @param[in] th: are the joint variables. Since we have six joints, th will be a 6 dimensions vector.
    @return the Jacobian matrix. It's a 6x6 matrix.
*/
Matrix6d computeJacobianCross(Vector6d th);

/*!
    @brief Function to compute direct kinematics matrix.
    @param[in] Th: are the joint variables. Since we have six joints, th will be a 6 dimensions vector.
    @return the direct kinematics matrix. It's a 4x4 matrix.
*/
Matrix4d computeDirectKin(VectorXd Th);

/*!
    @brief Function to compute inverse matrix.
    @param[in] p60: are the joint variables. Since we have six joints, th will be a 6 dimensions vector.
    @param[in] R60: is the rotational matrix, extracted from direct kinematics matrix.
    @param[in] scaleFactor: scale factor.
    @return the possible joint variables, in a matrix form, for a giving position and orientation given in the direct kinematics matrix.
*/
Matrix<double, 6, 8> InverseKinematics(Vector3d p60, Matrix3d R60, double scaleFactor);

/*! 
    @brief To linearly interpolate two vectors in the space.
    @details We find an "interpolated" point, since it is in the space, it will be a 3D vector.
    @param[in] time: is the time between the two position.
    @param[in] p1: first point.
    @param[in] p2: second pint.
    @return the interpolated 3D point.
*/
Vector3d lerp(double time, Vector3d p1, Vector3d p2);

/*! 
    @brief To spherically interpolate two quaternions.
    @details We find an "interpolated" quaternion.
    @param[in] time: is the time between the two quaternions.
    @param[in] q1: first quaternion.
    @param[in] q2: second quaternion.
    @return the interpolated quaternion.
*/
Quaterniond getSlerp(double time, Quaterniond q1, Quaterniond q2);

/*! 
    @brief To insert a new path instance each time it's computed using inverse differential kinematics 
    @details Path is a matrix with dynamic-rows-number, hence each time a path instance 
                (a new set of joint variables) is compute in inverse differential, it's added to path.
    @param[in] p: is the path the robot has to follow. It consists in a matrix with 8 columns 
                    (6 for the joints + 2 for the gripper) and a dynamic number of rows.
    @param[in] js: joint variables.
    @param[in] gs: gripper state.
    @return a matrix with 8 columns and a variable number of rows.
*/
Path insert_new_path_instance(Path p, Vector6d js, Vector2d gs);

/*! 
    @brief To compute the path the robot has to follow to reach the final point, which is expressed using position and quaternion for orientation.
    @details The computation is performed by firstly adding the starting position to the path and then the following ones are added with an increment of dt=0.1s.
            Joint velocities are computed using the Damped Least Square (DLS) Inverse, to avoid problems in inverting differential kinematics near a singularities.
    @param[in] mr: are the theta angles of joints plus 2 variables to express gripper state.
    @param[in] f_p: first quaternion.
    @param[in] q2: second quaternion.
    @return a matrix with 8 columns and a variable number of rows.
*/
Path diffInverseKinQuaternions(Vector8d mr, Vector3d f_p, Quaterniond f_q);

#endif