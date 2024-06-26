
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


bool almostZero(double x);

int displayMatrix(string str, MatrixXd m);

Matrix4d calculateTransMatrix(double th, double alpha, double distance, double a);

Matrix4d base_to_world();

Matrix4d alignGripper();

Matrix6d calculateJacobian(Vector6d th);

Matrix6d computeJacobianCross(Vector6d th);

Matrix4d computeDirectKin(VectorXd Th);

Matrix<double, 6, 8> InverseKinematics(Vector3d p60, Matrix3d R60, double scaleFactor);

Vector3d lerp(double time, Vector3d p1, Vector3d p2);

Quaterniond getSlerp(double time, Quaterniond q1, Quaterniond q2);

Path insert_new_path_instance(Path p, Vector6d js, Vector2d gs);

Path diffInverseKinQuaternions(Vector8d mr, Vector3d f_p, Quaterniond f_q);

/*! @} */

#endif