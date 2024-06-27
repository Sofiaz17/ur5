#include "motion_planner.h"
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry> //to use quaternions
#include <string.h>
#include <cmath>
#include <stdexcept>
using namespace Eigen;
using namespace std;

/*!
    @addtogroup Motion_module
    @{
*/

/**
 * @brief Calculates the homogeneous transformation matrix for a given DH parameters.
 * @param th Joint angle
 * @param alpha Link twist angle
 * @param distance Link offset
 * @param a Link length
 * @return Homogeneous transformation matrix
 */
Matrix4d calculateTransMatrix(double th, double alpha, double distance, double a)
{
    Matrix4d Tij;
    Tij << cos(th), -sin(th) * cos(alpha), sin(th) * sin(alpha), a * cos(th),
        sin(th), cos(th) * cos(alpha), -cos(th) * sin(alpha), a * sin(th),
        0, sin(alpha), cos(alpha), distance,
        0, 0, 0, 1;
    return Tij;
}

/**
 * @brief Computes the base-to-world transformation matrix.
 * @return Base-to-world transformation matrix
 */
Matrix4d base_to_world()
{
    Matrix4d roto_trasl_matrix;
    roto_trasl_matrix << 1, 0, 0, 0.5,
                        0, -1, 0, 0.35,
                        0, 0, -1, 1.75,
                        0, 0, 0, 1;
    return roto_trasl_matrix;
};

/**
 * @brief Aligns the gripper with a specific orientation.
 * @return Gripper alignment rotation matrix
 */
Matrix4d alignGripper()
{
    Matrix4d rotationMatrix;
    rotationMatrix << 0, -1, 0, 0,
                      1, 0, 0, 0,
                      0, 0, 1, 0,
                      0, 0, 0, 1;
    return rotationMatrix;
};

/**
 * @brief Checks if a double value is almost zero.
 * @param x Value to check
 * @return True if the value is almost zero, false otherwise
 */
bool almostZero(double x)
{
    return (abs(x) < 1e-7);
}

/**
 * @brief Displays a matrix with a specified string label.
 * @param str Label for the matrix
 * @param m Matrix to display
 * @return Always returns 0
 */
int displayMatrix(string str, MatrixXd m)
{
    cout << str << endl
         << m << endl;
    return 0;
}


/**
 * @brief Computes the direct kinematics transformation matrix.
 * @param Th Vector of joint angles
 * @return Direct kinematics transformation matrix
 */
Eigen::Matrix4d computeDirectKin(VectorXd Th)
{
    Matrix4d TransfMatrix;
    // compute single transformations
    Matrix4d T10 = calculateTransMatrix(Th(0), ALPHA(0), D(0), A(0));
    Matrix4d T21 = calculateTransMatrix(Th(1), ALPHA(1), D(1), A(1));
    Matrix4d T32 = calculateTransMatrix(Th(2), ALPHA(2), D(2), A(2));
    Matrix4d T43 = calculateTransMatrix(Th(3), ALPHA(3), D(3), A(3));
    Matrix4d T54 = calculateTransMatrix(Th(4), ALPHA(4), D(4), A(4));
    Matrix4d T65 = calculateTransMatrix(Th(5), ALPHA(5), D(5), A(5));
    // compute transformation matrix from 0 to 6(T60=Tm)
    return TransfMatrix = (T10) * (T21) * (T32) * (T43) * (T54) * (T65);
    // pe is the end effector postion: it's coincides with the first 3 rows of the last column( <3,1> vector) of Tm
    // pe = Tm.block<3, 1>(0, 3); // extract from row 0 and column 3 (so 4th column) a vector of 3x1 elements
    // Re is the rotation matrix-->3x3--> first 3 rows and first 3 columns of Tm
    // Re = Tm.block<3, 3>(0, 0);
}


/**
 * @brief Perform linear interpolation (LERP) between two 3D vectors.
 *
 * This function computes a linear interpolation (LERP) between two 3D vectors
 * `p1` and `p2` based on a given time parameter `time` and a predefined path time `path_dt`.
 *
 * @param time Current time value.
 * @param p1 Starting 3D vector.
 * @param p2 Ending 3D vector.
 * @return Eigen::Vector3d Interpolated vector.
 *
 * @note The function normalizes `time` with respect to `path_dt` to compute the interpolation.
 *       If `time` exceeds `path_dt`, the function returns `p2`.
 */
Vector3d lerp(double time, Vector3d p1, Vector3d p2)
{
    /*since interpolation evaluates which weight must be given to each vectors,
    we need to normalize the time with reference to path_dt*/
    const double normalizedTime = time / path_dt;

    if (normalizedTime < 1)
        // for position we need to use linear interpolation (lerp), that's why the use of quaternion is preferred
        return (normalizedTime * p2) + ((1 - normalizedTime) * p1);
    else
        return p2; // zero weight is given to p1, since time exceed our delta
}

/**
 * @brief Perform spherical linear interpolation (SLERP) between two quaternions.
 *
 * This function computes a spherical linear interpolation (SLERP) between two quaternions
 * `q1` and `q2` based on a given time parameter `time` and a predefined path time `path_dt`.
 *
 * @param time Current time value.
 * @param q1 Starting quaternion.
 * @param q2 Ending quaternion.
 * @return Eigen::Quaterniond Interpolated quaternion.
 *
 * @note The function normalizes `time` with respect to `path_dt` to compute the interpolation.
 *       If `time` exceeds `path_dt`, the function returns `q2`.
 */
Quaterniond getSlerp(double time, Quaterniond q1, Quaterniond q2)
{
    q1.normalize();
    q2.normalize();
    const double normalizedTime = time / path_dt;
    if (normalizedTime < 1)
        /*Returns the spherical linear interpolation between the two quaternions
         *this and other at the parameter t in [0;1]*/
        return (q1.slerp(normalizedTime, q2));
    else
        return q2;
}

/**
 * @brief Compute the geometric Jacobian matrix using the cross-product method.
 *
 * This function computes the geometric Jacobian matrix for a robotic manipulator
 * based on the joint angles `Th`.
 *
 * @param Th Vector of joint angles (size 6).
 * @return Eigen::Matrix<double, 6, 6> Geometric Jacobian matrix (6x6).
 *
 * @note This function assumes the following:
 *       - `base_to_world()` computes the transformation matrix from the base frame to the world frame.
 *       - `calculateTransMatrix(theta, alpha, d, a)` computes the transformation matrix for a given DH parameters.
 *       - `alignGripper()` adjusts the transformation matrix for the gripper, if applicable.
 *       - Joint angles `Th` are expected in radians.
 *
 * @note The function computes various transformation matrices T10 to T65, constructs
 *       vectors z0 to z5 and p0 to p6, and then constructs the geometric Jacobian matrix
 *       using the cross-product method.
 */
Matrix6d computeJacobianCross(Vector6d Th)
{
    Matrix4d T10 = base_to_world() * calculateTransMatrix(Th(0), ALPHA(0), D(0), A(0));
    Matrix4d T21 = calculateTransMatrix(Th(1), ALPHA(1), D(1), A(1));
    Matrix4d T32 = calculateTransMatrix(Th(2), ALPHA(2), D(2), A(2));
    Matrix4d T43 = calculateTransMatrix(Th(3), ALPHA(3), D(3), A(3));
    Matrix4d T54 = calculateTransMatrix(Th(4), ALPHA(4), D(4), A(4));
    Matrix4d T65 = calculateTransMatrix(Th(5), ALPHA(5), D(5), A(5));
    Matrix4d transMatrix20;
    transMatrix20 = T10 * T21;
    Matrix4d transMatrix30;
    transMatrix30 = transMatrix20 * T32;
    Matrix4d transMatrix40;
    transMatrix40 = transMatrix30 * T43;
    Matrix4d transMatrix50;
    transMatrix50 = transMatrix40 * T54;
    Matrix4d transMatrix60;
    transMatrix60 = transMatrix50 * T65 * alignGripper();
    Vector3d z0, z1, z2, z3, z4, z5;
    z0 << 0, 0, -1;
    z1 = T10.col(2).head(3);
    z2 = transMatrix20.col(2).head(3);
    z3 = transMatrix30.col(2).head(3);
    z4 = transMatrix40.col(2).head(3);
    z5 = transMatrix50.col(2).head(3);
    Vector3d p0, p1, p2, p3, p4, p5, p6;
    p0 << 0.5, 0.35, 1.75;
    p1 = T10.col(3).head(3);
    p2 = transMatrix20.col(3).head(3);
    p3 = transMatrix30.col(3).head(3);
    p4 = transMatrix40.col(3).head(3);
    p5 = transMatrix50.col(3).head(3);
    p6 = transMatrix60.col(3).head(3);
    Matrix6d geomJacobian;
    geomJacobian << z0.cross(p6 - p0), z1.cross(p6 - p1), z2.cross(p6 - p2), z3.cross(p6 - p3), z4.cross(p6 - p4), z5.cross(p6 - p5),
        z0, z1, z2, z3, z4, z5;
    return geomJacobian;
}



/**
 * @brief Perform differential inverse kinematics using quaternions for a robotic path.
 *
 * This function performs differential inverse kinematics (DIK) using quaternions to compute
 * the robot's path given motion constraints and initial/final positions.
 *
 * @param mr Vector of motion constraints and initial joint values (size 8).
 * @param f_p Final 3D position.
 * @param f_q Final quaternion orientation.
 * @return Path Computed robotic path.
 *
 * @note This function computes the robot's path based on the given motion constraints `mr`,
 *       final position `f_p`, and final orientation `f_q`.
 *       It uses differential kinematics, Jacobian computation, and PID-like correction to
 *       generate intermediate joint states and update the path accordingly.
 */
Path diffInverseKinQuaternions(Vector8d mr, Vector3d f_p, Quaterniond f_q)
{
    // gs is the gripper actual opening
    // js_k and ks_k_dot are joints values in the instant k and its derivative dot in the same insatnt
    Vector2d gs{mr(6), mr(7)};
    Vector6d js_k, js_dot_k;

    // angular and positional velocities combined with the correction error
    Vector6d fv;

    //path of the robot
	Path path;

	//initial transformation matrix
    Matrix4d i_tm;
	
    //transformation matrix in the instant k 
    Matrix4d tm_k;
	
	//initial position of the robot
    Vector3d i_p;
	
    //position of the robot in the instant k
    Vector3d p_k;

	// initial rotation matrix of the robot
    Matrix3d i_rm;
	
    // rotation matrix of the robot in the instant k
    Matrix3d rm_k;
	
	//quaternion related to the rotational matrix of the robot in the instant k
    Quaterniond i_q;
	
    //quaternion related to the rotational matrix of the robot in the instant k
    Quaterniond q_k;

    // angular and positional velocities of the robot in the instant k
    Vector3d av_k, pv_k;

    // quaternion velocity related to the angular velocity of the robot in the instant k
    Quaterniond qv_k;

    // quaternion error of the rotational path (slerp) of the robot
    Quaterniond qerr_k;

    // positional error of the linear path (x) of the robot
    Vector3d perr_k;

    // geometric jacobian and inverse geometric jacobian of the robot in the instant k
    Matrix6d j_k, invj_k;

    //matrices for correction
    Matrix3d Kp;
	Matrix3d Kq;
	Matrix6d Kjac;
    Kp = Matrix3d::Identity() * 0.01; //10
    Kq = Matrix3d::Identity() * 0.01; //1
	Kjac = Matrix6d::Identity() * 0.0001;
	
	//extract initial position and orientation
	i_tm = base_to_world() * computeDirectKin(mr.head(6)) * alignGripper();
	i_p = i_tm.block(0, 3, 3, 1);
	i_rm = i_tm.block(0, 0, 3, 3);
	i_q	= i_rm;
	
    //insert the starting point to the path
    for (int i = 0; i < 6; ++i) js_k(i) = mr(i);
    path = insert_new_path_instance(path, js_k, gs);
    // compute the direct kinematics in the instant k
    tm_k = base_to_world() * computeDirectKin(js_k) * alignGripper();
    p_k = tm_k.block(0, 3, 3, 1);
    rm_k = tm_k.block(0, 0, 3, 3);
    q_k = rm_k;

    std::cout << "starting p: " << p_k.transpose() << " actual q: " << q_k.coeffs().transpose() << std::endl;

    //each delta time (dt) compute the joints state to insert into the path 
    for (double t = 0; t < path_dt; t += dt) //for (double t = dt; t < path_dt; t += dt)
    {
//			std::cout << "##### t = " << t << " #####" << std::endl;
        //compute the direct kinematics in the instant k 
        tm_k = base_to_world() * computeDirectKin(js_k) * alignGripper();
        p_k = tm_k.block(0, 3, 3, 1);
        rm_k = tm_k.block(0, 0, 3, 3);
        q_k = rm_k;
		
//		std::cout << "actual p: " << p_k.transpose() << " actual q: " << q_k.coeffs().transpose() << std::endl;
		
		//compute the velocities in the instant k
        pv_k = (lerp(t + dt, i_p, f_p) - lerp(t, i_p, f_p)) / dt; //pv_k = (lerp(t, i_p, f_p) - lerp(t - dt, i_p, f_p)) / dt;
        qv_k = getSlerp(t + dt, i_q, f_q) * getSlerp(t, i_q, f_q).conjugate(); //qv_k = myslerp(t, i_q, f_q) * myslerp(t - dt, i_q, f_q).conjugate();
	    av_k = (qv_k.vec() * 2) / dt;
		
//			std::cout << "pv_k: " << pv_k.transpose() << " qv_k: " << qv_k.coeffs().transpose() << " av_k: " << av_k.transpose() << std::endl;
		
        //compute the jacobian and its inverse in the instant k
        j_k = computeJacobianCross(js_k);
        //invj_k = (j_k.transpose() * j_k + Matrix6d::Identity() * 0.0001).inverse() * j_k.transpose();
		invj_k = j_k.transpose() * (j_k * j_k.transpose() + Kjac).inverse();
        if (abs(j_k.determinant()) < 0.00001)
        {
            ROS_WARN("Near singular configuration");
        }
		
        //compute the errors in the path
		qerr_k = getSlerp(t, i_q, f_q) * q_k.conjugate();
			///qerr_k = Kq * qerr_k.coeffs();
        perr_k = (lerp(t, i_p, f_p) - p_k) / dt; //perr_k = lerp(t, i_p, f_p) - p_k;
        
        
        //compute the vector of the velocities composition with a parameter of correction
			///Quaterniond total_q;
			///total_q = qv_k * qerr_k;
			Vector3d aerr_k;
			aerr_k = (qerr_k.vec() * 2) / dt;
            fv << pv_k + (Kp * perr_k), av_k + (Kq * aerr_k); //fv << pv_k + (Kp * perr_k), av_k + (Kq * qerr_k.vec());
		
//			std::cout << "perr_k: " << perr_k.transpose() << " qerr_k: " << qerr_k.coeffs().transpose() << std::endl;
//			std::cout << "fv: " << fv.transpose() << std::endl;

        // compute the joints state in the instant k
        js_dot_k = invj_k * fv;
        js_k = js_k + (js_dot_k * dt);
		
//			std::cout << "js_dot_k: " << js_dot_k.transpose() << std::endl << std::endl;
			
//			std::cout << "target: " << lerp(t + dt, i_p, f_p).transpose() << std::endl << std::endl;

        //add it to the path
        path = insert_new_path_instance(path, js_k, gs);
    }

    return path;
}


/**
 * @brief Insert a new instance of joint values and gripper state into the Path data structure.
 *
 * This function inserts a new instance of joint values and gripper state into the existing
 * Path data structure `p`. It increments the size of `p` by one row and inserts the values
 * `js` (joint values) and `gs` (gripper state) into the new row.
 *
 * @param p Path data structure to which the new instance will be inserted.
 * @param js Vector of joint values (size 6).
 * @param gs Vector of gripper state values (size 2).
 * @return Path Updated Path data structure with the new instance.
 *
 * @note This function modifies the input Path `p` by adding a new row at the end,
 *       containing the provided joint values `js` and gripper state `gs`.
 */
Path insert_new_path_instance(Path p, Vector6d js, Vector2d gs)
{
    // Incrementa le dimensioni della matrice di una riga
    p.conservativeResize(p.rows() + 1, p.cols());

    // Inserisci i valori delle giunture e dello stato della pinza nella nuova riga
    p.row(p.rows() - 1) << js(0), js(1), js(2), js(3), js(4), js(5), gs(0), gs(1);

    return p;
}

/*! @} */
