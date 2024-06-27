#include "motion_planner.h"
using namespace Eigen;
using namespace std;
// testing
int main()
{
    Matrix6d Jac;
    Matrix6d JacCrossProd;
    Matrix4d directMatrix, directMatrix2;
    Vector6d myJointVariables;
    Vector6d myvars;
    Vector3d pe60;
    Matrix3d Re;
    Matrix4d Tm;
    // testing with homing position variables
    myJointVariables << -0.0001, -0.00016, -0.00006, -0.00014, -0.00004, 0.00001;
    
    // PREMULTIPLY to express position in a FIXED frame and POSTMULTIPLY to adjust gripper
    directMatrix = base_to_world() * computeDirectKin(myJointVariables) * alignGripper();
    displayMatrix("Direct matrix with reference to WORLD frame and gripper adjusted\n", directMatrix);
    cout << endl;

 
    JacCrossProd=computeJacobianCross(myJointVariables);
    displayMatrix("\n\nJacobian matrix with cross product\n", JacCrossProd);


    // testing inverse differential kinematics
    Vector8d mr;          // Stato del manipolatore
    Vector3d i_p, f_p;    // Punto iniziale e finale della traiettoria
    Quaterniond i_q, f_q; // Orientazione iniziale e finale della traiettoria

    // dati di input (esempio)
    i_p << 0.73, 0.74, 1.26;
    f_p << 1, 1, 1;
    i_q.setIdentity();
    f_q.setIdentity();
    Vector8d robot_state;
    robot_state << myJointVariables, 0, 0;

    
}
