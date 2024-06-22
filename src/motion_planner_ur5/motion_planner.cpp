
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

Matrix4d computeTransformationMatrix(double th, double alpha, double distance, double a)
{
    Matrix4d Tij;
    Tij << cos(th), -sin(th) * cos(alpha), sin(th) * sin(alpha), a * cos(th),
        sin(th), cos(th) * cos(alpha), -cos(th) * sin(alpha), a * sin(th),
        0, sin(alpha), cos(alpha), distance,
        0, 0, 0, 1;
    return Tij;
}

Matrix4d base_to_world()
{
    Matrix4d roto_trasl_matrix;
    roto_trasl_matrix << 1, 0, 0, 0.5,
        0, -1, 0, 0.35,
        0, 0, -1, 1.75,
        0, 0, 0, 1;
    return roto_trasl_matrix;
};

Matrix4d adjust_gripper()
{
    Matrix4d roto_matrix;
    roto_matrix << 0, -1, 0, 0,
        1, 0, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    return roto_matrix;
};

bool almostZero(double x)
{
    return (abs(x) < 1e-7);
}

int print_eigen(string str, MatrixXd m)
{
    cout << str << endl
         << m << endl;
    return 0;
}

Matrix6d computeJacobian(Vector6d th)
{
    Matrix6d jacobianMatrix;
    Vector6d J1(6, 1); // first column of the jacobian matrix
    J1 << D(4) * (cos(th(0)) * cos(th(4)) + cos(th(1) + th(2) + th(3)) * sin(th(0)) * sin(th(4))) + D(3) * cos(th(0)) - A(1) * cos(th(1)) * sin(th(0)) - D(4) * sin(th(1) + th(2) + th(3)) * sin(th(0)) - A(2) * cos(th(1)) * cos(th(2)) * sin(th(0)) + A(2) * sin(th(0)) * sin(th(1)) * sin(th(2)),
        D(4) * (cos(th(4)) * sin(th(0)) - cos(th(1) + th(2) + th(3)) * cos(th(0)) * sin(th(4))) + D(3) * sin(th(0)) + A(1) * cos(th(0)) * cos(th(1)) + D(4) * sin(th(1) + th(2) + th(3)) * cos(th(0)) + A(2) * cos(th(0)) * cos(th(1)) * cos(th(2)) - A(2) * cos(th(0)) * sin(th(1)) * sin(th(2)),
        0,
        0,
        0,
        1;
    Vector6d J2(6, 1); // second column of the jacobian matrix
    J2 << -cos(th(0)) * (A(2) * sin(th(1) + th(2)) + A(1) * sin(th(1)) + D(4) * (sin(th(1) + th(2)) * sin(th(3)) - cos(th(1) + th(2)) * cos(th(3))) - D(4) * sin(th(4)) * (cos(th(1) + th(2)) * sin(th(3)) + sin(th(1) + th(2)) * cos(th(3)))),
        -sin(th(0)) * (A(2) * sin(th(1) + th(2)) + A(1) * sin(th(1)) + D(4) * (sin(th(1) + th(2)) * sin(th(3)) - cos(th(1) + th(2)) * cos(th(3))) - D(4) * sin(th(4)) * (cos(th(1) + th(2)) * sin(th(3)) + sin(th(1) + th(2)) * cos(th(3)))),
        A(2) * cos(th(1) + th(2)) - (D(4) * sin(th(1) + th(2) + th(3) + th(4))) / 2 + A(1) * cos(th(1)) + (D(4) * sin(th(1) + th(2) + th(3) - th(4))) / 2 + D(4) * sin(th(1) + th(2) + th(3)),
        sin(th(0)),
        -cos(th(0)),
        0;
    Vector6d J3(6, 1); // third column of the Jacobian matrix
    J3 << cos(th(0)) * (D(4) * cos(th(1) + th(2) + th(3)) - A(2) * sin(th(1) + th(2)) + D(4) * sin(th(1) + th(2) + th(3)) * sin(th(4))),
        sin(th(0)) * (D(4) * cos(th(1) + th(2) + th(3)) - A(2) * sin(th(1) + th(2)) + D(4) * sin(th(1) + th(2) + th(3)) * sin(th(4))),
        A(2) * cos(th(1) + th(2)) - (D(4) * sin(th(1) + th(2) + th(3) + th(4))) / 2 + (D(4) * sin(th(1) + th(2) + th(3) - th(4))) / 2 + D(4) * sin(th(1) + th(2) + th(3)),
        sin(th(0)),
        -cos(th(0)),
        0;
    Vector6d J4(6, 1); // fourth column of the Jacobian matrix
    J4 << D(4) * cos(th(0)) * (cos(th(1) + th(2) + th(3)) + sin(th(1) + th(2) + th(3)) * sin(th(4))),
        D(4) * sin(th(0)) * (cos(th(1) + th(2) + th(3)) + sin(th(1) + th(2) + th(3)) * sin(th(4))),
        D(4) * (sin(th(1) + th(2) + th(3) - th(4)) / 2 + sin(th(1) + th(2) + th(3)) - sin(th(1) + th(2) + th(3) + th(4)) / 2),
        sin(th(0)),
        -cos(th(0)),
        0;
    Vector6d J5(6, 1); // fifth column of the jacobian matrix
    J5 << D(4) * cos(th(0)) * cos(th(1)) * cos(th(4)) * sin(th(2)) * sin(th(3)) - D(4) * cos(th(0)) * cos(th(1)) * cos(th(2)) * cos(th(3)) * cos(th(4)) - D(4) * sin(th(0)) * sin(th(4)) + D(4) * cos(th(0)) * cos(th(2)) * cos(th(4)) * sin(th(1)) * sin(th(3)) + D(4) * cos(th(0)) * cos(th(3)) * cos(th(4)) * sin(th(1)) * sin(th(2)),
        D(4) * cos(th(0)) * sin(th(4)) + D(4) * cos(th(1)) * cos(th(4)) * sin(th(0)) * sin(th(2)) * sin(th(3)) + D(4) * cos(th(2)) * cos(th(4)) * sin(th(0)) * sin(th(1)) * sin(th(3)) + D(4) * cos(th(3)) * cos(th(4)) * sin(th(0)) * sin(th(1)) * sin(th(2)) - D(4) * cos(th(1)) * cos(th(2)) * cos(th(3)) * cos(th(4)) * sin(th(0)),
        -D(4) * (sin(th(1) + th(2) + th(3) - th(4)) / 2 + sin(th(1) + th(2) + th(3) + th(4)) / 2),
        sin(th(1) + th(2) + th(3)) * cos(th(0)),
        sin(th(1) + th(2) + th(3)) * sin(th(0)),
        -cos(th(1) + th(2) + th(3));
    Vector6d J6(6, 1); // sixth column of the jacobian matrix
    J6 << 0,
        0,
        0,
        cos(th(4)) * sin(th(0)) - cos(th(1) + th(2) + th(3)) * cos(th(0)) * sin(th(4)),
        -cos(th(0)) * cos(th(4)) - cos(th(1) + th(2) + th(3)) * sin(th(0)) * sin(th(4)),
        -sin(th(1) + th(2) + th(3)) * sin(th(4));

    // to obtain the jacobian matrix we just need to put in jacobianMatrix the above six vectors.
    jacobianMatrix << J1, J2, J3, J4, J5, J6;
    return jacobianMatrix;
}

Eigen::Matrix4d directKin(VectorXd Th)
{
    Matrix4d TransformationMatrix;
    // compute single transformations
    Matrix4d T10 = computeTransformationMatrix(Th(0), ALPHA(0), D(0), A(0));
    Matrix4d T21 = computeTransformationMatrix(Th(1), ALPHA(1), D(1), A(1));
    Matrix4d T32 = computeTransformationMatrix(Th(2), ALPHA(2), D(2), A(2));
    Matrix4d T43 = computeTransformationMatrix(Th(3), ALPHA(3), D(3), A(3));
    Matrix4d T54 = computeTransformationMatrix(Th(4), ALPHA(4), D(4), A(4));
    Matrix4d T65 = computeTransformationMatrix(Th(5), ALPHA(5), D(5), A(5));
    // compute transformation matrix from 0 to 6(T60=Tm)
    return TransformationMatrix = (T10) * (T21) * (T32) * (T43) * (T54) * (T65);
    // pe is the end effector postion: it's coincides with the first 3 rows of the last column( <3,1> vector) of Tm
    // pe = Tm.block<3, 1>(0, 3); // extract from row 0 and column 3 (so 4th column) a vector of 3x1 elements
    // Re is the rotation matrix-->3x3--> first 3 rows and first 3 columns of Tm
    // Re = Tm.block<3, 3>(0, 0);
}

Matrix<double, 6, 8> inverseKin(Vector3d p60, Matrix3d R60, double scaleFactor)
{
    // Vector of the a distance (expressed in meters)
    VectorXd A_Distance(6);
    A_Distance = A * scaleFactor;

    // Vector of the D distance (expressed in meters)
    VectorXd distance(6);

    distance = D * scaleFactor;

    // Anonymous function for the computation of the transformation matrix (general form)
    Matrix4d T60 = Matrix4d::Zero();
    T60.block<3, 3>(0, 0) = R60;
    T60.col(3).head(3) = p60;

    // Finding th1
    auto p50 = T60 * Vector4d(0, 0, -D(5), 1);

    auto psi = atan2(p50(1), p50(0));
    auto p50xy = p50.head(2).norm();
    if (p50xy < D(3))
    {
        cout << "Position request in the unreachable cylinder" << endl;
        exit;
    }
    auto phi1_1 = acos(D(3) / p50xy);
    auto phi1_2 = -phi1_1;

    auto th1_1 = psi + phi1_1 + M_PI / 2;
    auto th1_2 = psi + phi1_2 + M_PI / 2;

    auto p61z_1 = p60(0) * sin(th1_1) - p60(1) * cos(th1_1);
    auto p61z_2 = p60(0) * sin(th1_2) - p60(1) * cos(th1_2);

    auto th5_1_1 = acos((p61z_1 - D(3)) / D(5));
    auto th5_1_2 = -acos((p61z_1 - D(3)) / D(5));
    auto th5_2_1 = acos((p61z_2 - D(3)) / D(5));
    auto th5_2_2 = -acos((p61z_2 - D(3)) / D(5));
    Matrix4d T10_1 = computeTransformationMatrix(th1_1, ALPHA(0), D(0), A(0));
    Matrix4d T10_2 = computeTransformationMatrix(th1_2, ALPHA(0), D(0), A(0));

    Matrix4d T16_1 = ((T10_1.inverse()) * T60).inverse();
    Matrix4d T16_2 = ((T10_2.inverse()) * T60).inverse();

    double zy_1 = T16_1(1, 2);
    double zx_1 = T16_1(0, 2);

    double zy_2 = T16_2(1, 2);
    double zx_2 = T16_2(0, 2);
    double th6_1_1;
    double th6_1_2;
    double th6_2_1;
    double th6_2_2;

    if (almostZero(sin(th5_1_1)) || (almostZero(zy_1) && almostZero(zx_1)))
    {
        cout << "singular configuration. Choosing arbitrary th6" << endl;
        th6_1_1 = 0;
    }
    else
    {
        th6_1_1 = atan2((-zy_1 / sin(th5_1_1)), (zx_1 / sin(th5_1_1)));
    }

    if (almostZero(sin(th5_1_2)) || (almostZero(zy_1) && almostZero(zx_1)))
    {
        cout << "singular configuration. Choosing arbitrary th6" << endl;
        th6_1_2 = 0;
    }
    else
    {
        th6_1_2 = atan2((-zy_1 / sin(th5_1_2)), (zx_1 / sin(th5_1_2)));
    }

    if (almostZero(sin(th5_2_1)) || (almostZero(zy_2) && almostZero(zx_2)))
    {
        cout << "singular configuration. Choosing arbitrary th6" << endl;
        th6_2_1 = 0;
    }
    else
    {
        th6_2_1 = atan2((-zy_2 / sin(th5_2_1)), (zx_2 / sin(th5_2_1)));
    }

    if (almostZero(sin(th5_2_2)) || (almostZero(zy_2) && almostZero(zx_2)))
    {
        cout << "singular configuration. Choosing arbitrary th6" << endl;
        th6_2_2 = 0;
    }
    else
    {
        th6_2_2 = atan2((-zy_2 / sin(th5_2_2)), (zx_2 / sin(th5_2_2)));
    }

    Matrix4d T61_1 = T16_1.inverse();
    Matrix4d T61_2 = T16_2.inverse();

    Matrix4d T54_1_1 = computeTransformationMatrix(th5_1_1, ALPHA(4), D(4), A(4));
    Matrix4d T54_1_2 = computeTransformationMatrix(th5_1_2, ALPHA(4), D(4), A(4));
    Matrix4d T54_2_1 = computeTransformationMatrix(th5_2_1, ALPHA(4), D(4), A(4));
    Matrix4d T54_2_2 = computeTransformationMatrix(th5_2_2, ALPHA(4), D(4), A(4));

    Matrix4d T65_1_1 = computeTransformationMatrix(th6_1_1, ALPHA(5), D(5), A(5));
    Matrix4d T65_1_2 = computeTransformationMatrix(th6_1_2, ALPHA(5), D(5), A(5));
    Matrix4d T65_2_1 = computeTransformationMatrix(th6_2_1, ALPHA(5), D(5), A(5));
    Matrix4d T65_2_2 = computeTransformationMatrix(th6_2_2, ALPHA(5), D(5), A(5));

    Matrix4d T41_1_1 = T61_1 * (T54_1_1 * T65_1_1).inverse();
    Matrix4d T41_1_2 = T61_1 * (T54_1_2 * T65_1_2).inverse();
    Matrix4d T41_2_1 = T61_2 * (T54_2_1 * T65_2_1).inverse();
    Matrix4d T41_2_2 = T61_2 * (T54_2_2 * T65_2_2).inverse();

    // Point position depending on 4 angles found previously
    Vector3d P31_1_1, P31_1_2, P31_2_1, P31_2_2;
    Vector4d P;
    P = T41_1_1 * Vector4d(0, -D(3), 0, 1);
    P31_1_1 = P.head(3);
    P = T41_1_2 * Vector4d(0, -D(3), 0, 1);
    P31_1_2 = P.head(3);
    P = T41_2_1 * Vector4d(0, -D(3), 0, 1);
    P31_2_1 = P.head(3);
    P = T41_2_2 * Vector4d(0, -D(3), 0, 1);
    P31_2_2 = P.head(3);

    double C;

    double th3_1_1_1;
    double th3_1_1_2;
    C = (((P31_1_1).norm() * (P31_1_1).norm()) - A(2) * A(2) - A(3) * A(3)) / (2 * A(2) * A(3));
    if (abs(C) > 1)
    {
        cout << "Point out of the work space" << endl;
        th3_1_1_1 = numeric_limits<double>::quiet_NaN();
        th3_1_1_2 = numeric_limits<double>::quiet_NaN();
    }
    else
    {
        th3_1_1_1 = acos(C);
        th3_1_1_2 = -acos(C);
    }

    double th3_1_2_1;
    double th3_1_2_2;
    C = (((P31_1_2).norm() * (P31_1_1).norm()) - A(2) * A(2) - A(3) * A(3)) / (2 * A(2) * A(3));
    if (abs(C) > 1)
    {
        cout << "Point out of the work space" << endl;
        th3_1_2_1 = numeric_limits<double>::quiet_NaN();
        th3_1_2_2 = numeric_limits<double>::quiet_NaN();
    }
    else
    {
        th3_1_2_1 = acos(C);
        th3_1_2_2 = -acos(C);
    }

    double th3_2_1_1;
    double th3_2_1_2;
    C = (((P31_2_1).norm() * (P31_1_1).norm()) - A(2) * A(2) - A(3) * A(3)) / (2 * A(2) * A(3));
    if (abs(C) > 1)
    {
        cout << "Point out of the work space" << endl;
        th3_2_1_1 = numeric_limits<double>::quiet_NaN();
        th3_2_1_2 = numeric_limits<double>::quiet_NaN();
    }
    else
    {
        th3_2_1_1 = acos(C);
        th3_2_1_2 = -acos(C);
    }

    double th3_2_2_1;
    double th3_2_2_2;
    C = (((P31_2_2).norm() * (P31_1_1).norm()) - A(2) * A(2) - A(3) * A(3)) / (2 * A(2) * A(3));
    if (abs(C) > 1)
    {
        cout << "Point out of the work space" << endl;
        th3_2_2_1 = numeric_limits<double>::quiet_NaN();
        th3_2_2_2 = numeric_limits<double>::quiet_NaN();
    }
    else
    {
        th3_2_2_1 = acos(C);
        th3_2_2_2 = -acos(C);
    }

    double th2_1_1_1 = -atan2(P31_1_1(1), -P31_1_1(0)) + asin((A(2) * sin(th3_1_1_1)) / P31_1_1.norm());
    double th2_1_1_2 = -atan2(P31_1_1(1), -P31_1_1(0)) + asin((A(2) * sin(th3_1_1_2)) / P31_1_1.norm());
    double th2_1_2_1 = -atan2(P31_1_2(1), -P31_1_2(0)) + asin((A(2) * sin(th3_1_2_1)) / P31_1_2.norm());
    double th2_1_2_2 = -atan2(P31_1_2(1), -P31_1_2(0)) + asin((A(2) * sin(th3_1_2_2)) / P31_1_2.norm());
    double th2_2_1_1 = -atan2(P31_2_1(1), -P31_2_1(0)) + asin((A(2) * sin(th3_2_1_1)) / P31_2_1.norm());
    double th2_2_1_2 = -atan2(P31_2_1(1), -P31_2_1(0)) + asin((A(2) * sin(th3_2_1_2)) / P31_2_1.norm());
    double th2_2_2_1 = -atan2(P31_2_2(1), -P31_2_2(0)) + asin((A(2) * sin(th3_2_2_1)) / P31_2_2.norm());
    double th2_2_2_2 = -atan2(P31_2_2(1), -P31_2_2(0)) + asin((A(2) * sin(th3_2_2_2)) / P31_2_2.norm());

    Matrix4d T21, T32, T41, T43;
    double xy, xx;

    T21 = computeTransformationMatrix(th2_1_1_1, ALPHA(1), D(1), A(1));
    T32 = computeTransformationMatrix(th3_1_1_1, ALPHA(2), D(2), A(2));
    T41 = T41_1_1;
    T43 = ((T21 * T32).inverse()) * T41;
    xy = T43(1, 0);
    xx = T43(0, 0);
    double th4_1_1_1 = atan2(xy, xx);

    T21 = computeTransformationMatrix(th2_1_1_2, ALPHA(1), D(1), A(1));
    T32 = computeTransformationMatrix(th3_1_1_2, ALPHA(2), D(2), A(2));
    T41 = T41_1_1;
    T43 = ((T21 * T32).inverse()) * T41;
    xy = T43(1, 0);
    xx = T43(0, 0);
    double th4_1_1_2 = atan2(xy, xx);

    T21 = computeTransformationMatrix(th2_1_2_1, ALPHA(1), D(1), A(1));
    T32 = computeTransformationMatrix(th3_1_2_1, ALPHA(2), D(2), A(2));
    T41 = T41_1_2;
    T43 = ((T21 * T32).inverse()) * T41;
    xy = T43(1, 0);
    xx = T43(0, 0);
    double th4_1_2_1 = atan2(xy, xx);

    T21 = computeTransformationMatrix(th2_1_2_2, ALPHA(1), D(1), A(1));
    T32 = computeTransformationMatrix(th3_1_2_2, ALPHA(2), D(2), A(2));
    T41 = T41_1_2;
    T43 = ((T21 * T32).inverse()) * T41;
    xy = T43(1, 0);
    xx = T43(0, 0);
    double th4_1_2_2 = atan2(xy, xx);

    T21 = computeTransformationMatrix(th2_2_1_1, ALPHA(1), D(1), A(1));
    T32 = computeTransformationMatrix(th3_2_1_1, ALPHA(2), D(2), A(2));
    T41 = T41_2_1;
    T43 = ((T21 * T32).inverse()) * T41;
    xy = T43(1, 0);
    xx = T43(0, 0);
    double th4_2_1_1 = atan2(xy, xx);

    T21 = computeTransformationMatrix(th2_2_1_2, ALPHA(1), D(1), A(1));
    T32 = computeTransformationMatrix(th3_2_1_2, ALPHA(2), D(2), A(2));
    T41 = T41_2_1;
    T43 = ((T21 * T32).inverse()) * T41;
    xy = T43(1, 0);
    xx = T43(0, 0);
    double th4_2_1_2 = atan2(xy, xx);

    T21 = computeTransformationMatrix(th2_2_2_1, ALPHA(1), D(1), A(1));
    T32 = computeTransformationMatrix(th3_2_2_1, ALPHA(2), D(2), A(2));
    T41 = T41_2_2;
    T43 = ((T21 * T32).inverse()) * T41;
    xy = T43(1, 0);
    xx = T43(0, 0);
    double th4_2_2_1 = atan2(xy, xx);

    T21 = computeTransformationMatrix(th2_2_2_2, ALPHA(1), D(1), A(1));
    T32 = computeTransformationMatrix(th3_2_2_2, ALPHA(2), D(2), A(2));
    T41 = T41_2_2;
    T43 = ((T21 * T32).inverse()) * T41;
    xy = T43(1, 0);
    xx = T43(0, 0);
    double th4_2_2_2 = atan2(xy, xx);
    cout << "hi1" << endl;
    // // Finally compute the inverse kinematic and find the joint angles (remembering we started from final rotation matrix)
    Matrix<double, 6, 8> Th;
    Th << th1_1, th1_1, th1_1, th1_1, th1_2, th1_2, th1_2, th1_2,
        th2_1_1_1, th2_1_1_2, th2_1_2_1, th2_1_2_2, th2_2_2_1, th2_2_1_2, th2_2_2_1, th2_2_2_2,
        th3_1_1_1, th3_1_1_2, th3_1_2_1, th3_1_2_2, th3_2_2_1, th3_2_1_2, th3_2_2_1, th3_2_2_2,
        th4_1_1_1, th4_1_1_2, th4_1_2_1, th4_1_2_2, th4_2_2_1, th4_2_1_2, th4_2_2_1, th4_2_2_2,
        th5_1_1, th5_1_1, th5_1_2, th5_1_2, th5_2_2, th5_2_1, th5_2_2, th5_2_2,
        th6_1_1, th6_1_1, th6_1_2, th6_1_2, th6_2_2, th6_2_1, th6_2_2, th6_2_2;
    return Th;
}

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

Quaterniond myslerp(double time, Quaterniond q1, Quaterniond q2)
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

Matrix6d computeJacobianCross(Vector6d Th)
{
    Matrix4d T10 = base_to_world() * computeTransformationMatrix(Th(0), ALPHA(0), D(0), A(0));
    Matrix4d T21 = computeTransformationMatrix(Th(1), ALPHA(1), D(1), A(1));
    Matrix4d T32 = computeTransformationMatrix(Th(2), ALPHA(2), D(2), A(2));
    Matrix4d T43 = computeTransformationMatrix(Th(3), ALPHA(3), D(3), A(3));
    Matrix4d T54 = computeTransformationMatrix(Th(4), ALPHA(4), D(4), A(4));
    Matrix4d T65 = computeTransformationMatrix(Th(5), ALPHA(5), D(5), A(5));
    Matrix4d transMatrix20;
    transMatrix20 = T10 * T21;
    Matrix4d transMatrix30;
    transMatrix30 = transMatrix20 * T32;
    Matrix4d transMatrix40;
    transMatrix40 = transMatrix30 * T43;
    Matrix4d transMatrix50;
    transMatrix50 = transMatrix40 * T54;
    Matrix4d transMatrix60;
    transMatrix60 = transMatrix50 * T65 * adjust_gripper();
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

Path differential_inverse_kin_quaternions(Vector8d mr, Vector3d f_p, Quaterniond f_q)
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
	i_tm = base_to_world() * directKin(mr.head(6)) * adjust_gripper();
	i_p = i_tm.block(0, 3, 3, 1);
	i_rm = i_tm.block(0, 0, 3, 3);
	i_q	= i_rm;
	
    //insert the starting point to the path
    for (int i = 0; i < 6; ++i) js_k(i) = mr(i);
    path = insert_new_path_instance(path, js_k, gs);
    // compute the direct kinematics in the instant k
    tm_k = base_to_world() * directKin(js_k) * adjust_gripper();
    p_k = tm_k.block(0, 3, 3, 1);
    rm_k = tm_k.block(0, 0, 3, 3);
    q_k = rm_k;

    std::cout << "starting p: " << p_k.transpose() << " actual q: " << q_k.coeffs().transpose() << std::endl;

    //each delta time (dt) compute the joints state to insert into the path 
    for (double t = 0; t < path_dt; t += dt) //for (double t = dt; t < path_dt; t += dt)
    {
//			std::cout << "##### t = " << t << " #####" << std::endl;
        //compute the direct kinematics in the instant k 
        tm_k = base_to_world() * directKin(js_k) * adjust_gripper();
        p_k = tm_k.block(0, 3, 3, 1);
        rm_k = tm_k.block(0, 0, 3, 3);
        q_k = rm_k;
		
//		std::cout << "actual p: " << p_k.transpose() << " actual q: " << q_k.coeffs().transpose() << std::endl;
		
		//compute the velocities in the instant k
        pv_k = (lerp(t + dt, i_p, f_p) - lerp(t, i_p, f_p)) / dt; //pv_k = (lerp(t, i_p, f_p) - lerp(t - dt, i_p, f_p)) / dt;
        qv_k = myslerp(t + dt, i_q, f_q) * myslerp(t, i_q, f_q).conjugate(); //qv_k = myslerp(t, i_q, f_q) * myslerp(t - dt, i_q, f_q).conjugate();
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
		qerr_k = myslerp(t, i_q, f_q) * q_k.conjugate();
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

Path insert_new_path_instance(Path p, Vector6d js, Vector2d gs)
{
    // Incrementa le dimensioni della matrice di una riga
    p.conservativeResize(p.rows() + 1, p.cols());

    // Inserisci i valori delle giunture e dello stato della pinza nella nuova riga
    p.row(p.rows() - 1) << js(0), js(1), js(2), js(3), js(4), js(5), gs(0), gs(1);

    return p;
}

// Matrix6d computeJacobianCross(Vector6d Th)
// {
//     Matrix4d T10 = base_to_world() * computeTransformationMatrix(Th(0), ALPHA(0), D(0), A(0));
//     Matrix4d T21 = computeTransformationMatrix(Th(1), ALPHA(1), D(1), A(1));
//     Matrix4d T32 = computeTransformationMatrix(Th(2), ALPHA(2), D(2), A(2));
//     Matrix4d T43 = computeTransformationMatrix(Th(3), ALPHA(3), D(3), A(3));
//     Matrix4d T54 = computeTransformationMatrix(Th(4), ALPHA(4), D(4), A(4));
//     Matrix4d T65 = computeTransformationMatrix(Th(5), ALPHA(5), D(5), A(5));
//     Matrix4d transMatrix20;
//     transMatrix20 = T10 * T21;
//     Matrix4d transMatrix30;
//     transMatrix30 = transMatrix20 * T32;
//     Matrix4d transMatrix40;
//     transMatrix40 = transMatrix30 * T43;
//     Matrix4d transMatrix50;
//     transMatrix50 = transMatrix40 * T54;
//     Matrix4d transMatrix60;
//     transMatrix60 = transMatrix50 * T65 * adjust_gripper();
//     Vector3d z0, z1, z2, z3, z4, z5;
//     z0 << 0, 0, -1;
//     z1 = T10.col(2).head(3);
//     z2 = transMatrix20.col(2).head(3);
//     z3 = transMatrix30.col(2).head(3);
//     z4 = transMatrix40.col(2).head(3);
//     z5 = transMatrix50.col(2).head(3);
//     Vector3d p0, p1, p2, p3, p4, p5, p6;
//     p0 << 0.5, 0.35, 1.75;
//     p1 = T10.col(3).head(3);
//     p2 = transMatrix20.col(3).head(3);
//     p3 = transMatrix30.col(3).head(3);
//     p4 = transMatrix40.col(3).head(3);
//     p5 = transMatrix50.col(3).head(3);
//     p6 = transMatrix60.col(3).head(3);
//     Matrix6d geomJacobian;
//     geomJacobian << z0.cross(p6 - p0), z1.cross(p6 - p1), z2.cross(p6 - p2), z3.cross(p6 - p3), z4.cross(p6 - p4), z5.cross(p6 - p5),
//         z0, z1, z2, z3, z4, z5;
//         return geomJacobian;
// }