#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <stdio.h>


/// @todo In all the functions that return homogeneous matrices the matrices have to be initialized to the identity

typedef Eigen::Quaternion<double>   eQuaternion;
typedef Eigen::Vector2d             eVector2;
typedef Eigen::Vector3d             eVector3;

typedef Eigen::Matrix2d             eMatrixRot2d;
typedef Eigen::Matrix3d             eMatrixRot;
typedef Eigen::Isometry2d           eMatrixHom2d;
typedef Eigen::Isometry3d           eMatrixHom;
typedef Eigen::AngleAxisd           eAngleAxis;

const double DEG_TO_RAD = M_PI/180.0;
const double RAD_TO_DEG = 1.0/DEG_TO_RAD;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

inline double ABS(double x){    if (x<0) return -x ; else return x;  }

inline double SQR(double x){   return x*x;  }

//simetrical bilateral clamp
inline double CLAMP(double x, double clamp)
{
    if( x > clamp ) return  clamp;
    if( x < -clamp) return -clamp;
    return x;
}

inline double CLAMP(double x, double clamp_min, double clamp_max)
{
    if( x > clamp_max ) return  clamp_max;
    if( x < clamp_min) return clamp_min;
    return x;
}

inline double SIGN(double x)
{
    return (x > 0.0 ) ? 1.0 : -1.0;
}

enum{ X = 0,
      Y = 1,
      Z = 2 };

enum{ FX = 0,
      FY = 1,
      FZ = 2,
      TX = 3,
      TY = 4,
      TZ = 5};

enum{ ROLL   = X,
      PITCH  = Y,
      YAW    = Z };

//------------------------------------------------------------------

inline eMatrixRot  eulerRotX(double eulerX) {
    double ci ( cos(eulerX));
    double si ( sin(eulerX));

    eMatrixRot temp;
    temp <<     1,   0,   0,
                0,   ci,  -si,
                0,   si,  ci;
    return temp;
}

inline eMatrixRot  eulerRotY(double eulerY) {
    double ci ( cos(eulerY));
    double si ( sin(eulerY));

    eMatrixRot temp;
    temp <<  ci,   0,   si,
            0,    1,   0,
            -si,   0,   ci;
    return temp;
}

inline eMatrixRot  eulerRotZ(double eulerZ) {
    double ci ( cos(eulerZ));
    double si ( sin(eulerZ));

    eMatrixRot temp;
    temp << ci ,   -si ,  0,
            si ,   ci,   0,
            0 ,   0,    1;
    return temp;
}

/// see http://planning.cs.uiuc.edu/node102.html

inline eMatrixRot matrixRollPitchYaw( double roll, double pitch, double yaw)
{
    double ca ( cos(yaw));
    double sa ( sin(yaw));

    double cb ( cos(pitch));
    double sb ( sin(pitch));

    double cc ( cos(roll));
    double sc ( sin(roll));

    eMatrixRot temp;
    temp << ca*cb ,   ca*sb*sc - sa*cc ,  ca*sb*cc + sa*sc,
            sa*cb ,   sa*sb*sc + ca*cc ,  sa*sb*cc - ca*sc,
            -sb   ,   cb*sc            ,  cb*cc;
    return temp;
}

inline eMatrixHom HommatrixRollPitchYaw( double roll, double pitch, double yaw)
{
    double ca ( cos(yaw));
    double sa ( sin(yaw));

    double cb ( cos(pitch));
    double sb ( sin(pitch));

    double cc ( cos(roll));
    double sc ( sin(roll));

    eMatrixRot rot;
    rot << ca*cb ,   ca*sb*sc - sa*cc ,  ca*sb*cc + sa*sc,
            sa*cb ,   sa*sb*sc + ca*cc ,  sa*sb*cc - ca*sc,
            -sb   ,   cb*sc            ,  cb*cc;

    eVector3 trans(0,0,0);
    eMatrixHom hom;
    hom.setIdentity();
    hom = ( rot );
    hom.translation() = trans;
    return hom;
}

/// see http://planning.cs.uiuc.edu/node103.html

inline void extractRollPitchYaw(eMatrixRot const& matrix, double *roll, double *pitch, double *yaw)
{
    *yaw = atan2( matrix.data()[1], matrix.data()[0] );
    *pitch = atan2( matrix.data()[2], sqrt( SQR( matrix.data()[5]) + SQR(matrix.data()[8]) ) );
    *roll = atan2( matrix.data()[5],  matrix.data()[8] ) ;
}

inline void extractRollPitchYaw(eMatrixHom const& matrix, double *roll, double *pitch, double *yaw)
{
    *yaw   = atan2(   matrix.data()[1],  matrix.data()[0] );
    *pitch = atan2(  -matrix.data()[2],  sqrt( SQR( matrix.data()[6]) + SQR(matrix.data()[10]) ) );
    *roll  = atan2(   matrix.data()[6],  matrix.data()[10] ) ;
}

inline double extractYaw(eMatrixHom const& matrix)
{
    return atan2(   matrix.data()[1],  matrix.data()[0] );
}
inline double extractRoll(eMatrixHom const& matrix)
{
    return atan2(    matrix.data()[6],  matrix.data()[10] ) ;
}
inline double extractPitch(eMatrixHom const& matrix)
{
    return atan2(  -matrix.data()[2],  sqrt( SQR( matrix.data()[6]) + SQR(matrix.data()[10]) ) );
}


inline void setRoll(eMatrixHom &matrix_hom, double des_roll)
{
    eMatrixRot matrix = matrix_hom.rotation();
    double yaw = atan2( matrix.data()[1], matrix.data()[0] );
    double pitch = atan2( matrix.data()[2], sqrt( SQR( matrix.data()[5]) + SQR(matrix.data()[8]) ) );
    double roll = des_roll;

    matrix_hom.linear() =  matrixRollPitchYaw(roll, pitch, yaw);

}

//Optimize this
inline void setPitch(eMatrixHom &matrix_hom, double des_pitch)
{
    eMatrixRot matrix = matrix_hom.rotation();
    double yaw = atan2( matrix.data()[1], matrix.data()[0] );
    double pitch = des_pitch;
    double roll = atan2( matrix.data()[5],  matrix.data()[8] );

    matrix_hom.linear() =  matrixRollPitchYaw(roll, pitch, yaw);

}

inline void setYaw(eMatrixHom &matrix_hom, double des_yaw)
{
    eMatrixRot matrix = matrix_hom.rotation();
    double yaw = des_yaw;
    double pitch = atan2( matrix.data()[2], sqrt( SQR( matrix.data()[5]) + SQR(matrix.data()[8]) ) );
    double roll = atan2( matrix.data()[5],  matrix.data()[8] );

    matrix_hom.linear() =  matrixRollPitchYaw(roll, pitch, yaw);
}


inline eMatrixHom createMatrix( eQuaternion const& quat, eVector3 const& trans)
{
    eMatrixHom temp;
    temp.setIdentity();
    temp = ( Eigen::AngleAxisd(quat) );
    temp.translation() = trans;
    return temp;
}

inline eMatrixHom createMatrix( eAngleAxis const& aa, eVector3 const& trans)
{
    eMatrixHom temp;
    temp.setIdentity();
    temp = ( aa );
    temp.translation() = trans;
    return temp;
}

inline eMatrixHom createMatrix( eMatrixRot const& rot, eVector3 const& trans)
{
    eMatrixHom temp;
    temp.setIdentity();
    temp = (rot);
    temp.translation() = trans;
    return temp;
}


//Construtor that receives a pointer to boos array, in the array the translation comes first
//and rpy after
inline eMatrixHom createMatrix(eVector3 const& rotation, eVector3 const& position)
{
  eMatrixRot rot = matrixRollPitchYaw(rotation[0], rotation[1], rotation[2]);
  return createMatrix(rot, position);

}

inline double RPM_to_RAD_SEC(double x)
{
    return (x/60)*(2.0*M_PI);
}

inline void printMatrix( eMatrixHom const& mat)
{
    printf("%.4f\t%.4f\t%.4f\t\t %.4f\n", mat(0,0), mat(0,1), mat(0,2), mat(0,3));
    printf("%.4f\t%.4f\t%.4f\t\t %.4f\n", mat(1,0), mat(1,1), mat(1,2), mat(1,3));
    printf("%.4f\t%.4f\t%.4f\t\t %.4f\n", mat(2,0), mat(2,1), mat(2,2), mat(2,3));
    printf("%.4f\t%.4f\t%.4f\t\t %.4f\n", mat(3,0), mat(3,1), mat(3,2), mat(3,3));
}


#endif // MATH_UTILS_H
