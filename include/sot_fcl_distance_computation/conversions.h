#ifndef _CONVERSIONS_
#define _CONVERSIONS_

#include <typeinfo>
#include <sot_fcl_distance_computation/math_utils.h>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <assert.h>
#include <fcl/collision.h>
#include <urdf/model.h>
#include <tf/tf.h>

/// @todo this should be inlines

namespace distance{

  //KDL Twist to eigen
  void convert(const KDL::Twist &in, eVector3 &out_vel, eVector3 &out_rot);

  void convert(const eVector3 &in_vel, const eVector3 &in_rot, KDL::Twist &out);

  //KDL vector to eigen
  void convert(const KDL::Vector &in, eVector3 &out);

  //eigen to KDL vector
  void convert(const eVector3 &in, KDL::Vector &out);

  void convert(const urdf::Vector3 &v, KDL::Vector &out);

  // construct rotation
  void convert(const urdf::Rotation &r, KDL::Rotation &out);

  // construct pose
  void convert(const urdf::Pose &in, KDL::Frame &out);

  void convert(const eVector3 &in_pos, const eVector3 &in_rot, KDL::Frame &out);

  typedef boost::shared_ptr<const urdf::Link> linkPtr;
  typedef boost::shared_ptr<const urdf::Joint> jointPtr;

  void convert(const std::map<std::string, std::vector<double> > &mapIn, KDL::JntArray &out);

  // std vector
  void convert(const std::vector<double> &in, KDL::JntArray &out);

  void convert(const std::vector<double> &in, Eigen::VectorXd &out);

  // KDL
  void convert(const KDL::Frame &in, fcl::Transform3f &out);

  // KDL Frame to Eigen Homogeneous matrix
  void convert(const KDL::Frame &in, eMatrixHom &out);

  // KDL Frame to tf
  void convert(const KDL::Frame &in, tf::Transform &out);

  void convert(const eMatrixHom &in,  KDL::Frame &out);

  void convert(const eMatrixHom &in,  tf::Transform &out);

  void convert(const fcl::Transform3f &in, tf::Transform &out);


  template<std::size_t N>
  void convert(const boost::array<double, N> &in, Eigen::Matrix<double, N, 1> &out){
    assert(in.size() == out.rows());
    for(unsigned int i=0; i<in.size(); ++i) out[i] = in[i];
  }

  /// @todo add an assert that checkes the sizes match
  template<std::size_t N>
  void convert(const Eigen::Matrix<double, N, 1> &in, boost::array<double, N> &out){
    assert(in.rows() == out.size());
    for(unsigned int i=0; i<in.size(); ++i) out[i] = in[i];
  }

  template <typename Derived>
  //WARING: Using derived clases from eigen oblies you to pass the values by reference
  double squared_difference(const Eigen::MatrixBase<Derived> &t1, const  Eigen::MatrixBase<Derived> &t2){
    return (t1.array().abs2() -t2.array().abs2()).sum();
  }


  // KDL transfor to fcl transform
  inline fcl::Transform3f kdl2fcl(const KDL::Frame &in){
      fcl::Transform3f out;
      double x,y,z,w;
      in.M.GetQuaternion(x, y, z, w);
      fcl::Vec3f t(in.p[0], in.p[1], in.p[2]);
      fcl::Quaternion3f q(w, x, y, z);
      out.setQuatRotation(q);
      out.setTranslation(t);
      return out;
  }

  //Returns the sum of the square root diference of the homogeneous tranformations
  //expressed as matrices. Mainly used for testing
  template <class T1, class T2>
  double differnce(const T1 &t1, const T2 &t2){
    //Only perfrom conversion if the two types are different
    T1 t2converted;
  //  if(typeid(T1) != typeid(T2)){
      convert(t2, t2converted);
      return squared_difference(t1, t2converted);
 //   }
  }

  //Conver a 3d homogeneous matrix into 2d.
  /// @todo can this be done copying needed values?
  void convert(const eMatrixHom &in, eMatrixHom2d &out);
}
#endif
