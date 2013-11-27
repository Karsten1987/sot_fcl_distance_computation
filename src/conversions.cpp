#include <sot_fcl_distance_computation/conversions.h>

//TODO: this should be inlines
namespace distance{
  //KDL vector to eigen
  void convert(const KDL::Vector &in, eVector3 &out){
    out<<in.x(), in.y(), in.z();
  }

  /*
//wykobi vector to eigen
eVector3 to2Eigen(wykobi::vector3d in){
  eVector3 out;
  out<<in.x, in.y, in.z;
  return out;
}
*/

  void convert(const KDL::Twist &in, eVector3 &out_vel, eVector3 &out_rot){
    for(unsigned int i=0; i<3; ++i){
      out_vel[i] = in.vel[i];
    }

    for(unsigned int i=0; i<3; ++i){
      out_rot[i] = in.rot[i];
    }
  }

  void convert(const eVector3 &in_vel, const eVector3 &in_rot, KDL::Twist &out){
    out = KDL::Twist(KDL::Vector(in_vel[X],
                                 in_vel[Y],
                                 in_vel[Z]),
                     KDL::Vector(in_rot[X],
                                 in_rot[Y],
                                 in_rot[Z]));
  }

  void convert(const eMatrixHom &in,  tf::Transform &out){
    Eigen::Quaternion<double> q(in.rotation());
    out = tf::Transform(tf::Quaternion(q.x(),
                                       q.y(),
                                       q.z(),
                                       q.w()),
                        tf::Vector3(in.translation()[X],
                                    in.translation()[Y],
                                    in.translation()[Z]));
  }


  void convert(const eMatrixHom &in,  KDL::Frame &out){
    Eigen::Quaternion<double> q(in.rotation());
    out = KDL::Frame(KDL::Rotation::Quaternion(q.x(),
                                               q.y(),
                                               q.z(),
                                               q.w()),
                     KDL::Vector(in.translation()[X],
                                 in.translation()[Y],
                                 in.translation()[Z]));
  }

  void convert(const eVector3 &in_pos, const eVector3 &in_rot, KDL::Frame &out){
    out = KDL::Frame(KDL::Rotation::RPY(in_rot[X],
                                        in_rot[Y],
                                        in_rot[Z]),
                     KDL::Vector(in_pos[X],
                                 in_pos[Y],
                                 in_pos[Z]));
  }


  //eigen to KDL vector
  void convert(const eVector3 &in, KDL::Vector &out){
    for(unsigned int i=0; i<3; ++i) out[i] = in[i];
  }

  void convert(const urdf::Vector3 &v, KDL::Vector &out)
  {
    out = KDL::Vector(v.x, v.y, v.z);
  }

  // construct rotation
  void convert(const urdf::Rotation &r, KDL::Rotation &out)
  {
    out = KDL::Rotation::Quaternion(r.x, r.y, r.z, r.w);
  }

  // construct pose
  void convert(const urdf::Pose &in, KDL::Frame &out)
  {
    KDL::Rotation r;
    KDL::Vector p;
    convert(in.rotation, r);
    convert(in.position, p);

    out = KDL::Frame(r, p);
  }

  typedef boost::shared_ptr<const urdf::Link> linkPtr;
  typedef boost::shared_ptr<const urdf::Joint> jointPtr;

  // double vector to KDL array
  void convert(const std::vector<double> &in, KDL::JntArray &out){
    assert(in.size() == out.rows());
    for(unsigned int i=0; i<in.size(); ++i){
      out(i) = in[i];
    }
  }

  void convert(const std::vector<double> &in, Eigen::VectorXd &out){
    assert(in.size() == out.rows());
    for(unsigned int i=0; i<in.size(); ++i) out[i] = in[i];
  }

  // KDL transform to fcl transform
  void convert(const KDL::Frame &in, fcl::Transform3f &out){
    double x,y,z,w;
    in.M.GetQuaternion(x, y, z, w);
    fcl::Vec3f t(in.p[0], in.p[1], in.p[2]);
    fcl::Quaternion3f q(w, x, y, z);
    out.setQuatRotation(q);
    out.setTranslation(t);
  }

  void convert(const KDL::Frame &in, eMatrixHom &out){
    double x,y,z,w;
    in.M.GetQuaternion(x, y, z, w);
    out =  createMatrix(eQuaternion(w, x, y, z),
                        eVector3(in.p.x(), in.p.y(), in.p.z()));

  }

  void convert(const KDL::Frame &in, tf::Transform &out){
    double x,y,z,w;
    in.M.GetQuaternion(x, y, z, w);

    out.setOrigin(tf::Vector3(in.p.x(),
                              in.p.y(),
                              in.p.z()) );
    out.setRotation(tf::Quaternion(x, y, z, w) );
  }


  void convert(const fcl::Transform3f &in, tf::Transform &out){
    out.setOrigin(tf::Vector3(in.getTranslation()[0],
                              in.getTranslation()[1],
                              in.getTranslation()[2]) );
    out.setRotation(tf::Quaternion(in.getQuatRotation().getX(),
                                   in.getQuatRotation().getY(),
                                   in.getQuatRotation().getZ(),
                                   in.getQuatRotation().getW()) );
  }

  //Convert from a 3d Homogeneous matrix to a 2d Homogeneous matrix
  //(Project into the floor)

  void convert(const eMatrixHom &in, eMatrixHom2d &out){
     double yaw = extractYaw(in);
     out.setIdentity();
     out.rotate(yaw);
     out.translation() =  eVector2(in.translation()[X], in.translation()[Y]);
  }

}

