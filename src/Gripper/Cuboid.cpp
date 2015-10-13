#include <algorithm>
#include <Gripper/Cuboid.h>

namespace Gripper{
  std::string Cuboid::getID() const {
    return "Cuboid";
  }
  auto Cuboid::getKnownIntersections() const -> KnownIntersections{
    return {"Cuboid", "Sphere"};
  }

  Eigen::Matrix<double, 4, Eigen::Dynamic> Cuboid::getCubettiMatrix(std::vector<double> dimensions) {
    assert(dimensions.size()==3 && "Cubetti from something which is not a cube");

    double W=dimensions[0];
    double H=dimensions[1];
    double D=dimensions[2];
    /** Get an approximation using points of this cube size 1/100 */
    double stepX=W/NCUT;
    double stepY=H/NCUT;
    double stepZ=D/NCUT;
    double startX=stepX/2.0;
    double startY=stepY/2.0;
    double startZ=stepZ/2.0;

    Eigen::Matrix<double, Eigen::Dynamic, 4> cubePoints(NCUT*NCUT*NCUT, 4);

    size_t pos=0;
    for(size_t i=0; i<NCUT; ++i){
      for(size_t j=0; j<NCUT; ++j){
        for(size_t k=0; k<NCUT; ++k){
          cubePoints.row(pos++)=Eigen::Vector4d{startX+i*stepX, startY+j*stepY, startZ+k*stepZ, 1}.transpose();
        }
      }
    }
    return cubePoints.transpose();
  }

  double Cuboid::intersectionVolume(const Shape& s) const{
    auto id=s.getID();
    if(id=="Sphere"){
      auto cubePoints=getCubettiMatrix(_dimensions);
      auto realCubePoints=_pose.matrix().topRows<3>()*cubePoints;
      auto distances=Eigen::Vector3d{1,1,1}.transpose()*((realCubePoints-s.getPose().translation().rowwise().replicate(NCUT*NCUT*NCUT)).cwiseAbs2());

      double R=s.getDimensions()[0];
      auto bom=R*R-distances.array();
      double V=(bom > 0.0).count()*(W()*H()*D()/(NCUT*NCUT*NCUT));
      return V;
    }
    assert(id=="Cuboid");
    using std::min;
    using std::max;

    double myVol=W()*H()*D();
    auto d2=s.getDimensions();
    double vol2=d2[0]*d2[1]*d2[2];

    const Shape& base=(myVol > vol2) ? *this : s;
    const Shape& other=(myVol > vol2) ? s : *this;
    double otherVol=(myVol > vol2) ? vol2 : myVol;

    auto otherPoints=getCubettiMatrix(other.getDimensions());
    auto relPose=base.getPose().inverse()*other.getPose();
    auto realOtherPoints=(relPose.matrix().topRows<3>()*otherPoints).array();

    auto baseD=base.getDimensions();

    auto ltdPoints=(realOtherPoints.row(0) < baseD[0]) && (realOtherPoints.row(1) < baseD[1]) && (realOtherPoints.row(2) < baseD[2]);
    auto greatPoints=(realOtherPoints.row(0) > 0) && (realOtherPoints.row(1) > 0) && (realOtherPoints.row(2) > 0);
    double V=( ltdPoints && greatPoints).count() *otherVol/(NCUT*NCUT*NCUT);
    return V;
  }

  double Cuboid::W() const {
    return _dimensions[0];
  }
  double Cuboid::H() const { 
    return _dimensions[1];
  }
  double Cuboid::D() const {
    return _dimensions[2];
  }
  Cuboid::Cuboid(Eigen::Affine3d pose, double W, double H, double D)
    :
      Shape(pose, {W,H,D})
    {
    }

}

