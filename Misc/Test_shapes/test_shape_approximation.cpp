#include <sstream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Geometry>
#include <Gripper/Shape.h>
#include <Gripper/Sphere.h>
#include <Gripper/Cuboid.h>
#include <Gripper/ComposedShape.h>


void mostra(const Gripper::Shape& sp){
  using Eigen::Translation3d;
  using Eigen::Vector3d;
  using Eigen::Affine3d;
  using Eigen::AngleAxisd;
  using Gripper::Shape;
  using Gripper::Cuboid;
  using Gripper::Sphere;
  using std::vector;
  typedef pcl::visualization::PointCloudColorHandlerCustom<Shape::PointType> ColorHandler;

  std::array<size_t, 3> levels{9, 25, 41};

  {
    std::array<Gripper::Shape::PointsPtr, 3> spSurf;
    std::vector<std::vector<double> > colors{{255,0,0},{0,255,0},{0,0,255}};

    for(int i=0; i<3; ++i){
      spSurf[i]=sp.getPCSurface(levels[i]);
    }

    std::vector<ColorHandler> spColorHandlers;
    pcl::visualization::PCLVisualizer spViewer(sp.getID());

    for(int i=0; i<3; ++i){
      spColorHandlers.emplace_back(spSurf[i], colors[i][0], colors[i][1], colors[i][2]);
      std::stringstream title;
      title << "l=" << levels[i];
      spViewer.addPointCloud<pcl::PointXYZ>(spSurf[i], spColorHandlers[i], title.str().c_str());
    }
    {
      std::stringstream data;
      data << "Surface approximation for l=" ;
      for(int i=0; i<3; ++i){
        data << levels[i] ;
        if(i!=2){
          data << ", ";
        }
        else{
          data << ".";
        }
      }
      data << "\n\n";
      data << "ID: " << sp.getID() << "\nTransformation matrix:\n" << sp.getPose().matrix() ;
      spViewer.addText(data.str(), 20, 20);
    }

    spViewer.addCoordinateSystem(1);
    while(!spViewer.wasStopped()){
      spViewer.spinOnce(100);
    }
  }

  {
    std::array<Gripper::Shape::PointsPtr, 3> spSurf;
    std::vector<std::vector<double> > colors{{255,0,0},{0,255,0},{0,0,255}};

    for(int i=0; i<3; ++i){
      spSurf[i]=sp.getPCVolume(levels[i]);
    }

    std::vector<ColorHandler> spColorHandlers;
    pcl::visualization::PCLVisualizer spViewer(sp.getID());

    for(int i=0; i<3; ++i){
      spColorHandlers.emplace_back(spSurf[i], colors[i][0], colors[i][1], colors[i][2]);
      std::stringstream title;
      title << "l=" << levels[i];
      spViewer.addPointCloud<pcl::PointXYZ>(spSurf[i], spColorHandlers[i], title.str().c_str());
    }
    {
      std::stringstream data;
      data << "Volume approximation for l=" ;
      for(int i=0; i<3; ++i){
        data << levels[i] ;
        if(i!=2){
          data << ", ";
        }
        else{
          data << ".";
        }
      }
      data << "\n\n";
      data << "ID: " << sp.getID() << "\nTransformation matrix:\n" << sp.getPose().matrix() ;
      spViewer.addText(data.str(), 20, 20);
    }

    spViewer.addCoordinateSystem(1);
    while(!spViewer.wasStopped()){
      spViewer.spinOnce(100);
    }
  }
}

int main(){

  using Eigen::Translation3d;
  using Eigen::Vector3d;
  using Eigen::Affine3d;
  using Eigen::AngleAxisd;
  using Gripper::Shape;
  using Gripper::Cuboid;
  using Gripper::Sphere;
  using Gripper::ComposedShape;
  using std::vector;
  using std::make_shared;
  //TODO cube

  std::cout << "Approximating a cube's and a sphere's surface and volume using point clouds\n";

  Sphere sp{Affine3d{Translation3d{0,0,2}}, 3};

  Cuboid c{Affine3d{Translation3d{0,0,-1}*AngleAxisd(50*M_PI/180.0, Vector3d{44,89,13}.normalized())}, 1, 1.4, 3};

  std::vector<std::shared_ptr<Shape> > components{make_shared<decltype(sp)>(sp), make_shared<decltype(c)>(c)};
  ComposedShape comp{Affine3d{Translation3d{0,-1,0}*AngleAxisd(45*M_PI/180.0, Vector3d{10,11,0}.normalized())}, components};
  mostra(sp);
  mostra(c);
  mostra(comp);




  return 0;
}
