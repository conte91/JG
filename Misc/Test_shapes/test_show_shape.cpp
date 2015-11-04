#include <sstream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Geometry>
#include <Gripper/Shape.h>
#include <Gripper/ShapeBuilder.h>

void mostra(const Gripper::Shape& sp){
  using Eigen::Translation3d;
  using Eigen::Vector3d;
  using Eigen::Affine3d;
  using Eigen::AngleAxisd;
  using Gripper::Shape;
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

int main(int argc, char** argv){

  using cv::FileStorage;
  using Gripper::Shape;
  if(argc!=2){
    std::cerr << "Usage: " << argv[0] << " shape_file\n";
    return -1;
  }

  FileStorage fs(argv[1], FileStorage::READ);
  std::unique_ptr<Shape> result;
  fs["shape"] >> result;

  mostra(*result);

  return 0;
}
