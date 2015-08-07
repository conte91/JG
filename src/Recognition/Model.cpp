#include <cassert>
#include <cmath>
#include <Eigen/Core>
#include <Recognition/Model.h>
#include <Camera/CameraModel.h>
#include <boost/filesystem.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/rgbd.hpp>
#include <opencv2/core/eigen.hpp>
#include <C5G/Pose.h>

namespace Recognition{
  Model::Model(const std::string& id, const boost::filesystem::path& trainDir)
    :
      _myId(id),
      _camModel(1,1,1,1,1,1,1,1,1,1,1,1,1),
      _detector(cv::linemod::getDefaultLINEMOD())
  {
    readFrom(id, trainDir);
  }

  Model::Model(const std::string& id, const std::string& meshFile, const Camera::CameraModel& cam)
    :
      _myId(id),
      mesh_file_path(meshFile),
      _detector(cv::linemod::getDefaultLINEMOD()),
      _camModel(cam),
      renderer_width(cam.getWidth()),
      renderer_height(cam.getHeight()),
      renderer_focal_length_x(cam.getFx()),
      renderer_focal_length_y(cam.getFy()),
      renderer_near(0.1),
      renderer_far(10),
      renderer_n_points(10)
  {
    std::shared_ptr<Renderer3d> renderer (new Renderer3d(mesh_file_path));
    renderer->set_parameters(renderer_width, renderer_height, renderer_focal_length_x, renderer_focal_length_y, renderer_near, renderer_far);
    _renderer_iterator=std::shared_ptr<RendererIterator>(new RendererIterator(renderer, renderer_n_points));
  }


  void Model::readFrom(const std::string& id, const boost::filesystem::path& trainDir)
  {
    namespace fs=boost::filesystem;

    using fs::path;
    _myId=id;
    /* Load the yml of that class obtained in the Training phase */
    fs::path lmSaveFile = trainDir / path(_myId) / fs::path(_myId+"_Linemod.yml");

    cv::FileStorage inFile(lmSaveFile.string(), cv::FileStorage::READ);
    inFile["mesh_file_path"] >> mesh_file_path;
    _detector->read(inFile.root());
    std::cout<<"\tNumber of templates:"<<_detector->numTemplates()<<"\n";


    /** Read the trained class ID for this object */
    cv::FileNode fn = inFile["object"];
    fn["class_id"] >> _myId;
    _detector->readClass(fn);

    /** Initialize the renderer with the same parameters used for learning */
    std::shared_ptr<Renderer3d> renderer (new Renderer3d(mesh_file_path));

    cv::FileNode fr = inFile["rendering"];
    fr["renderer_width"] >> renderer_width;
    fr["renderer_height"] >> renderer_height;
    fr["renderer_focal_length_x"] >> renderer_focal_length_x;
    fr["renderer_focal_length_y"] >> renderer_focal_length_y;
    fr["renderer_near"] >> renderer_near;
    fr["renderer_far"] >> renderer_far;
    _camModel=Camera::CameraModel(renderer_width, renderer_height, renderer_focal_length_x, renderer_focal_length_y, 0, renderer_height/2.0, renderer_width/2.0);
    renderer->set_parameters(renderer_width, renderer_height, renderer_focal_length_x, renderer_focal_length_y, renderer_near, renderer_far);

    fr["renderer_n_points"] >> renderer_n_points;
    _renderer_iterator=std::shared_ptr<RendererIterator>(new RendererIterator(renderer, renderer_n_points));

    /**Read R**/
    const auto& _Rmap=readSequence<cv::Mat>(inFile["Rot"]);
    /**Read T**/
    const auto& _Tmap=readSequence<cv::Vec3d>(inFile["Transl"]);
    /**Read K**/
    std::vector<Camera::CameraModel> _Kmap;
    {
      const cv::FileNode& n=inFile["Camera"];
      /** Reads the camera models: can't use readSequence! */
      assert(n.type() == cv::FileNode::SEQ && "Data are not a sequence!");
      for(const auto& it : n){
        _Kmap.emplace_back(Camera::CameraModel::readFrom(it));
      }
    }

    /**Read Dist**/
    const auto& _distMap=readSequence<double>(inFile["dist"]);
    /**Read HueHist**/
    const auto& _hueHistMap=readSequence<cv::Mat>(inFile["Hue"]);

    _myData={};
    for(int i=0; i<_Rmap.size(); ++i){
      _myData.push_back({_Rmap[i], _Tmap[i], _distMap[i], _Kmap[i], _hueHistMap[i]});
    }

  }

  const std::vector<cv::linemod::Template> Model::getTemplates(int templateID) const {
    return _detector->getTemplates(_myId, templateID);
  }

  void Model::addAllTemplates(cv::linemod::Detector& det) const {
    for(int i=0; i<_detector->numTemplates(); ++i){
      auto& t=getTemplates(i);
      det.addSyntheticTemplate(t, _myId);
    }
  }

  int Model::numTemplates() const {
    return _detector->numTemplates();
  }
  cv::Mat Model::getR(int templateID) const {
    return _myData[templateID].R;
  }
  cv::Vec3d Model::getT(int templateID) const {
    return _myData[templateID].T;
  }
  double Model::getDist(int templateID) const {
    return _myData[templateID].dist;
  }
  cv::Mat Model::getK(int templateID) const {
    return _myData[templateID].cam.getIntrinsic();
  }
  Camera::CameraModel Model::getCam(int templateID) const {
    return _myData[templateID].cam;
  }
  cv::Mat Model::getHueHist(int templateID) const {
    return _myData[templateID].hueHist;
  }
  Model::TrainingData Model::getData(int templateID) const {
    return _myData[templateID];
  }

  void Model::render(cv::Vec3d T, cv::Vec3d up, cv::Mat &image_out, cv::Mat &depth_out, cv::Mat &mask_out, cv::Rect &rect_out) const {
    renderDepthOnly(T, up, depth_out, mask_out, rect_out);
    renderImageOnly(T, up, image_out, rect_out);
  }

  void Model::renderImageOnly(cv::Vec3d T, cv::Vec3d up, cv::Mat &image_out, cv::Rect &rect_out) const {
    _renderer_iterator->renderer_->lookAt(T(0), T(1), T(2), up(0), up(1), up(2));
    _renderer_iterator->renderer_->renderImageOnly(image_out, rect_out);
  }

  void Model::renderDepthOnly(cv::Vec3d T, cv::Vec3d up, cv::Mat &depth_out, cv::Mat &mask_out, cv::Rect &rect_out) const {
    _renderer_iterator->renderer_->lookAt(T(0), T(1), T(2), up(0), up(1), up(2));
    _renderer_iterator->renderer_->renderDepthOnly(depth_out, mask_out, rect_out);
  }

  //TODO pcl::PointCloud<pcl::PointXYZRGB> Model::getPointCloud(cv::Vec3d T, cv::Vec3d up, const Camera::CameraModel& cam ){
  //TODO   cv::Matrix
  //TODO }

  const std::shared_ptr<RendererIterator> Model::getRenderer() const {
    return _renderer_iterator;
  }

  cv::Matx33d Model::camTUp2ObjRot(const cv::Vec3d& tDir, const cv::Vec3d& upDir){
    cv::Vec3d t=tDir, up=upDir;
    normalize_vector(t(0),t(1),t(2));

    // compute the left vector
    cv::Vec3d y;
    y = up.cross(t);
    normalize_vector(y(0),y(1),y(2));

    // re-compute the orthonormal up vector
    up = t.cross(y);
    normalize_vector(up(0), up(1), up(2));

    cv::Mat R_full = (cv::Mat_<double>(3, 3) <<
        -y(0), -y(1), -y(2),
        -up(0), -up(1), -up(2),
        t(0), t(1), t(2)
        );

    cv::Matx33d R = R_full;
    R = R.t();

    return R.inv();
  }

  void Model::addTraining(const cv::Vec3d& T, const cv::Vec3d& up, const Camera::CameraModel& cam){
    cv::Mat image, depth, mask;
    cv::Matx33d R;

    cv::Rect rect;
    /** Performs a render of the object at the desired camera's position and adds it to the trained templates */
    render(T, up, image, depth, mask, rect);
    if(image.empty())
    {
      /** Nothing to be done, this template is completely unuseful as the object can't be seen from this position */
      return;
    }
    std::vector<cv::Mat> sources(2);
    sources[0] = image;
    sources[1] = depth;
    assert(image.type()==CV_8UC3);
    assert(depth.type()==CV_16UC1);
    int template_in = _detector->addTemplate(sources, _myId, mask);
    if (template_in == -1)
    {
      /** Nothing to be done, this template is completely unuseful as the object can't be seen from this position */
      return;
    }

    /** Computes and saves the parameters used for training */
    R = camTUp2ObjRot(T, up);
    double radius=::hypot(::hypot(T(0), T(1)), T(2));
    double distance = fabs(radius - depth.at<ushort>(depth.rows/2.0, depth.cols/2.0)/1000.0);

    /** hue histogram */
    /* Convert to HSV */
    cv::Mat hsv_mat;
    cv::cvtColor(image, hsv_mat, CV_BGR2HSV);
    std::vector<cv::Mat> hsv_planes;
    cv::split( hsv_mat, hsv_planes );
    /* Compute histogram over 30 bins */
    int Hbins = 30;
    int histSize = Hbins; /* 1D histogram */
    /* Range over which to compute the histogram */
    const float histRange[] = { 0, 180 };
    const float* histRangePtr = histRange;
    cv::Mat Hue_Hist;
    /* Compute and normalize hist only on the mask */
    cv::calcHist( &hsv_planes[0], 1, 0, mask, Hue_Hist, 1, &histSize, &histRangePtr, true, false );
    cv::normalize(Hue_Hist, Hue_Hist, 0.0, 1.0, cv::NORM_MINMAX, -1, cv::Mat() );

    /** Save the computed data */
    _myData.emplace_back(TrainingData{cv::Mat(R), -T, distance, cam, cv::Mat(Hue_Hist)});

  }

  void Model::saveToDirectory(const boost::filesystem::path& saveDir) const
  {
    using boost::filesystem::path;
    assert(is_directory(saveDir) && "no valid directory provided");
    path filename=saveDir / path(_myId) / path(_myId+"_Linemod.yml");

    cv::FileStorage fs(filename.string(), cv::FileStorage::WRITE);

    fs << "mesh_file_path" << mesh_file_path ;

    _detector->write(fs);

    assert(_detector->classIds().size()==1 && "Multiple classes into the same model");
    fs << "object" ;
    fs << "{";
    _detector->writeClass(_myId, fs);
    fs << "}";

    //save : R, T, dist, and Ks for that class
    int nData=_myData.size();
    fs << "Rot" << "[";
    for (int R_idx=0;R_idx<nData;++R_idx)
    {
      fs << _myData[R_idx].R;
    }
    fs << "]";

    fs << "Transl" << "[";
    for (int T_idx=0;T_idx<nData;++T_idx)
    {
      fs << _myData[T_idx].T;
    }
    fs << "]";

    fs << "dist" << "[";
    for (int dist_idx=0;dist_idx<nData;++dist_idx)
    {
      fs << _myData[dist_idx].dist;
    }
    fs << "]";

    fs << "Camera" << "[";
    for (int Ks_idx=0;Ks_idx<nData;++Ks_idx)
    {
      fs << _myData[Ks_idx].cam;
    }
    fs << "]";

    fs << "Hue" << "[";
    for (int Hue_idx=0;Hue_idx<nData;++Hue_idx)
    {
      fs << _myData[Hue_idx].hueHist;
    }
    fs << "]";

    fs << "rendering"  << "{";
    fs << "renderer_width" <<  renderer_width;
    fs << "renderer_height" <<  renderer_height;
    fs << "renderer_focal_length_x" <<  renderer_focal_length_x;
    fs << "renderer_focal_length_y" <<  renderer_focal_length_y;
    fs << "renderer_near" <<  renderer_near;
    fs << "renderer_far" <<  renderer_far;
    fs << "renderer_n_points" <<  renderer_n_points;
    fs << "}";

  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr Model::getPointCloud(const C5G::Pose& pose) const {
    cv::Mat image_out, depth_out, mask_out;
    cv::Rect rect_out;

    /** Gets the T and UP vector from the Pose object */
    Eigen::Affine3d objTransformation=C5G::Pose::poseToTransform(pose);
    cv::Mat t;
    eigen2cv(Eigen::Vector3d(objTransformation.translation()), t);
    Eigen::Matrix3d R_temp(objTransformation.rotation());
    R_temp=R_temp.inverse();
    cv::Vec3d u (-R_temp(0,1), -R_temp(1,1), -R_temp(2,1));
    std::cout << "Up vector: " << u(0) << ","<< u(1) << ","<< u(2) << "\n";

    render(t, u, image_out, depth_out, mask_out, rect_out);

    cv::Mat_<cv::Vec3d> pointsXYZ;
    cv::rgbd::depthTo3d(depth_out, _camModel.getIntrinsic(), pointsXYZ);
    /** Fills model and reference pointClouds with points taken from (X,Y,Z) coordinates */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr modelCloudPtr (new pcl::PointCloud<pcl::PointXYZRGB>);

    /** Model PointCloud*/
    int mySize = pointsXYZ.rows*pointsXYZ.cols;
    modelCloudPtr->resize(mySize);
    modelCloudPtr->height = 1;
    modelCloudPtr->is_dense = true;

    for(int i=0; i<pointsXYZ.rows; ++i)
    {
      for(int j=0; j<pointsXYZ.cols; ++j){
        modelCloudPtr->points[i*pointsXYZ.cols+j].x=pointsXYZ[i][j][0];
        modelCloudPtr->points[i*pointsXYZ.cols+j].y=pointsXYZ[i][j][1];
        modelCloudPtr->points[i*pointsXYZ.cols+j].z=pointsXYZ[i][j][2];
        modelCloudPtr->points[i*pointsXYZ.cols+j].r=0;
        modelCloudPtr->points[i*pointsXYZ.cols+j].g=255;
        modelCloudPtr->points[i*pointsXYZ.cols+j].b=0;
      }
    }
    return modelCloudPtr;
  }
}
