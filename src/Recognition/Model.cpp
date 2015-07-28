#include <cassert>
#include <Recognition/Model.h>

namespace Recognition{
  Model::Model(const std::string& id, const boost::filesystem::path& myDir)
    :
      _id(id),
      _detector(new cv::linemod::Detector) 
  {

    namespace fs=boost::filesystem;

    /* Load the yml of that class obtained in the Training phase */
    fs::path lmSaveFile = myDir / fs::path(_id+"_Linemod.yml");
    std::string mesh_file_path;
    //Read mesh_file_path

    cv::FileStorage inFile(lmSaveFile.string(), cv::FileStorage::READ);
    inFile["mesh_file_path"] >> mesh_file_path;
    _detector->read(inFile.root());
    std::cout<<"\tNumber of templates:"<<_detector->numTemplates()<<"\n";


    /** Read the trained class ID for this object */
    cv::FileNode fn = inFile["classes"];
    CV_Assert(fn.size()==1 && "File contains more than one class");
    fn[0]["class_id"] >> _myId;
    _detector->readClass(fn[0]);

    /** Initialize the renderer with the same parameters used for learning */
    std::shared_ptr<Renderer3d> renderer (new Renderer3d(mesh_file_path));
    int renderer_width, renderer_height;
    float renderer_near,  renderer_far,  renderer_focal_length_x, renderer_focal_length_y;
    inFile["renderer_width"] >> renderer_width;
    inFile["renderer_height"] >> renderer_height;
    inFile["renderer_focal_length_x"] >> renderer_focal_length_x;
    inFile["renderer_focal_length_y"] >> renderer_focal_length_y;
    inFile["renderer_near"] >> renderer_near;
    inFile["renderer_far"] >> renderer_far;
    renderer->set_parameters(renderer_width, renderer_height, renderer_focal_length_x, renderer_focal_length_y, renderer_near, renderer_far);

    int renderer_n_points;
    inFile["renderer_n_points"] >> renderer_n_points;
    _renderer_iterator=std::shared_ptr<RendererIterator>(new RendererIterator(renderer, renderer_n_points));
    inFile["renderer_angle_step"] >> _renderer_iterator->angle_step_;
    inFile["renderer_radius_min"] >> _renderer_iterator->radius_min_;
    inFile["renderer_radius_max"] >> _renderer_iterator->radius_max_;
    inFile["renderer_radius_step"] >> _renderer_iterator->radius_step_;

    /**Read R**/
    _Rmap=readSequence<cv::Mat>(inFile["Rot"]);
    /**Read T**/
    _Tmap=readSequence<cv::Vec3d>(inFile["Transl"]);
    /**Read K**/
    _Kmap=readSequence<cv::Mat>(inFile["Ks"]);
    /**Read Dist**/
    _distMap=readSequence<float>(inFile["dist"]);
    /**Read HueHist**/
    _hueHistMap=readSequence<cv::Mat>(inFile["Hue"]);
  }

  const std::vector<cv::linemod::Template> Model::getTemplates(int templateID) const {
    return _detector->getTemplates(_myId, templateID);
  }

  const std::vector<cv::linemod::Template> Model::getAllTemplates() const {
    std::vector<cv::linemod::Template> result;
    for(int i=0; i<_detector->numTemplates(); ++i){
      auto& t=getTemplates(i);
      result.insert(result.end(), t.begin(), t.end());
    }
    return result;
  }

  int Model::numTemplates() const {
    return _detector->numTemplates();
  }
  cv::Mat Model::getR(int templateID) const {
    return _Rmap[templateID];
  }
  cv::Vec3d Model::getT(int templateID) const {
    return _Tmap[templateID];
  }
  float Model::getDist(int templateID) const {
    return _distMap[templateID];
  }
  cv::Mat Model::getK(int templateID) const {
    return _Kmap[templateID];
  }
  cv::Mat Model::getHueHist(int templateID) const {
    return _hueHistMap[templateID];
  }
  const std::shared_ptr<RendererIterator> Model::getRenderer() const{
    return _renderer_iterator;
  }
}
