#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>//to publish pcl::PointCloud
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h> //da Hydro in poi
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>//serve per convertire ROS image in Mat image e viceversa
#include <sensor_msgs/image_encodings.h> //indica la codifica dell'immagine (CV_BRIDGE)
#include <image_transport/image_transport.h>//compressione dell'immagine da ricevere
#include <image_geometry/pinhole_camera_model.h>
#include <camera_info_manager/camera_info_manager.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//Conversion Structure mm to meters of the Depth Image
#include <giorgio_latif/depth_traits.h>

//#include <sensor_msgs/point_cloud2_iterator.h>


using namespace depth_image_proc;

class HandDetector
{
   private:
   /** ROS NodeHandle **/
   ros::NodeHandle nh;
   ros::NodeHandle nh_privato;
   
   /** ROS Publisher **/
   ros::Publisher pub_hand_pc;
   //ros::Publisher pub_marker;
   
   /** ROS Subscriber **/
   
   //image transport subsciber
   boost::shared_ptr<image_transport::ImageTransport> it_;
   image_transport::Subscriber sub_depth_mm;
   //ros::Subscriber sub_depth_mm;
   
   ros::Subscriber sub_camera_info_depth;
   
   /** ROS Structures **/
   tf::TransformListener listener;
   image_geometry::PinholeCameraModel cam_model_;
   
   /* OPENCV Structures */
   cv::Mat depth_gray_img;
   cv::Mat depth_color_img;
   
   
   /** ROS Callbacks **/
   void DepthRawCb(const sensor_msgs::Image::ConstPtr& depth_img_mm);
   
   void DepthCameraInfoCb(const sensor_msgs::CameraInfo::ConstPtr& depth_camera_info)
   {
      
      //load the camera model structure with the pinhole model (Intrinsic + distortion coefficients) of the IR camera (depth)
      cam_model_.fromCameraInfo(depth_camera_info);
      
      ROS_INFO("Depth Camera Model Loaded");
      
      //do it once since the camera model is constant
      sub_camera_info_depth.shutdown();
      
      ROS_INFO("Camera Info subscriber shut down");
      
      
   }
   
   /** Class Private Methods **/
   
   
   
   public:
   /** Class Public Methods **/
   
   HandDetector(): nh_privato("~")
   {
      pub_hand_pc=nh.advertise< pcl::PointCloud<pcl::PointXYZ> >("hand_detector/hand",1);
      
      //It Image transport subscriber
      it_.reset(new image_transport::ImageTransport(nh));
      //define compression type
      //image_transport::TransportHints hints("raw", ros::TransportHints(), nh_privato);
      sub_depth_mm = it_->subscribe("/camera/depth/image_raw", 1, &HandDetector::DepthRawCb, this);
      //old way image subscriber (it is not advisable!!! use it instead)
      //sub_depth_mm = nh.subscribe("/camera/depth/image_raw", 1, &HandDetector::DepthRawCb, this);
      
      sub_camera_info_depth = nh.subscribe("/camera/depth/camera_info", 1, &HandDetector::DepthCameraInfoCb, this);
      //pub_marker = nh.advertise<visualization_msgs::Marker>("hand_detector/marker_hand",1);
      
      //openCV image window
      cv::namedWindow( "depth_color_img", CV_WINDOW_AUTOSIZE );
        
   }//End Constructure
   
   ~HandDetector()
   {
      cv::destroyWindow("depth_color_img");
   }
   
   // Handles float or uint16 depths (m or mm respectively)
  template<typename T>
  void convertDepth2PointCloud(const sensor_msgs::ImageConstPtr& depth_msg, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_msg);
   
   
};//End Class

int main(int argc, char** argv)
{

   ros::init(argc,argv,"hand_detector_depth_image");
   HandDetector _HandDetector_;
   ros::spin();
   return 0;
}

void HandDetector::DepthRawCb(const sensor_msgs::Image::ConstPtr& depth_img_mm)
{

   cv_bridge::CvImageConstPtr cv_ptr_depth_mm;
   //DEPTH in mm. Turn Image into cv::Mat
  try
    {
      cv_ptr_depth_mm = cv_bridge::toCvShare(depth_img_mm);
    }
  catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    //depth_image_raw_mm = cv::Mat(cv_ptr_depth->image);
    
    //Turn cv::Mat in mm into cv::Mat in gray (0..255)
    double minVal, maxVal;
    cv::minMaxLoc(cv_ptr_depth_mm->image,&minVal,&maxVal);
    cv_ptr_depth_mm->image.convertTo(depth_gray_img,CV_8U,255.0/(maxVal - minVal), -minVal*255.0/(maxVal-minVal));
    
    //get left hand pose w.r.t. the camera_depth_optical_frame
   //WARNING: The left hand frame == my own right hand !!!!
   //WARNING: Here we consider only 1 user...!!!!
   tf::StampedTransform camera2left_handTransform;
   tf::StampedTransform camera2left_wristTransform;
   try{
         //HAND                        dest_frame  <---  origin_frame    
         listener.waitForTransform(depth_img_mm->header.frame_id,"/left_hand_1", ros::Time(0), ros::Duration(1.0));
         listener.lookupTransform(depth_img_mm->header.frame_id,"/left_hand_1" ,ros::Time(0), camera2left_handTransform);
         //WRIST
         listener.waitForTransform(depth_img_mm->header.frame_id,"/left_wrist_1", ros::Time(0), ros::Duration(1.0));
         listener.lookupTransform(depth_img_mm->header.frame_id,"/left_wrist_1" ,ros::Time(0), camera2left_wristTransform);
         
      }
   catch(tf::TransformException &ex){  
         return;
      }
      
    //my own right hand centroid == origin left_hand_frame   
    cv::Point3f hand_centroid(camera2left_handTransform.getOrigin().x() , camera2left_handTransform.getOrigin().y(), 
      camera2left_handTransform.getOrigin().z());
      
    //my own right wrist centroid == origin left_wrist_frame   
    cv::Point3f wrist_centroid(camera2left_wristTransform.getOrigin().x() , camera2left_wristTransform.getOrigin().y(), 
      camera2left_wristTransform.getOrigin().z());
      
    //Convert depth gray img to depth color (RBG) img
    cv::cvtColor(depth_gray_img, depth_color_img, CV_GRAY2RGB);
          
    /*** HAND CENTROID PROJECTION INTO THE DEPTH IMAGE  ***/
    //return the (u,v) image pixel position
    //Point == Point2i == Point_<int>
    cv::Point uv_hand_centroid = cam_model_.project3dToPixel(hand_centroid);
    
    //draw a filled red circle in the image at the hand centroid
    cv::circle(depth_color_img,uv_hand_centroid,4,cv::Scalar(0,0,255),-1);
    
    
    /*** WRIST CENTROID PROJECTION INTO THE DEPTH IMAGE  ***/
    //return the (u,v) image pixel position
    //Point == Point2i == Point_<int>
    cv::Point uv_wrist_centroid = cam_model_.project3dToPixel(wrist_centroid);
    
    //draw a filled blue circle in the image at the hand centroid
    cv::circle(depth_color_img,uv_wrist_centroid,4,cv::Scalar(255,0,0),-1);
    
    cv::imshow("depth_color_img", depth_color_img);
    
    //needed to refresh the image window
    cv::waitKey(1);   

}



template<typename T>
void HandDetector::convertDepth2PointCloud(const sensor_msgs::ImageConstPtr& depth_msg, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_msg)
{
  // Use correct principal point from calibration
  float center_x = cam_model_.cx();
  float center_y = cam_model_.cy();

  // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
  double unit_scaling = DepthTraits<T>::toMeters( T(1) );
  float constant_x = unit_scaling / cam_model_.fx();
  float constant_y = unit_scaling / cam_model_.fy();
  float bad_point = std::numeric_limits<float>::quiet_NaN();

  pcl::PointCloud<pcl::PointXYZ>::iterator pt_iter = cloud_msg->begin();
  const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
  int row_step = depth_msg->step / sizeof(T);
  for (int v = 0; v < (int)cloud_msg->height; ++v, depth_row += row_step)
  {
    for (int u = 0; u < (int)cloud_msg->width; ++u)
    {
      pcl::PointXYZ& pt = *pt_iter++;
      T depth = depth_row[u];

      // Missing points denoted by NaNs
      if (!DepthTraits<T>::valid(depth))
      {
        pt.x = pt.y = pt.z = bad_point;
        continue;
      }

      // Fill in XYZ
      pt.x = (u - center_x) * depth * constant_x;
      pt.y = (v - center_y) * depth * constant_y;
      pt.z = DepthTraits<T>::toMeters(depth);
    }
  }
}


