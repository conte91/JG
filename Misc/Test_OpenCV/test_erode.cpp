/** From http://opencvexamples.blogspot.com/2013/09/void-erode-inputarray-src-outputarray.html */
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
 
using namespace cv;
using namespace std;
 
int main() {
 
       Mat image,dst;
       image = imread("erode.png", CV_LOAD_IMAGE_COLOR);  
 
       // Create a structuring element
       int erosion_size = 6;  
       Mat element = getStructuringElement(cv::MORPH_CROSS,
              cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
              cv::Point(erosion_size, erosion_size) );
 
       // Apply erosion or dilation on the image
       erode(image,dst,element);  // dilate(image,dst,element);
       
       namedWindow( "Display window", CV_WINDOW_AUTOSIZE );  
       imshow( "Display window", image );                 
 
       namedWindow( "Result window", CV_WINDOW_AUTOSIZE );   
       imshow( "Result window", dst );
 
       waitKey(0);                                         
       return 0;
}

