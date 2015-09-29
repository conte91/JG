/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Vincent Rabaud
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <Camera/CameraModel.h>
#include <GL/glew.h>
#include <GL/gl.h>
#include <Recognition/Renderer3d.h>

#include <iostream>
#include <stdlib.h>

#include <Eigen/Geometry>
#include <unsupported/Eigen/OpenGLSupport>

#include <Recognition/Mesh.h>

#include <boost/date_time/posix_time/posix_time.hpp>

#include <Recognition/renderer3d_impl_glut.h>
#include <highgui.h>

Renderer3d::Renderer3d(const std::string & mesh_path)
    :
      renderer_(new Renderer3dImpl(mesh_path, 0, 0)),
      angle_(0),
      focal_length_x_(0),
      focal_length_y_(0),
      near_(0),
      far_(0),
      model_(new Mesh(mesh_path)),
      scene_list_(0)
{
  // get a handle to the predefined STDOUT log stream and attach
  // it to the logging system. It remains active for all further
  // calls to aiImportFile(Ex) and aiApplyPostProcessing.
  //renderer_.is_glut_initialized_ = true;
  ai_stream_ = new aiLogStream(aiGetPredefinedLogStream(aiDefaultLogStream_STDOUT, NULL));
  aiAttachLogStream(ai_stream_);
}

Renderer3d::~Renderer3d()
{
  // We added a log stream to the library, it's our job to disable it
  // again. This will definitely release the last resources allocated
  // by Assimp.
  aiDetachAllLogStreams();
}

void
Renderer3d::set_parameters(const Camera::CameraModel& cam, double near,
                         double far) {
  renderer_->width_ = cam.getWidth();
  renderer_->height_ = cam.getHeight();

  focal_length_x_ = cam.getFx();
  focal_length_y_ = cam.getFy();
  cx_=cam.getXc();
  cy_=cam.getYc();

  near_ = near;
  far_ = far;

  renderer_->clean_buffers();

  // Initialize the OpenGL context
  renderer_->set_parameters_low_level();
  model_->LoadMesh(renderer_->mesh_path_);

  // Initialize the environment
  glClearColor(0.f, 0.f, 0.f, 1.f);

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0); // Uses default lighting parameters

  glEnable(GL_DEPTH_TEST);

  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
  glEnable(GL_NORMALIZE);
/****/
  //glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
  //glEnable(GL_COLOR_MATERIAL);
/****/
  GLfloat LightAmbient[]= { 0.5f, 0.5f, 0.5f, 2.0f };
  GLfloat LightDiffuse[]= { 2.0f, 2.0f, 2.0f, 2.0f };
  GLfloat LightPosition[]= { 0.0f, -5.0f, 0.0f, 1.0f };
    glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);
    glLightfv(GL_LIGHT1, GL_POSITION, LightPosition);
    glEnable(GL_LIGHT1);

  // Initialize the projection
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

    // These parameters define the final viewport that is rendered into by
    // the camera.
    /* In camera's frame, X axis is left-to-right */
    double L = 0;
    double R = renderer_->width_;
    /** We have downward Y axis in our camera system, so height is at bottom of image and 0 at top */
    double B = renderer_->height_;
    double T = 0;
     
    // near and far clipping planes, these only matter for the mapping from
    // world-spaaace z-coordinate into the depth coordinate for OpenGL
    double N = near;
    double F = far;
     
    // construct an orthographic matrix which maps from projected
    // coordinates to normalized device coordinates in the range
    // [-1, 1].  OpenGL then maps coordinates in NDC to the current
    // viewport
    Eigen::Matrix4d ortho = Eigen::Matrix4d::Zero();
    ortho(0,0) =  2.0/(R-L); ortho(0,3) = -(R+L)/(R-L);
    ortho(1,1) =  2.0/(T-B); ortho(1,3) = -(T+B)/(T-B);
    ortho(2,2) = -2.0/(F-N); ortho(2,3) = -(F+N)/(F-N);
    ortho(3,3) =  1.0;
     
    // construct a projection matrix, this is identical to the 
    // projection matrix computed for the intrinsicx, except an
    // additional row is inserted to map the z-coordinate to
    // OpenGL. 
    Eigen::Matrix4d tproj = Eigen::Matrix4d::Zero();
    tproj(0,0) = focal_length_x_; tproj(0,1) = 0; tproj(0,2) = -cx_;
    tproj(1,1) = focal_length_y_; tproj(1,2) = -cy_;
    tproj(2,2) = (N+F); tproj(2,3) = N*F;
    tproj(3,2) = -1.0;

    // resulting OpenGL frustum is the product of the orthographic
    // mapping to normalized device coordinates and the augmented
    // camera intrinsic matrix
    Eigen::Matrix4d frustum = ortho*tproj;
    // set perspective
    std::cout << "Proj: " << tproj << "\nOrtho: " << ortho << "\nFinal: " << frustum << "\n";
    Eigen::glLoadMatrix(frustum);
    glViewport(0, 0, renderer_->width_, renderer_->height_);
    glDepthRange(near_, far_);

}

void
Renderer3d::lookAt(double x, double y, double z, double upx, double upy, double upz)
{
  renderer_->bind_buffers();

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  gluLookAt(x, y, z, 0, 0, 0, upx, upy, upz);

 //glDisable(GL_CULL_FACE);

  // if the display list has not been made yet, create a new one and
  // fill it with scene contents
  if (scene_list_ == 0)
  {
    scene_list_ = glGenLists(1);
    glNewList(scene_list_, GL_COMPILE);
    // now begin at the root node of the imported data and traverse
    // the scenegraph by multiplying subsequent local transforms
    // together on GL's matrix stack.
    model_->Draw();
    glEndList();
  }

  glCallList(scene_list_);
}

void Renderer3d::setObjectPose(const Eigen::Affine3d& pose){
  renderer_->bind_buffers();

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  Eigen::glMultMatrix(pose);

  // scale the whole asset to fit into our view frustum
  aiVector3D scene_min, scene_max, scene_center;
  model_->get_bounding_box(&scene_min, &scene_max);
  glFrontFace(GL_CW);
  //glDisable(GL_CULL_FACE);
  // if the display list has not been made yet, create a new one and
  // fill it with scene contents
  if (scene_list_ == 0)
  {
    scene_list_ = glGenLists(1);
    glNewList(scene_list_, GL_COMPILE);
    // now begin at the root node of the imported data and traverse
    // the scenegraph by multiplying subsequent local transforms
    // together on GL's matrix stack.
    model_->Draw();
    glEndList();
  }

  glCallList(scene_list_);
}
void Renderer3d::setCameraPose(const Eigen::Affine3d& pose){
  renderer_->bind_buffers();

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  Eigen::glMultMatrix(pose.inverse());//.inverse());

  // scale the whole asset to fit into our view frustum
  //glDisable(GL_CULL_FACE);
  // if the display list has not been made yet, create a new one and
  // fill it with scene contents
  if (scene_list_ == 0)
  {
    scene_list_ = glGenLists(1);
    glNewList(scene_list_, GL_COMPILE);
    // now begin at the root node of the imported data and traverse
    // the scenegraph by multiplying subsequent local transforms
    // together on GL's matrix stack.
    model_->Draw();
    glEndList();
  }

  glCallList(scene_list_);
}

void
Renderer3d::render(cv::Mat &image_out, cv::Mat &depth_out, cv::Mat &mask_out, cv::Rect &rect) const
{
  // Create images to copy the buffers to
  cv::Mat_ < cv::Vec3b > image(renderer_->height_, renderer_->width_);
  cv::Mat_<float> depth(renderer_->height_, renderer_->width_);
  cv::Mat_ < uchar > mask = cv::Mat_ < uchar > ::zeros(cv::Size(renderer_->width_, renderer_->height_));

  glFlush();

  // Get data from the depth/image buffers
  renderer_->bind_buffers();

  // Deal with the RGB image
  glReadBuffer(GL_COLOR_ATTACHMENT0);
  glReadPixels(0, 0, renderer_->width_, renderer_->height_, GL_BGR, GL_UNSIGNED_BYTE, image.ptr());

  // Deal with the depth image
  glReadBuffer(GL_DEPTH_ATTACHMENT);
  glReadPixels(0, 0, renderer_->width_, renderer_->height_, GL_DEPTH_COMPONENT, GL_FLOAT, depth.ptr());

  float zNear = near_, zFar = far_;
  cv::Mat_<float>::iterator it = depth.begin(), end = depth.end();
  float max_allowed_z = zFar * 0.99;

  unsigned int i_min = renderer_->width_, i_max = 0, j_min = renderer_->height_, j_max = 0;
  for (unsigned int j = 0; j < renderer_->height_; ++j)
    for (unsigned int i = 0; i < renderer_->width_; ++i, ++it)
    {
      //need to undo the depth buffer mapping
      //http://olivers.posterous.com/linear-depth-in-glsl-for-real
      *it = 2 * zFar * zNear / (zFar + zNear - (zFar - zNear) * (2 * (*it) - 1));
      if (*it > max_allowed_z){
        *it = 0;
      }
      else
      {
        mask(j, i) = 255;
        // Figure the inclusive bounding box of the mask
        if (j > j_max)
          j_max = j;
        else if (j < j_min)
          j_min = j;
        if (i > i_max)
          i_max = i;
        else if (i < i_min)
          i_min = i;
      }
    }

  // Rescale the depth to be in millimeters
  cv::Mat depth_scale(cv::Size(renderer_->width_, renderer_->height_), CV_16UC1);
  depth.convertTo(depth_scale, CV_16UC1, 1e3);

  // Crop the images, just so that they are smaller to write/read
  if (i_min > 0)
    --i_min;
  if (i_max < renderer_->width_ - 1)
    ++i_max;
  if (j_min > 0)
    --j_min;
  if (j_max < renderer_->height_ - 1)
    ++j_max;
  rect = cv::Rect(i_min, j_min, i_max - i_min + 1, j_max - j_min + 1);

  if ((rect.width <=0) || (rect.height <= 0)) {
    depth_out = cv::Mat();
    image_out = cv::Mat();
    mask_out = cv::Mat();
  } else {
    depth_scale(rect).copyTo(depth_out);
    image(rect).copyTo(image_out);
    mask(rect).copyTo(mask_out);
  }
}

void
Renderer3d::renderDepthOnly(cv::Mat &depth_out, cv::Mat &mask_out, cv::Rect &rect) const
{
  // Create images to copy the buffers to
  cv::Mat_<float> depth(renderer_->height_, renderer_->width_);
  cv::Mat_ < uchar > mask = cv::Mat_ < uchar > ::zeros(cv::Size(renderer_->width_, renderer_->height_));

  glFlush();

  // Get data from the OpenGL buffers
  renderer_->bind_buffers();

  // Deal with the depth image
  glReadBuffer(GL_DEPTH_ATTACHMENT);
  glReadPixels(0, 0, renderer_->width_, renderer_->height_, GL_DEPTH_COMPONENT, GL_FLOAT, depth.ptr());

  float zNear = near_, zFar = far_;
  cv::Mat_<float>::iterator it = depth.begin(), end = depth.end();
  float max_allowed_z = zFar * 0.99;

  unsigned int i_min = renderer_->width_, i_max = 0, j_min = renderer_->height_, j_max = 0;
  for (unsigned int j = 0; j < renderer_->height_; ++j)
    for (unsigned int i = 0; i < renderer_->width_; ++i, ++it)
    {
      //need to undo the depth buffer mapping
      //http://olivers.posterous.com/linear-depth-in-glsl-for-real
      *it = 2 * zFar * zNear / (zFar + zNear - (zFar - zNear) * (2 * (*it) - 1));
      if (*it > max_allowed_z){
        *it = 0;
      }
      else
      {
        mask(j, i) = 255;
        // Figure the inclusive bounding box of the mask
        if (j > j_max)
          j_max = j;
        else if (j < j_min)
          j_min = j;
        if (i > i_max)
          i_max = i;
        else if (i < i_min)
          i_min = i;
      }
    }

  // Rescale the depth to be in millimeters
  cv::Mat depth_scale(cv::Size(renderer_->width_, renderer_->height_), CV_16UC1);
  depth.convertTo(depth_scale, CV_16UC1, 1e3);

  // Crop the images, just so that they are smaller to write/read
  if (i_min > 0)
    --i_min;
  if (i_max < renderer_->width_ - 1)
    ++i_max;
  if (j_min > 0)
    --j_min;
  if (j_max < renderer_->height_ - 1)
    ++j_max;
  rect = cv::Rect(i_min, j_min, i_max - i_min + 1, j_max - j_min + 1);

  if ((rect.width <=0) || (rect.height <= 0)) {
    depth_out = cv::Mat();
    mask_out = cv::Mat();
  } else {
    depth_scale(rect).copyTo(depth_out);
    mask(rect).copyTo(mask_out);
  }
}

void
Renderer3d::renderImageOnly(cv::Mat &image_out, const cv::Rect &rect) const
{
  // Create images to copy the buffers to
  cv::Mat_ < cv::Vec3b > image(renderer_->height_, renderer_->width_);

  glFlush();

  // Get data from the OpenGL buffers
  renderer_->bind_buffers();

  // Deal with the RGB image
  glReadBuffer(GL_COLOR_ATTACHMENT0);
  glReadPixels(0, 0, renderer_->width_, renderer_->height_, GL_BGR, GL_UNSIGNED_BYTE, image.ptr());

  if ((rect.width <=0) || (rect.height <= 0)) {
    image_out = cv::Mat();
  } else {
    image(rect).copyTo(image_out);
  }
}
