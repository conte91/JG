#include <iostream>

/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

#define GL_GLEXT_PROTOTYPES

#include <cassert>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include "Recognition/renderer3d_impl_glut.h"

#ifndef NDEBUG
static inline void checkNoErrorCode(){
  const GLubyte * errStr;
  GLenum errorCode=glGetError();
  errStr=gluErrorString(errorCode);
  std::cout << "Error code: " << errorCode << errStr << "\n";
  assert(errorCode==GL_NO_ERROR);
}
#else
static inline void checkNoErrorCode(){
}
#endif
Renderer3dImpl::Renderer3dImpl(int width, int height) :
        Renderer3dImplBase(width, height)
{
};

void
Renderer3dImpl::clean_buffers()
{
  if (color_rbo_id_)
    glDeleteRenderbuffers(1, &color_rbo_id_);
  color_rbo_id_ = 0;

  if (color_rbo_resolve_id_)
    glDeleteRenderbuffers(1, &color_rbo_resolve_id_);
  color_rbo_resolve_id_ = 0;

  if (depth_rbo_id_)
    glDeleteRenderbuffers(1, &depth_rbo_id_);
  depth_rbo_id_ = 0;
  if (depth_rbo_resolve_id_)
    glDeleteRenderbuffers(1, &depth_rbo_resolve_id_);
  depth_rbo_resolve_id_ = 0;

  // clean up FBO, RBO
  if (fbo_id_)
    glDeleteFramebuffers(1, &fbo_id_);
  fbo_id_ = 0;
  if (fbo_resolve_id_)
    glDeleteFramebuffers(1, &fbo_resolve_id_);
  fbo_resolve_id_ = 0;
}


void
Renderer3dImpl::set_parameters_low_level()
{

  /* create a framebuffer object on which the multisampled image will be drawn */
  glGenFramebuffers(1, &fbo_id_);
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);

  /** Create the rendering area for colour and depth information (Renderbuffers) */
  /* Colour */
  glGenRenderbuffers(1, &color_rbo_id_);
  glBindRenderbuffer(GL_RENDERBUFFER, color_rbo_id_);
  glRenderbufferStorageMultisample(GL_RENDERBUFFER, 4, GL_RGBA8, width_, height_);
  //glRenderbufferStorage(GL_RENDERBUFFER,GL_RGBA8,width_,height_);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, color_rbo_id_);
  /* Depth */
  glGenRenderbuffers(1, &depth_rbo_id_);
  glBindRenderbuffer(GL_RENDERBUFFER, depth_rbo_id_);
  glRenderbufferStorageMultisample(GL_RENDERBUFFER, 4, GL_DEPTH_COMPONENT, width_, height_);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_rbo_id_);

  GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER) ;
  if(!(status == GL_FRAMEBUFFER_COMPLETE)){
    std::cout << "GL_FRAMEBUFFER " << status << "\n#" << gluErrorString(status) << "#Failed to make complete framebuffer object!\n" << std::endl;
    assert(false);
  }

  /* Create the resolve framebuffer object */
  glGenFramebuffers(1, &fbo_resolve_id_);
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_resolve_id_);

  /* create the resolve renderbuffer object for depth and colour */
  glGenRenderbuffers(1, &depth_rbo_resolve_id_);
  glGenRenderbuffers(1, &color_rbo_resolve_id_);

  /** Bind depth resolve buffer to resolve frame buffer */
  glBindRenderbuffer(GL_RENDERBUFFER, depth_rbo_resolve_id_);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width_, height_);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_rbo_resolve_id_);

  /** Bind colour resolve buffer to resolve frame buffer */
  glBindRenderbuffer(GL_RENDERBUFFER, color_rbo_resolve_id_);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, width_, height_);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, color_rbo_resolve_id_);

  status = glCheckFramebufferStatus(GL_FRAMEBUFFER) ;

  if(!(status == GL_FRAMEBUFFER_COMPLETE)){
    std::cout << "GL_FRAMEBUFFER " << status << "\n#" << gluErrorString(status) << "#Failed to make complete framebuffer object!\n" << std::endl;
    assert(false);
  }

}

void
Renderer3dImpl::bind_buffers() const
{
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);
  glDrawBuffer(GL_COLOR_ATTACHMENT0);
}

void
Renderer3dImpl::bind_buffers_for_reading() const
{
  glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo_id_);
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo_resolve_id_);
  glReadBuffer(GL_COLOR_ATTACHMENT0);
  glDrawBuffer(GL_COLOR_ATTACHMENT0);
  glBlitFramebuffer(0, 0, width_, height_, 0, 0, width_, height_, GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT, GL_NEAREST);
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_resolve_id_);
}

Renderer3dImpl::~Renderer3dImpl(){
      clean_buffers();
}
