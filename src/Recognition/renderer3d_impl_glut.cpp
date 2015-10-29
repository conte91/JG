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
  std::cout << "clean_buffers";
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

  std::cout << "set_parameters\n";
  glGetError();
  /* create a framebuffer object on which the multisampled image will be drawn */
  glGenFramebuffers(1, &fbo_id_);
  checkNoErrorCode();
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);
  checkNoErrorCode();

  /** Create the rendering area for colour and depth information (Renderbuffers) */
  /* Colour */
  glGenRenderbuffers(1, &color_rbo_id_);
  checkNoErrorCode();
  glBindRenderbuffer(GL_RENDERBUFFER, color_rbo_id_);
  checkNoErrorCode();
  //glRenderbufferStorageMultisample(GL_RENDERBUFFER, 4, GL_RGBA8, width_, height_);
  glRenderbufferStorage(GL_RENDERBUFFER,GL_RGBA8,width_,height_);
  checkNoErrorCode();
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, color_rbo_id_);
  checkNoErrorCode();
  /* Depth */
  glGenRenderbuffers(1, &depth_rbo_id_);
  checkNoErrorCode();
  glBindRenderbuffer(GL_RENDERBUFFER, depth_rbo_id_);
  checkNoErrorCode();
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width_, height_);
  checkNoErrorCode();
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_rbo_id_);
  checkNoErrorCode();

  GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER) ;
  checkNoErrorCode();
  assert(status == GL_FRAMEBUFFER_COMPLETE || "Failed to make complete framebuffer object!");

  /* Create the resolve framebuffer object */
  glGenFramebuffers(1, &fbo_resolve_id_);
  checkNoErrorCode();
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_resolve_id_);
  checkNoErrorCode();

  /* create the resolve renderbuffer object for depth and colour */
  glGenRenderbuffers(1, &depth_rbo_resolve_id_);
  checkNoErrorCode();
  glGenRenderbuffers(1, &color_rbo_resolve_id_);
  checkNoErrorCode();

  /** Bind depth resolve buffer to resolve frame buffer */
  glBindRenderbuffer(GL_RENDERBUFFER, depth_rbo_resolve_id_);
  checkNoErrorCode();
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width_, height_);
  checkNoErrorCode();
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_rbo_resolve_id_);
  checkNoErrorCode();

  /** Bind colour resolve buffer to resolve frame buffer */
  glBindRenderbuffer(GL_RENDERBUFFER, color_rbo_resolve_id_);
  checkNoErrorCode();
  glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, width_, height_);
  checkNoErrorCode();
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, color_rbo_resolve_id_);
  checkNoErrorCode();

  status = glCheckFramebufferStatus(GL_FRAMEBUFFER) ;
  checkNoErrorCode();

  assert(status == GL_FRAMEBUFFER_COMPLETE || "Failed to make complete framebuffer object!");

}

void
Renderer3dImpl::bind_buffers() const
{
  checkNoErrorCode();
  std::cout << "bind\n";
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);
  checkNoErrorCode();
  std::cout << "end\n";
  glBindRenderbuffer(GL_RENDERBUFFER, depth_rbo_id_);
}

void
Renderer3dImpl::bind_buffers_for_reading() const
{
  checkNoErrorCode();
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
  glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
  std::cout <<"bind_read\n";
  assert(fbo_id_ && fbo_resolve_id_);
  checkNoErrorCode();
  assert(glIsFramebuffer(fbo_id_)==GL_TRUE);
  checkNoErrorCode();
  glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo_id_);
  checkNoErrorCode();
  assert(glCheckFramebufferStatus(GL_READ_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE || "Failed to make complete framebuffer object!");
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo_resolve_id_);
  checkNoErrorCode();
  assert(glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE || "Failed to make complete framebuffer object!");
  checkNoErrorCode();
  glReadBuffer(GL_COLOR_ATTACHMENT0);
  checkNoErrorCode();
  glDrawBuffer(GL_COLOR_ATTACHMENT0);
  checkNoErrorCode();
  assert(glCheckFramebufferStatus(GL_READ_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE || "Failed to make complete framebuffer object!");
  assert(glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE || "Failed to make complete framebuffer object!");
  glBlitFramebuffer(0, 0, width_, height_, 0, 0, width_, height_, GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT, GL_NEAREST);
  checkNoErrorCode();
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_resolve_id_);
  checkNoErrorCode();
  std::cout << "end read\n";
}

Renderer3dImpl::~Renderer3dImpl(){
      clean_buffers();
}
