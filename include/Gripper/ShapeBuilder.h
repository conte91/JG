#pragma once

namespace cv{
  void write( FileStorage& fs, const std::string& name, const Gripper::Shape& model);
  void read(const FileNode& node, std::unique_ptr<Gripper::Shape>& res, const std::unique_ptr<Gripper::Shape>& default_value );
}
