#pragma once

namespace InterProcessCommunication{
  class RobotData {
    public:
      static RobotData& getInstance();
      std::string getBinItem(int row, int column, int item);
      void setBinItem(int row,int column,int item,const std::string& val);
      static Camera::Image getImageFrame();

    private:
      struct Bin {
        std::string object[5];
      };
      struct Shelf {
        Bin bins[12];
      };

      Shelf shelf;

      RobotData();
      ~RobotData();

      void operator=(RobotData const&); // Don't implement
      RobotData(RobotData const&);              // Don't Implement
  };
}
