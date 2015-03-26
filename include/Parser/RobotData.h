#pragma once
#include <Camera/Image.h>

namespace InterProcessCommunication{
  class RobotData {
    public:
      static RobotData& getInstance();
      std::string getBinItem(int row, int column, int item);
      void setBinItem(int row,int column,int item,const std::string& val);
      bool isDirty(int row, int column);
      void setDirty(int row, int column);
      std::vector<std::string> getWorkOrder();
      void setWorkOrder(int row,int column,const std::string& itemName);
      static Camera::Image getImageFrame();

    private:
      struct Bin {
        std::string object[5];
        bool dirty;
        Camera::Image photo;
      };
      struct Shelf {
        Bin bins[12];
      };

      Shelf shelf;
      std::vector<std::string> workOrder;

      /** Returns the index of the bin (into the internal array) corresponding to the (row, column) coordinate */
      int xyToBin(int row, int column);

      RobotData();
      ~RobotData();

      void operator=(RobotData const&); // Don't implement
      RobotData(RobotData const&);              // Don't Implement
  };
}
