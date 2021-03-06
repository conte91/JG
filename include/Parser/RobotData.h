#pragma once
#include <Img/Image.h>
#include <Camera/ImageViewer.h>
#include <APC/Order.h>
#include <C5G/Pose.h>

namespace InterProcessCommunication{
  class RobotData {
    public:
      typedef Img::Image Image;
      static const int ROW_N=4;
      static const int COL_N=3;
      static Camera::ImageViewer demoViewer;
      static RobotData& getInstance();
      std::string getBinItem(int row, int column, int item);
      void setBinItem(int row,int column,int item,const std::string& val);
      bool isDirty(int row, int column);
      void setDirty(int row, int column, bool value=true);
      APC::OrderStatus getWorkOrder();
      void setWorkOrder(int row,int column,const std::string& itemName);
      C5G::Pose getObjPose(int row, int column, int item) const;
      void setObjPose(int row,int column,int item,const C5G::Pose& val);
      void setPhoto(int row, int column, const Image& frame);
      Image getFrame(int row, int column);
      Image getPhoto(int row, int column);
      static std::string xyToName(int row, int column);
      int getCurrentRow();
      int getCurrentColumn();
      void setCurrentRow(int row);
      void setCurrentColumn(int column);
      friend std::ostream& operator<< (std::ostream& os, const RobotData& r);
      static const int MAX_ITEM_N=6;
      static Image getImageFrame();

    private:
      struct Bin {
        std::string object[MAX_ITEM_N];
        bool dirty;
        Image photo;
        C5G::Pose objPose[MAX_ITEM_N];
        friend std::ostream& operator<< (std::ostream& os, const InterProcessCommunication::RobotData r);
      };
      struct Shelf {
        Bin bins[12];
      };

      Shelf shelf;
      APC::OrderStatus workOrder;

      /** Returns the index of the bin (into the internal array) corresponding to the (row, column) coordinate */
      static int xyToBin(int row, int column);

      RobotData();
      ~RobotData();

      void operator=(RobotData const&); // Don't implement
      RobotData(RobotData const&);              // Don't Implement

      int _row, _column;
    public:
      Bin getBinItems(int row, int column);
  };
}
