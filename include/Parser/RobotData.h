#pragma once

namespace InterProcessCommunication{
  class RobotData {
    public:
      static RobotData& getInstance();
      std::string getBinItem(int row, int column, int item);
      void setBinItem(int row,int column,int item,const std::string& val);
      static Camera::Image getImageFrame();
      int start();
      int stop();
    private:
      struct Bin {
        std::string object[5];
        bool dirty;
      };
      struct Shelf {
        Bin bins[12];
      };

      Shelf shelf;

      RobotData();
      ~RobotData();

      void operator=(RobotData const&); // Don't implement
      RobotData(RobotData const&);              // Don't Implement

      // C++ 03
      // ========
      // Dont forget to declare these two. You want to make sure they
      // are unacceptable otherwise you may accidentally get copies of
      // your singleton appearing.

      // C++ 11
      // =======
      // We can use the better technique of deleting the methods
      // we don't want.
      //RobotData(RobotData const&)               = delete;
      //void operator=(RobotData const&)  = delete;
  };
}
