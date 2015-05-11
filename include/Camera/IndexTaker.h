#pragma once

namespace Camera{
  class IndexTaker{
    protected:
      int _x1, _x2, _y1, _y2;
    public:
      virtual int getX1() const final;
      virtual int getX2() const final;
      virtual int getY1() const final;
      virtual int getY2() const final;
      virtual void updateCoords(int row, int column) = 0;
  };
}
