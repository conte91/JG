#include "Mesh.h"

class PointMesh : public Mesh
{
  public:
    PointMesh(double x, double y, double z);
    virtual void
    Draw() const;
  private:
    double _x;
    double _y;
    double _z;
};

