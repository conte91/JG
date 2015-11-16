#include <Recognition/PointMesh.h>

PointMesh::PointMesh(double x, double y, double z)
  :
    _x(x), _y(y), _z(z)
{
}

void
PointMesh::Draw() const
{
  std::cout << "Mia sorella @" << _x << " " << _y << " " << _z << "\n";
  glBegin(GL_POINTS);
  glVertex3d(_x,_y,_z);
  glEnd();
}
