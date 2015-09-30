#include <iostream>
#include <unordered_set>
#include <vector>
#include <cmath>
#include <Recognition/SphereSplitter.h>

namespace Recognition{
  SphereSplitter::SphereSplitter(unsigned int minimumNPoints){
        std::unordered_set<Vertex> polarV;

        // create 12 vertices of a icosahedron
        double lat1=atan(1.0/2.0);
        double lat2=-atan(1.0/2.0);
        std::vector<Vertex> icoV;
        for(int i=0; i<5; ++i){
          icoV.emplace_back(M_PI/2, M_PI/5.0+2*M_PI*i/5.0);
        }
        for(int i=0; i<5; ++i){
          icoV.emplace_back(lat1, 2*M_PI*i/5.0);
        }
        for(int i=0; i<5; ++i){
          icoV.emplace_back(lat2, M_PI/5.0+2*M_PI*i/5.0);
        }
        for(int i=0; i<5; ++i){
          icoV.emplace_back(-M_PI/2, 2*M_PI*i/5.0);
        }

        std::vector<Face> faces;

        std::cout << "{\n";
        for(auto i: icoV){
          std::cout << i << "\n";
        }
        std::cout << "}\n";
        auto addV=[&faces, &icoV](int a, int b, int c){
          faces.push_back({icoV[a], icoV[b], icoV[c]});
          std::cout << "Adding face:\n" << faces.back()[0] << "\n;\n" << faces.back()[1] << "\n;\n" << faces.back()[2] << "\n\n";
        };

        // 5 faces around point 0
        addV(0,5,6);
        addV(1,6,7);
        addV(2,7,8);
        addV(3,8,9);
        addV(4,9,5);
        addV(5,6,10);
        addV(6,7,11);
        addV(7,8,12);
        addV(8,9,13);
        addV(9,5,14);
        addV(10,11,15);
        addV(10,11,6);
        addV(11,12,16);
        addV(11,12,7);
        addV(12,13,17);
        addV(12,13,8);
        addV(13,14,18);
        addV(13,14,9);
        addV(14,10,19);
        addV(14,10,5);

        // refine triangles
        size_t nVertex=12;
        assert(faces.size()==20);
        const auto& medianPoint=[](const Eigen::Vector2d& x, const Eigen::Vector2d& y) -> Eigen::Vector2d {
          double lon1=x[1];
          double lon2=y[1];
          auto xx=x, yy=y;
          /** If latitude difference is > 180°, take the other side of the arc */
          if(fabs(lon1-lon2)>=M_PI){
            if(xx[1] < yy[1]){
              xx[1]+=2*M_PI;
            }
            else{
              yy[1]+=2*M_PI;
            }
          }
          return (xx+yy)/2.0;
        };

        while(nVertex<minimumNPoints) {
          std::vector<Face> newFaces;
          for (auto& tri : faces)
          {
            auto m01=medianPoint(tri[0],tri[1]);
            auto m12=medianPoint(tri[1],tri[2]);
            auto m02=medianPoint(tri[0],tri[2]);
            newFaces.push_back({m01, m02, tri[0]});
            newFaces.push_back({m12, m01, tri[1]});
            newFaces.push_back({m02, m12, tri[2]});
            newFaces.push_back({m01, m02, m12});
          }
          std::cout << "Old: " << faces.size() << "new: " << newFaces.size() << "\n";
          assert(newFaces.size()>faces.size());
          faces=newFaces;

          /** Next suddivision increased by a factor of 4 # of vertices (except for top and bottom)*/
          nVertex=(nVertex-2)*4+2;
        }
        std::cout << "NFaces: " << faces.size() << "\n";

        /* Done, now add vertex to our set 
         * Notice that, coming from the SAME calculations, we CAN use == operator between floats for our set :) */
        int i=0, j=0;
        for (auto& tri:faces) {
          std::cout << "inserting " << i << "\n";
          i++;
          std::cout << (polarV.insert(tri[0]).second? "Success" : "Nope") << tri[0]<<"\n";
          std::cout << (polarV.insert(tri[1]).second? "Success" : "Nope") << tri[1]<<"\n";
          std::cout << (polarV.insert(tri[2]).second? "Success" : "Nope") << tri[2]<<"\n";
        }
        std::cout << "v: " << polarV.size() << "\n";
        assert(polarV.size()>=minimumNPoints);

        for(auto& v: polarV){
          double lat=v[0], lon=v[1];
          myPoints.emplace(cos(lat)*cos(lon), cos(lat)*sin(lon), sin(lat));
          //myPoints.emplace(lat, lon, 0);
        }
        std::cout << "My points #elements: " << myPoints.size() << "\n";
  }
  const SphereSplitter::UvPoints& SphereSplitter::points() const{
    return myPoints;
  }
}
