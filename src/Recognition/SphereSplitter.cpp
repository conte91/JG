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
        icoV.emplace_back(M_PI/2, 0);
        for(int i=0; i<5; ++i){
          icoV.emplace_back(lat1, 2*M_PI*i/5);
        }
        for(int i=0; i<5; ++i){
          icoV.emplace_back(lat2, M_PI/5+2*M_PI*i/5);
        }
        icoV.emplace_back(-M_PI/2, 0);

        std::vector<Face> faces;

        std::cout << "{\n";
        for(auto i: icoV){
          std::cout << i << "\n";
        }
        std::cout << "}\n";
        auto addV=[&faces, &icoV](int a, int b, int c){
          faces.push_back({icoV[a], icoV[b], icoV[c]});
        };

        // 5 faces around point 0
        addV(0,1,2);
        addV(0,2,3);
        addV(0,3,4);
        addV(0,4,5);
        addV(0,5,1);
        addV(1,6,2);
        addV(2,7,3);
        addV(3,8,4);
        addV(4,9,5);
        addV(5,10,1);
        addV(6,2,7);
        addV(7,3,8);
        addV(8,4,9);
        addV(9,5,10);
        addV(10,1,6);
        addV(11,6,7);
        addV(11,7,8);
        addV(11,8,9);
        addV(11,9,10);
        addV(11,10,6);

        // refine triangles
        size_t nVertex=12;
        assert(faces.size()==20);
        while(nVertex<minimumNPoints) {
          std::vector<Face> newFaces;
          for (auto& tri : faces)
          {
            newFaces.push_back({(tri[0]+tri[1])/2, (tri[0]+tri[2])/2, tri[0]});
            newFaces.push_back({(tri[1]+tri[0])/2, (tri[1]+tri[2])/2, tri[1]});
            newFaces.push_back({(tri[2]+tri[0])/2, (tri[2]+tri[1])/2, tri[2]});
            newFaces.push_back({(tri[2]+tri[0])/2, (tri[2]+tri[1])/2, (tri[1]+tri[0])/2});
          }
          assert(newFaces.size()>faces.size());
          faces=newFaces;

          /** Next suddivision increased by a factor of 4 # of vertices (except for top and bottom)*/
          nVertex=(nVertex-2)*4+2;
        }
        std::cout << "NFaces: " << faces.size() << "\n";

        /* Done, now add vertex to our set 
         * Notice that, coming from the SAME calculations, we CAN use == operator between floats for our set :) */
        for (auto& tri:faces) {
          polarV.insert(tri[0]);
          polarV.insert(tri[1]);
          polarV.insert(tri[2]);
        }
        std::cout << "v: " << polarV.size() << "\n";
        assert(polarV.size()>=minimumNPoints);

        for(auto& v: polarV){
          double t=v[1], p=v[0];
          myPoints.emplace_back(sin(t)*cos(p), sin(t)*sin(p), cos(t));
        }
  }
  const std::vector<Eigen::Vector3d>& SphereSplitter::points(){
    return myPoints;
  }
}
