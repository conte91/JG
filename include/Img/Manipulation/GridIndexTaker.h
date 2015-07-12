#pragma once

#include <string>
#include <vector>
#include "IndexTaker.h"
#include <boost/property_tree/ptree.hpp>

namespace Img{
  namespace Manipulation{
    class GridIndexTaker : public IndexTaker {
      private:
        typedef std::vector<double> coeffs;
        /** Coordinates of the horizontal and vertical lines y=mx+q for each border of the grid */
        const unsigned int _width, _height;
        const coeffs _mH, _qH, _mV, _qV;

        template<typename T>
          static std::vector<T> readVector(const boost::property_tree::ptree& src, const std::string& key);
      public:
        GridIndexTaker(unsigned int width, unsigned int height, coeffs mH, coeffs qH, coeffs mV, coeffs qV);
        static GridIndexTaker fromFile(const std::string& filename);
        void toFile(const std::string& filename);
        virtual void updateCoords(int row, int column);
    };
  }
}

/** Template implementation details */
#include "GridIndexTaker.hpp"
