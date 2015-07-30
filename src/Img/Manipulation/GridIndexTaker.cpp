#include <cassert>
#include <boost/property_tree/json_parser.hpp>
#include <Img/Manipulation/GridIndexTaker.h>

namespace Img{
  namespace Manipulation{

    GridIndexTaker::GridIndexTaker(unsigned int width, unsigned int height, coeffs mH, coeffs qH, coeffs mV, coeffs qV)
      :
        _width(width),
        _height(height),
        _mH(mH),
        _qH(qH),
        _mV(mV),
        _qV(qV)
    {
      assert(_mH.size()==(_height+1) && _qH.size()==(_height+1) && mV.size()==(_width+1) && qV.size()==(_width+1));
    }

    GridIndexTaker GridIndexTaker::fromFile(const std::string& filename){
      namespace pt=boost::property_tree;

      /* Read JSON Vector */
      pt::ptree tree;
      pt::read_json(filename, tree);
      pt::ptree grid=tree.get_child("shelfGrid");
      unsigned int width=grid.get<unsigned int>("width");
      unsigned int height=grid.get<unsigned int>("height");
      std::vector<double> mH=readVector<double>(grid,"mH");
      std::vector<double> mV=readVector<double>(grid, "mV");
      std::vector<double> qH=readVector<double>(grid, "qH");
      std::vector<double> qV=readVector<double>(grid, "qV");
      if(mH.size()!=(height+1) || qH.size()!=(height+1) || mV.size()!=(width+1) || qV.size()!=(width+1)){
        throw std::string("Bad save file: "+filename);
      }
      return GridIndexTaker(width, height, mH, qH, mV, qV);
    }

    void GridIndexTaker::toFile(const std::string& filename){
      namespace pt=boost::property_tree;

      /* Read JSON Vector */
      pt::ptree tree;
      pt::ptree grid;
      pt::ptree mHTree, qHTree, mVTree, qVTree;
      grid.put<unsigned int>("width", _width);
      grid.put<unsigned int>("height", _height);

      for(unsigned int i=0; i<=_height; ++i){
        pt::ptree mHVal, qHVal;
        mHVal.put_value(_mH[i]);
        qHVal.put_value(_qH[i]);
        mHTree.push_back(std::make_pair("", mHVal));
        qHTree.push_back(std::make_pair("", qHVal));
      }

      for(unsigned int i=0; i<=_width; ++i){
        pt::ptree mVVal, qVVal;
        mVVal.put_value(_mH[i]);
        qVVal.put_value(_qH[i]);
        mVTree.push_back(std::make_pair("", mVVal));
        qVTree.push_back(std::make_pair("", qVVal));
      }
      grid.put_child("mH", mHTree);
      grid.put_child("qH", qHTree);
      grid.put_child("mV", mVTree);
      grid.put_child("qV", qVTree);
      tree.add_child("shelfGrid", grid);

      pt::write_json(filename, tree);
    }

    void GridIndexTaker::updateCoords(int row, int column){
      double m=_mH[row];
      double q=_qH[row];
      double M=_mV[column];
      double Q=_qV[column];
      _x1=(Q-q)/(m-M);
      _y1=m*_x1+q;
      m=_mH[row+1];
      q=_qH[row+1];
      M=_mV[column+1];
      Q=_qV[column+1];
      _x2=(Q-q)/(m-M);
      _y2=m*_x2+q;
    }
    


  }
}

