#include <iostream>
#include <string>
#include <tuple>
#include <functional>
#if 0
#include "Options.h"

const char* xxx="ciao";

Utils::Options<int, 3, xxx, int, 2, xxx, int, 1, xxx> a;
#endif

/** Values are indeces from N+1 to something */
template<int N, int ... VALUES >
struct mif{
  static constexpr decltype(auto) value(){
    return mif<N-1, N, VALUES...>::value();
  }
};

template<int ... VALUES>
struct mif<0, VALUES... >{
  static constexpr decltype(auto) value(){
    return std::make_tuple(0, VALUES...);
  }
};

/** Gets indeces from 0 to N */
template<int N>
constexpr decltype(auto) make_indices(){
  return mif<N>::value();
}

int main(){
  auto x=make_indices<5>();
  return 0;
}

