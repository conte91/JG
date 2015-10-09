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
  static constexpr auto value() -> decltype(mif<N-1, N, VALUES ...>::value()){
    return mif<N-1, N, VALUES...>::value();
  }
};

template<int ... VALUES>
struct mif<0, VALUES... >{
  static constexpr auto value() -> decltype(std::make_tuple(0, VALUES...))  {
    return std::make_tuple(0, VALUES...);
  }
};

/** Gets indeces from 0 to N */
template<int N>
constexpr auto make_indices() -> decltype(mif<N>::value()){
  return mif<N>::value();
}
#if 0

template<char... CHARS>
constexpr std::string strAdded(){
  return std::string{CHARS...}+"mianonna";
}

#endif

int main(){
  constexpr std::string x = strAdded<'A','B','C'>;
  std::cout << x  << "\n";
  return 0;
}

