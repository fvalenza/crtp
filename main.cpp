#include <iostream>
#include "include/skew.hpp"
// #include "crtp-base-interface.hpp"
// #include "crtp-traits-interface.hpp"
#include "include/se3all-crtp.hpp"
// #include "include/forceall.hpp"
// #include "include/inertiaall.hpp"
// #include "include/motionall.hpp"
// #include <boost/variant/variant_fwd.hpp>

// #include "force-base.hpp" 
// clang++ -I/usr/include/eigen3 main.cpp -o main
// g++ -I/usr/include/eigen3 main.cpp -o main 
// g++ -I/usr/include/eigen3 main.cpp -lboost_unit_test_framework -o main

// #define BOOST_MPL_LIMIT_LIST_SIZE 30  // see http://boost.2283326.n4.nabble.com/using-boost-variant-with-20-or-more-variant-types-td3510610.html
// #define BOOST_MPL_LIMIT_VECTOR_SIZE 30 // influe pas sur BOOST_VARIANT_LIMIT_TYPES
// #define BOOST_VARIANT_LIMIT_TYPES 150 //http://www.boost.org/doc/libs/1_53_0/doc/html/BOOST_VARIANT_LIMIT_TYPES.html  std::cout << BOOST_VARIANT_LIMIT_TYPES << std::endl;  DEFAULT : 20
int main()
{
  // using namespace se3;

  Derived<double,0> t;
  new_type tt;

  return 0;
}
