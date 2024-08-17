#ifndef COMMON_IO_H
#define COMMON_IO_H

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <iostream>

#include <vector>

inline Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
template<typename T>
inline void print(const std::vector<T>& vec){
    if (!vec.empty()){
    std::cout<<"[";
    for (T t: vec){
      std::cout << t << ", ";
    }
    std::cout<<"]"<<std::endl;
  }
}

inline void print(){
    std::cout << '\n';
}

template<typename T, typename... Ts>  
inline void print(const T head, Ts... tail){
  std::cout << head <<' ';
  print(tail...);
  // std::cout<<std::endl;
}

// // helper function for eigen formatting
// // cant use the base T print because it overloads with a similar template
template<typename T>
inline void print_eigen(const Eigen::EigenBase<T>& out){
    std::cout << out.derived().format(CleanFmt) << std::endl;
}

// specialisation for vectors
// but for some reason vector Xd is considered a different type as vector 3d.
inline void print_eigen(const Eigen::Vector3d& out){
    print_eigen(out.transpose());
}


// specialisation for eigen datatypes
// template<>
// void print(const Eigen::VectorXd& out){
//     print(out.transpose());
// }

#endif