#include "htproject/htheader.h"

void myproject::myFunction1() {
  std::cout << "This is myFunction1." << std::endl;
}

void myproject::myFunction2() {
  std::cout << "This is myFunction2." << std::endl;
}

Eigen::Matrix4f myproject::get4x4IdentityMatrix() {
  return Eigen::Matrix4f::Identity();
}

