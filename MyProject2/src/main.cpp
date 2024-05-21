// 멋없게 htproject가 header파일에 붙어있는 것을 볼 수 있음
// 원작자가 왜 `teaser`라는 폴더 내에서 설치를 했는지 알 수 있는 부분
// 그러면 #include <teaser/${파일이름}.h>로 include를 하게 되기 때문
// See https://github.com/MIT-SPARK/TEASER-plusplus/tree/master/teaser
#include <htproject/htheader.h>

int main() {
  myproject::myFunction1();

  auto matrix = myproject::get4x4IdentityMatrix();
  std::cout << "4x4 Identity Matrix:" << std::endl;
  std::cout << matrix << std::endl;
  return 0;
}
