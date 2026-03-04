#ifndef ORB_SLAM3_VERBOSE_H
#define ORB_SLAM3_VERBOSE_H

#include <iostream>
#include <string>

namespace ORB_SLAM3 {

class Verbose {
public:
  enum eLevel {
    VERBOSITY_QUIET = 0,
    VERBOSITY_NORMAL = 1,
    VERBOSITY_VERBOSE = 2,
    VERBOSITY_VERY_VERBOSE = 3,
    VERBOSITY_DEBUG = 4
  };

  static eLevel th;

  static void PrintMess(const std::string &str, eLevel lev) {
    if (lev <= th) {
      std::cout << str << std::endl;
    }
  }

  static void SetTh(eLevel _th) { th = _th; }
};

} // namespace ORB_SLAM3

#endif // ORB_SLAM3_VERBOSE_H
