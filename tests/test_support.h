#pragma once

#include <iostream>
#include <string>

struct TestStats {
  int failed = 0;

  void expect(bool condition, const std::string& message) {
    if (!condition) {
      std::cerr << "FAIL " << message << '\n';
      ++failed;
    }
  }
};

int runBodyTests();
int runFkTests();
int runMotionTests();
