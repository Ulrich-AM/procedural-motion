#include "test_support.h"

#include "body/proc_body.h"

#include <iostream>

int main(int argc, char** argv) {
  if (argc > 1) {
    proc::ProcBody body;
    const proc::Status status = proc::loadProcBodyFromFile(argv[1], body);
    if (!status.ok()) {
      std::cerr << status.text() << '\n';
      return 1;
    }
    std::cout << "loaded " << body.name << '\n';
    return 0;
  }
  const int body = runBodyTests();
  const int fk = runFkTests();
  const int motion = runMotionTests();
  const int failed = body + fk + motion;
  if (failed != 0) {
    std::cerr << failed << " test(s) failed\n";
    return 1;
  }
  std::cout << "all tests passed\n";
  return 0;
}
