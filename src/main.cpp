#include "app/application.h"

#include <iostream>

int main(int argc, char** argv) {
  try {
    pm::Application app;
    return app.run(argc, argv);
  } catch (const std::exception& exception) {
    std::cerr << exception.what() << "\n";
    return 1;
  }
}
