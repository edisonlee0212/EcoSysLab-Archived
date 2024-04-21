#include <iostream>
#include "jsSetupConfigParser.hpp"
#include "joescan_pinchot.h"

/// Log output from `jsSetupConfigParse`
void logger(jsError err, std::string msg)
{
  std::cout << msg << std::endl;
  if (0 != err) {
    // If `err` is non-zero, `jsSetupConfigParse` failed parsing or initializing
    // something in the JSON file.
    const char *err_str = nullptr;
    jsGetError(err, &err_str);
    std::cout << "jsError (" << err << "): " << err_str << std::endl;
  }
}

int main(int argc, char *argv[])
{
  if (2 != argc) {
    std::cout << "Usage: " << argv[0] << " FILE" << std::endl;
    return 1;
  }

  // Path to JSON config file.
  std::string path = argv[1];

  // Scan System and Scan Head vector declared here, but uninitialized. Will
  // be initialized in the `jsSetupConfigParse` function.
  jsScanSystem system;
  std::vector<jsScanHead> heads;

  int r = joescan::jsSetupConfigParse(path, system, heads, &logger);

  // Check the results of parsing the configuration file.
  if (0 > r) {
    // The Scan System and Scan Heads should be assumed to be in an
    // indeterminate state; only action to take is to free the Scan System.
    std::cout << "Configuration failed" << std::endl;
    r = 1;
  } else {
    // Scan System and Scan Heads are fully configured.
    std::cout << "Configured successfully" << std::endl;
  }

  // Any user code that needs to be run before scanning should be placed here.
  // After everything is setup and initialized, then connect and begin scanning.

  // Free the Scan System and all of its resources.
  jsScanSystemFree(system);

  return r;
}
