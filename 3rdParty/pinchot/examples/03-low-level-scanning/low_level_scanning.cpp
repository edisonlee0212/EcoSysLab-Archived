/**
 * Copyright (c) JoeScan Inc. All Rights Reserved.
 *
 * Licensed under the BSD 3 Clause License. See LICENSE.txt in the project
 * root for license information.
 */

/**
 * @file low_level_scanning.cpp
 * @brief Example demonstrating how to read profile data from scan heads
 * individually.
 *
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * NOTE: Frame Scanning is the recommended mode of scanning for most
 * applications. The below example is provided for legacy purposes and
 * for customers requiring low level access.
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *
 * While Frame Scanning is recommended for most applications, users looking
 * for greater manual control of their scan heads, or needing to distribute the
 * CPU load across multiple cores, can make use of "Low Level Scanning". When
 * scanning in this manner, the profiles are returned individually for each
 * scan head and it is the responsibility of the developer to reassemble the
 * profiles. It is recommened with Low Level Scanning mode to create a thread
 * for each individual scan head in order to achieve the highest performance.
 */

#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
#include "joescan_pinchot.h"

static std::mutex lock;
static uint64_t received_profiles = 0;

class ApiError : public std::runtime_error {
 private:
  jsError m_return_code;

 public:
  ApiError(const char* what, int32_t return_code) : std::runtime_error(what)
  {
    if ((0 < return_code) || (JS_ERROR_UNKNOWN > return_code)) {
      m_return_code = JS_ERROR_UNKNOWN;
    } else {
      m_return_code = (jsError) return_code;
    }
  }

  jsError return_code() const
  {
    return m_return_code;
  }
};

/**
 * @brief This function is a small utility function used to explore profile
 * data. In this case, it will iterate over the valid profile data and
 * find the highest measurement in the Y axis.
 *
 * @param profiles Array of profiles from a single scan head.
 * @param num_profiles Total number of profiles contained in array.
 * @return jsProfileData The profile measurement with the greatest Y axis value
 * from the array of profiles passed in.
 */
jsProfileData find_scan_profile_highest_point(jsProfile *profiles,
                                              uint32_t num_profiles)
{
  jsProfileData p = {0, 0, 0};

  for (unsigned int i = 0; i < num_profiles; i++) {
    for (unsigned int j = 0; j < profiles[i].data_len; j++) {
      if (profiles[i].data[j].y > p.y) {
        p.brightness = profiles[i].data[j].brightness;
        p.x = profiles[i].data[j].x;
        p.y = profiles[i].data[j].y;
      }
    }
  }

  return p;
}

/**
 * @brief This function receives profile data from a given scan head. We start
 * a thread for each scan head to pull out the data as fast as possible.
 *
 * @param scan_head Reference to the scan head to recieve profile data from.
 */
static void receiver(jsScanHead scan_head)
{
  // Allocate memory to recieve profile data for this scan head.
  const int max_profiles = 100;
  jsProfile *profiles = nullptr;
  profiles = new jsProfile[max_profiles];
  auto id = jsScanHeadGetId(scan_head);

  {
    std::lock_guard<std::mutex> guard(lock);
    std::cout << "begin receiving on scan head ID " << id << std::endl;
  }

  // For this example, we'll grab some profiles and then act on the data before
  // repeating this process again. Note that for high performance applications,
  // printing to standard out while receiving data should be avoided as it
  // can add significant latency. This example only prints to standard out to
  // provide some illustrative feedback to the user, indicating that data
  // is actively being worked on in multiple threads.
  uint32_t timeout_us = 1000000;
  int32_t r = 0;
  do {
    jsScanHeadWaitUntilProfilesAvailable(scan_head, max_profiles, timeout_us);
    r = jsScanHeadGetProfiles(scan_head, profiles, max_profiles);
    if (0 < r) {
      auto p = find_scan_profile_highest_point(profiles, r);
      {
        std::lock_guard<std::mutex> guard(lock);
        std::cout << "highest point for scan head ID " << id << " is x=" << p.x
                  << ",y=" << p.y << ",brightness=" << p.brightness
                  << std::endl;
        received_profiles += r;
      }
    }
  } while (0 < r);

  {
    std::lock_guard<std::mutex> guard(lock);
    std::cout << "end receiving on scan head ID " << id << std::endl;
  }

  delete[] profiles;
}

/**
 * @brief Creates a basic phase table using all the scan heads managed by the
 * scan system.
 *
 * @param scan_system Reference to the scan system.
 * @param scan_heads Reference to vector of all created scan heads.
 */
void initialize_phase_table(jsScanSystem &scan_system,
                            std::vector<jsScanHead> &scan_heads)
{
  int32_t r = 0;

  // Assume that the system is comprised of scan heads that are all the same.
  jsScanHeadType type = jsScanHeadGetType(scan_heads[0]);

  // For this example we will create a phase table that interleaves lasers
  // seen by Camera A and Camera B. This allows fast and efficient scanning
  // by allowing one camera to scan while the other has the scan data read out
  // & processed; if the same camera is used back to back, a time penalty
  // will be incurred to ensure scan data isn't overwritten.
  switch (type) {
  case (JS_SCAN_HEAD_JS50X6B20):
  case (JS_SCAN_HEAD_JS50X6B30):
    // Phase | Laser | Camera
    //   1   |   1   |   B
    //   2   |   4   |   A
    //   3   |   2   |   B
    //   4   |   5   |   A
    //   5   |   3   |   B
    //   6   |   6   |   A

    for (int n = 0; n < 3; n++) {
      jsLaser laser = JS_LASER_INVALID;

      // Lasers associated with Camera B
      r = jsScanSystemPhaseCreate(scan_system);
      if (0 != r) {
        throw ApiError("failed to create phase", r);
      }

      laser = (jsLaser) (JS_LASER_1 + n);
      for (auto scan_head : scan_heads) {
        r = jsScanSystemPhaseInsertLaser(scan_system, scan_head, laser);
        if (0 != r) {
          throw ApiError("failed to insert into phase", r);
        }
      }

      // Lasers associated with Camera A
      r = jsScanSystemPhaseCreate(scan_system);
      if (0 != r) {
        throw ApiError("failed to create phase", r);
      }

      laser = (jsLaser) (JS_LASER_4 + n);
      for (auto scan_head : scan_heads) {
        r = jsScanSystemPhaseInsertLaser(scan_system, scan_head, laser);
        if (0 != r) {
          throw ApiError("failed to insert into phase", r);
        }
      }
    }
    break;

  case (JS_SCAN_HEAD_JS50Z820):
  case (JS_SCAN_HEAD_JS50Z830):
    // Phase | Laser | Camera
    //   1   |   1   |   B
    //   2   |   5   |   A
    //   3   |   2   |   B
    //   4   |   6   |   A
    //   5   |   3   |   B
    //   6   |   7   |   A
    //   7   |   4   |   B
    //   8   |   8   |   A

    for (int n = 0; n < 4; n++) {
      jsLaser laser = JS_LASER_INVALID;

      // Lasers associated with Camera B
      r = jsScanSystemPhaseCreate(scan_system);
      if (0 != r) {
        throw ApiError("failed to create phase", r);
      }

      laser = (jsLaser) (JS_LASER_1 + n);
      for (auto scan_head : scan_heads) {
        r = jsScanSystemPhaseInsertLaser(scan_system, scan_head, laser);
        if (0 != r) {
          throw ApiError("failed to insert into phase", r);
        }
      }

      // Lasers associated with Camera A
      r = jsScanSystemPhaseCreate(scan_system);
      if (0 != r) {
        throw ApiError("failed to create phase", r);
      }

      laser = (jsLaser) (JS_LASER_5 + n);
      for (auto scan_head : scan_heads) {
        r = jsScanSystemPhaseInsertLaser(scan_system, scan_head, laser);
        if (0 != r) {
          throw ApiError("failed to insert into phase", r);
        }
      }
    }
    break;

  case (JS_SCAN_HEAD_JS50WSC):
  case (JS_SCAN_HEAD_JS50MX):
    // Phase | Laser | Camera
    //   1   |   1   |   A

    r = jsScanSystemPhaseCreate(scan_system);
    if (0 != r) {
      throw ApiError("failed to create phase", r);
    }

    for (auto scan_head : scan_heads) {
      r = jsScanSystemPhaseInsertCamera(scan_system, scan_head, JS_CAMERA_A);
      if (0 != r) {
        throw ApiError("failed to insert into phase", r);
      }
    }
    break;

  case (JS_SCAN_HEAD_JS50WX):
    // Phase | Laser | Camera
    //   1   |   1   |   A
    //   2   |   1   |   B

    r = jsScanSystemPhaseCreate(scan_system);
    if (0 != r) {
      throw ApiError("failed to create phase", r);
    }

    for (auto scan_head : scan_heads) {
      r = jsScanSystemPhaseInsertCamera(scan_system, scan_head, JS_CAMERA_A);
      if (0 != r) {
        throw ApiError("failed to insert into phase", r);
      }
    }

    r = jsScanSystemPhaseCreate(scan_system);
    if (0 != r) {
      throw ApiError("failed to create phase", r);
    }

    for (auto scan_head : scan_heads) {
      r = jsScanSystemPhaseInsertCamera(scan_system, scan_head, JS_CAMERA_B);
      if (0 != r) {
        throw ApiError("failed to insert into phase", r);
      }
    }
    break;

  case (JS_SCAN_HEAD_INVALID_TYPE):
  default:
    throw ApiError("invalid scan head type", 0);
  }
}

/**
 * @brief Initializes and configures scan heads.
 *
 * @param scan_system Reference to the scan system.
 * @param scan_heads Reference to vector to be filled with created scan heads.
 * @param serial_numbers Reference to vector of serial numbers of scan heads
 * to initialize.
 */
void initialize_scan_heads(jsScanSystem &scan_system,
                           std::vector<jsScanHead> &scan_heads,
                           std::vector<uint32_t> &serial_numbers)
{
  int32_t r = 0;

  jsScanHeadConfiguration config;
  config.camera_exposure_time_min_us = 10000;
  config.camera_exposure_time_def_us = 47000;
  config.camera_exposure_time_max_us = 900000;
  config.laser_on_time_min_us = 100;
  config.laser_on_time_def_us = 100;
  config.laser_on_time_max_us = 1000;
  config.laser_detection_threshold = 120;
  config.saturation_threshold = 800;
  config.saturation_percentage = 30;

  // Create a scan head for each serial number passed in on the command line
  // and configure each one with the same parameters. Note that there is
  // nothing stopping users from configuring each scan head independently.
  for (unsigned int i = 0; i < serial_numbers.size(); i++) {
    uint32_t serial = serial_numbers[i];
    auto scan_head = jsScanSystemCreateScanHead(scan_system, serial, i);
    if (0 > scan_head) {
      throw ApiError("failed to create scan head", scan_head);
    }
    scan_heads.push_back(scan_head);

    uint32_t major, minor, patch;
    r = jsScanHeadGetFirmwareVersion(scan_head, &major, &minor, &patch);
    if (0 > r) {
      throw ApiError("failed to read firmware version", r);
    }

    std::cout << serial << " v" << major << "." << minor << "." << patch
              << std::endl;

    r = jsScanHeadSetConfiguration(scan_head, &config);
    if (0 > r) {
      throw ApiError("failed to configure scan head", r);
    }

    r = jsScanHeadSetWindowRectangular(scan_head, 20.0, -20.0, -20.0, 20.0);
    if (0 > r) {
      throw ApiError("failed to set scan window", r);
    }

    r = jsScanHeadSetAlignment(scan_head, 0.0, 0.0, 0.0);
    if (0 > r) {
      throw ApiError("failed to set alignment", r);
    }
  }
}

int main(int argc, char *argv[])
{
  jsScanSystem scan_system;
  std::vector<jsScanHead> scan_heads;
  std::thread *threads = nullptr;
  int32_t r = 0;

  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " SERIAL..." << std::endl;
    return 0;
  }

  std::vector<uint32_t> serial_numbers;
  for (int i = 1; i < argc; i++) {
    serial_numbers.emplace_back(strtoul(argv[i], NULL, 0));
  }

  {
    const char *version_str;
    jsGetAPIVersion(&version_str);
    std::cout << "joescanapi " << version_str << std::endl;
  }

  try {
    scan_system = jsScanSystemCreate(JS_UNITS_INCHES);
    if (0 > scan_system) {
      throw ApiError("failed to create scan system", scan_system);
    }

    // Initialize all the scan heads in the scan system and configure them
    // for scanning.
    initialize_scan_heads(scan_system, scan_heads, serial_numbers);

    // Now that the scan heads are configured, we'll connect to the heads.
    r = jsScanSystemConnect(scan_system, 10);
    if (0 > r) {
      // This error condition indicates that something wrong happened during
      // the connection process itself and should be understood by extension
      // that none of the scan heads are connected.
      throw ApiError("failed to connect", r);
    } else if (jsScanSystemGetNumberScanHeads(scan_system) != r) {
      // On this error condition, connection was successful to some of the scan
      // heads in the system. We can query the scan heads to determine which
      // one successfully connected and which ones failed.
      for (auto scan_head : scan_heads) {
        if (false == jsScanHeadIsConnected(scan_head)) {
          uint32_t serial = jsScanHeadGetSerial(scan_head);
          std::cout << serial << " is NOT connected" << std::endl;
        }
      }
      throw ApiError("failed to connect to all scan heads", 0);
    }

    // Initialize the phase table for the scan system. This will schedule
    // scanning time for all of the scan heads.
    initialize_phase_table(scan_system, scan_heads);

    // Once the phase table is created, we can then read the minimum scan 
    // period of the scan system. This value depends on how many phases there
    // are in the phase table and each scan head's laser on time & window
    // configuration.
    int32_t min_period_us = jsScanSystemGetMinScanPeriod(scan_system);
    if (0 >= min_period_us) {
      throw ApiError("failed to read min scan period", min_period_us);
    }
    std::cout << "min scan period is " << min_period_us << " us" << std::endl;

    std::cout << "start scanning" << std::endl;
    jsDataFormat data_format = JS_DATA_FORMAT_XY_BRIGHTNESS_FULL;
    r = jsScanSystemStartScanning(scan_system, min_period_us, data_format);
    if (0 > r) {
      throw ApiError("failed to start scanning", r);
    }

    // In order to achieve a performant application, we'll create a thread
    // for each scan head. This allows the CPU load of reading out profiles
    // to be distributed across all the cores available on the system rather
    // than keeping the heavy lifting in an application within a single process.
    threads = new std::thread[scan_heads.size()];
    for (unsigned long i = 0; i < scan_heads.size(); i++) {
      threads[i] = std::thread(receiver, scan_heads[i]);
    }

    // Put this thread to sleep until the total scan time is done.
    unsigned long int scan_time_sec = 10;
    std::this_thread::sleep_for(std::chrono::seconds(scan_time_sec));

    r = jsScanSystemStopScanning(scan_system);
    if (0 > r) {
      throw ApiError("failed to stop scanning", r);
    }

    for (unsigned long i = 0; i < scan_heads.size(); i++) {
      threads[i].join();
    }
    delete[] threads;
    threads = nullptr;
    std::cout << "stop scanning" << std::endl;

    // We can verify that we received all of the profiles sent by the scan
    // heads by reading each scan head's status message and summing up the
    // number of profiles that were sent. If everything went well and the
    // CPU load didn't exceed what the system can manage, this value should
    // be equal to the number of profiles we received in this application.
    jsScanHeadStatus status;
    unsigned int expected = 0;
    for (auto scan_head : scan_heads) {
      r = jsScanHeadGetStatus(scan_head, &status);
      if (0 > r) {
        throw ApiError("failed to obtain status message", r);
      }
      expected += status.num_profiles_sent;
    }
    std::cout << "received " << received_profiles << " profiles" << std::endl;
    std::cout << "expected " << expected << " profiles" << std::endl;

    r = jsScanSystemDisconnect(scan_system);
    if (0 > r) {
      throw ApiError("failed to disconnect", r);
    }
  } catch (ApiError &e) {
    std::cout << "ERROR: " << e.what() << std::endl;
    r = 1;

    const char *err_str = nullptr;
    jsError err = e.return_code();
    if (JS_ERROR_NONE != err) {
      jsGetError(err, &err_str);
      std::cout << "jsError (" << err << "): " << err_str << std::endl;
    }
  }

  // Free memory allocated for threads if not already done.
  if (nullptr != threads) {
    for (unsigned long i = 0; i < scan_heads.size(); i++) {
      threads[i].join();
    }
    delete[] threads;
  }

  jsScanSystemFree(scan_system);

  return (0 == r) ? 0 : 1;
}
