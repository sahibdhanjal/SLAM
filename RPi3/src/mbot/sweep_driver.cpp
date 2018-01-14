// Make use of the CMake build system or compile manually, e.g. with:
// g++ -std=c++11 example.cc -lsweep


#include <signal.h>
#include <sys/time.h>

#include <cstdlib>
#include <iostream>
#include <cmath>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/lidar_t.hpp>

#include <common/sweep.hpp>
#include <common/lcm_config.h> 
#include <common/timestamp.h>

bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

int main(int argc, char* argv[]) try {
  const char * opt_com_path = NULL;
  int32_t rotationHZ = 5;
  int32_t sampleHZ = 1000;
  ctrl_c_pressed = false;

  lcm::LCM lcmConnection(MULTICAST_URL);

  if(!lcmConnection.good()){
      return 1;
  }

  //get rotation rate if specified...
  if (argc > 1) rotationHZ = atoi(argv[1]);
  //get sample rte if specified...
  if (argc > 2) sampleHZ = atoi(argv[2]);
  //get serial port name if specified...
  if (argc > 3) opt_com_path = argv[3]; 

    if (!opt_com_path) {
#ifdef _WIN32
        // use default com port
        opt_com_path = "\\\\.\\com3";
#else
        opt_com_path = "/dev/ttyUSB0";
#endif
    }

  std::cout << "Constructing sweep device..." << std::endl;
  sweep::sweep device{opt_com_path};

  std::cout << "setting rotation rate to: " << rotationHZ << " HZ, and sample rate to: " << sampleHZ << " HZ\n";

  if(rotationHZ != device.get_motor_speed())  device.set_motor_speed(rotationHZ);
  if(sampleHZ != device.get_sample_rate()){
	 std::cout << "setting sample rate to: " << sampleHZ << " HZ.\n";
	 device.set_sample_rate(sampleHZ);
  }
  std::cout << "Motor Speed Setting: " << device.get_motor_speed() << " Hz" << std::endl;
  std::cout << "Sample Rate Setting: " << device.get_sample_rate() << " Hz" << std::endl;

  assert(device.get_motor_speed() != 0);

  std::cout << "Beginning data acquisition as soon as motor speed stabilizes..." << std::endl;

  signal(SIGINT, ctrlc);
  signal(SIGTERM, ctrlc);

  device.start_scanning();

  int64_t now;
  int pos = 0;

/** for testing speed changes
int updateHZ_count = 0;
int speeds[] = {5, 8, 3, 10};
int speed_at = 0;
float stab_time = 0.0f;
uint64_t changed_time = 0;
int prev_size[] = {0, 0, 0};
bool stabilized = false;
**/

  while(1) {
    const sweep::scan scan = device.get_scan();

//    std::cout << "\nnew scan:\n";

    now = utime_now();

    lidar_t newLidar;

    newLidar.utime = now;
    newLidar.num_ranges = scan.samples.size();

    newLidar.ranges.resize(newLidar.num_ranges);
    newLidar.thetas.resize(newLidar.num_ranges);
    newLidar.intensities.resize(newLidar.num_ranges);
    newLidar.times.resize(newLidar.num_ranges);

/**for testing speed changes
    updateHZ_count++;
    if(updateHZ_count > 100){
      updateHZ_count = 0;

      std::cout << "set speed to: " << speeds[speed_at] << "\n";

      device.set_motor_speed(speeds[speed_at]);

      speed_at++;
      if(speed_at > 3) speed_at = 0;

      prev_size[0] = 0;
      prev_size[1] = 0;
      prev_size[2] = 0;
      changed_time = now;

      stabilized = false;
    }


    if(!stabilized && newLidar.num_ranges == prev_size[0] && prev_size[0] == prev_size[1] && prev_size[1] == prev_size[2]){
      prev_size[0] = 0;
      prev_size[1] = 0;
      prev_size[2] = 0;

      stab_time = float(now - changed_time) / 1000.0f;

      std::cout << "stabilization time: " << stab_time << "\n";

      stabilized = true;
    }else{
        prev_size[2] = prev_size[1];
        prev_size[1] = prev_size[0];
        prev_size[0] = newLidar.num_ranges;
    }
**/
//    std::cout << newLidar.num_ranges << "\n";

    pos = 0;
    for (const sweep::sample& sample : scan.samples) {
      now = utime_now();

      newLidar.ranges[pos] = sample.distance / 100.0f;
      newLidar.thetas[pos] = (2 * M_PI) - (sample.angle * M_PI / 180000.0f);
      newLidar.intensities[pos] = sample.signal_strength;
      newLidar.times[pos] = now;

//      std::cout << "angle " << newLidar.thetas[pos] << " distance " << newLidar.ranges[pos] << " strength " << newLidar.intensities[pos] << "\n";

      ++pos;
    }

    lcmConnection.publish("LIDAR", &newLidar);

    if (ctrl_c_pressed){ 
      break;
    }
  }

  device.stop_scanning();
} catch (const sweep::device_error& e) {
  std::cerr << "Error: " << e.what() << std::endl;
}
