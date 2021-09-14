#include "data_collector/natnet_client.hpp"
#include "data_collector/teensy_serial.hpp"
#include "run_parallel/run_parallel.hpp"
#include <iostream>
#include <chrono>
/*
	rb_data_map =
	{
		"Arm" : std::vector<float> {0.0, 0.0, 0.0},
		"Pivot" : std::vector<float> {0.0, 0.0, 0.0}
	}
*/

int main()
{
	// 192.168.50.164
	NatNetCollector natnet("192.168.50.10", "192.168.50.164", 3); // host_ip, client_ip, buffer_size
	natnet.add_rigid_body("Arm");
	natnet.add_rigid_body("Pivot");
	natnet.initialize_client_thread(); // threaded function
	TeensyCollector teensy("COM5", 4, 115200); // serial_port (string not char*), number of values read for each sample

	ParallelRun khois_mpc_controller_full(&teensy, &natnet, false, true, "csv", "output_file", 300); 
	// (teensy_object, natnet_object, verbose = false, log = true, format = "csv", file_name_prefix = "output_file", log_freq_in_hz = 300)
	khois_mpc_controller_full.run_parallel_controller();

    return 0;
}