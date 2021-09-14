#ifndef __RUN_PARALLEL_H__
#define __RUN_PARALLEL_H__

#include "data_collector/teensy_serial.hpp"
#include "data_collector/natnet_client.hpp"
#include "logger/logger.hpp"
#include "mpc_controller/mpc.hpp"
#include "data_collector/collector.hpp"
#include <thread>
#include <iostream>
#include <mutex>

class ParallelRun {
	public:
		ParallelRun(TeensyCollector*, NatNetCollector*, bool, bool, std::string, std::string, float);
		ParallelRun(TeensyCollector*, NatNetCollector*); // no verbose no log overload
		~ParallelRun();
		std::thread * data_thread;
		std::thread * control_thread;
		std::thread* logger;
		TeensyCollector * teensy;
		NatNetCollector * natnet;
		Logger* logger_obj;
		bool verbose;
		std::vector<std::string> list_of_keys;
		float experiment_time;
		bool log;
		void run_parallel_controller();
		void data_collector_callback();
		void logger_callback();
		void mpc_controller_callback();
		void read_from_common_dictionary(std::string, std::vector<float>&);
		void read_common_dictionary_all(std::map<std::string, std::vector<float>>&);
		void write_common_dictionary(std::string, std::vector<float>);
		std::map<std::string, std::vector<float>> shared_data_map; // this is the shared variable
		std::mutex our_mutex;
};

#endif