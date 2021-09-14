#include "run_parallel/run_parallel.hpp"
#include <chrono>
#include <iomanip>      // std::setprecision
#include <iostream>     // std::cout, std::fixed
#include <thread>

float reference_generator(int ref_counter, std::vector < float > steps, float off_time, float on_time, float LOOP_FREQUENCY);

ParallelRun::ParallelRun(TeensyCollector* teensy, NatNetCollector* natnet, bool verbose, bool log, std::string logfile_format, std::string logfile_name, float log_rate_hz) {
	
	this->teensy = teensy;
	this->natnet = natnet;
	this->log = log;
	this->verbose = verbose;
	std::vector<float> initial_data = { -1 };
	this->shared_data_map.insert(std::pair<std::string, std::vector<float>>("arm", { -1, -1, -1 }));
	this->shared_data_map.insert(std::pair<std::string, std::vector<float>>("pivot", { -1, -1, -1 }));
	this->shared_data_map.insert(std::pair<std::string, std::vector<float>>("teensy_write", { -1, -1, -1 }));
	this->shared_data_map.insert(std::pair<std::string, std::vector<float>>("teensy_read", { -1, -1, -1, -1 }));
	this->shared_data_map.insert(std::pair<std::string, std::vector<float>>("write_flag", initial_data));
	this->shared_data_map.insert(std::pair<std::string, std::vector<float>>("read_flag", initial_data));
	this->shared_data_map.insert(std::pair<std::string, std::vector<float>>("processed_vel", initial_data));
	this->shared_data_map.insert(std::pair<std::string, std::vector<float>>("reference_vel", initial_data));
	this->shared_data_map.insert(std::pair<std::string, std::vector<float>>("current_Teensy_data", { -1, -1, -1, -1 }));
	this->shared_data_map.insert(std::pair<std::string, std::vector<float>>("previous_Teensy_data", { -1, -1, -1, -1 }));
	this->shared_data_map.insert(std::pair<std::string, std::vector<float>>("first_Teensy_data", { -1, -1, -1, -1 }));

	this->data_thread = new std::thread(&ParallelRun::data_collector_callback, this);
	this->control_thread = new std::thread(&ParallelRun::mpc_controller_callback, this);
	
	if (this->log) {
		this->logger_obj = new Logger(logfile_format, logfile_name, log_rate_hz);
		this->logger = new std::thread(&ParallelRun::logger_callback, this);
		this->list_of_keys = { "arm", "pivot", "teensy_write", "teensy_read","processed_vel", "reference_vel" }; // TODO: make it an argument
		this->experiment_time = 0.0;
		this->logger_obj->open_connection();
		if (!this->logger_obj->log_file.is_open()) {
			std::cout << "Couldn't open log file" << std::endl;
		}
		else {
			std::cout << "Opened file called: " << this->logger_obj->file_name << std::endl;
		}
	}
}

ParallelRun::ParallelRun(TeensyCollector* teensy, NatNetCollector* natnet) {
	this->teensy = teensy;
	this->natnet = natnet;

	std::vector<float> initial_data = { -1 };
	this->shared_data_map.insert(std::pair<std::string, std::vector<float>>("arm", { -1, -1, -1 }));
	this->shared_data_map.insert(std::pair<std::string, std::vector<float>>("pivot", { -1, -1, -1 }));
	this->shared_data_map.insert(std::pair<std::string, std::vector<float>>("teensy_write", { -1, -1, -1 }));
	this->shared_data_map.insert(std::pair<std::string, std::vector<float>>("teensy_read", { -1, -1, -1, -1 }));
	this->shared_data_map.insert(std::pair<std::string, std::vector<float>>("write_flag", initial_data));
	this->shared_data_map.insert(std::pair<std::string, std::vector<float>>("read_flag", initial_data));
	this->shared_data_map.insert(std::pair<std::string, std::vector<float>>("processed_vel", initial_data));
	this->shared_data_map.insert(std::pair<std::string, std::vector<float>>("reference_vel", initial_data));
	this->shared_data_map.insert(std::pair<std::string, std::vector<float>>("current_Teensy_data", { -1, -1, -1, -1 }));
	this->shared_data_map.insert(std::pair<std::string, std::vector<float>>("previous_Teensy_data", { -1, -1, -1, -1 }));
	this->shared_data_map.insert(std::pair<std::string, std::vector<float>>("first_Teensy_data", { -1, -1, -1, -1 }));

	this->data_thread = new std::thread(&ParallelRun::data_collector_callback, this);
	this->control_thread = new std::thread(&ParallelRun::mpc_controller_callback, this);
	this->verbose = false;
	this->log = false;
	this->logger = NULL;
}

ParallelRun::~ParallelRun() {
	delete[] this->data_thread;
	delete[] this->control_thread;
	this->control_thread = NULL;
	this->data_thread = NULL;
}

void ParallelRun::read_from_common_dictionary(std::string key, std::vector<float>& value) {
	std::lock_guard<std::mutex> guard(this->our_mutex);
	value = this->shared_data_map[key];
}

void ParallelRun::read_common_dictionary_all(std::map<std::string, std::vector<float>>& shared_data) {
	// to read from common dictionary just send a &shared_data (pass by reference)
	// this will rewrite the shared data structure that the function receives
	std::lock_guard<std::mutex> guard(this->our_mutex);
	shared_data.clear();  // making sure there's no leftover values (or keys)
	shared_data = this->shared_data_map;
}

void ParallelRun::write_common_dictionary(std::string key, std::vector<float> new_data) {
	// this function rewrites the values in a specific key 
	std::lock_guard<std::mutex> guard(this->our_mutex);
	if (this->shared_data_map.find(key) != this->shared_data_map.end()) {
		//rewrite
		this->shared_data_map[key] = new_data;
	}
	else {
		//write new
		this->shared_data_map.insert(std::pair <std::string, std::vector<float>>(key, new_data));
	}
}

void ParallelRun::logger_callback() {

	auto start = std::chrono::system_clock::now();
	float interval = 1000000. / this->logger_obj->frequency_in_hz;
	std::map<std::string, std::vector<float>> data_map;
	
	while (1) {
		try {
			auto now = std::chrono::system_clock::now();
			float elapsed = float(std::chrono::duration_cast<std::chrono::microseconds>(now - start).count());
			if (elapsed >= interval) {
				this->experiment_time += elapsed/1000000.; // converting to seconds
				start = now;
			
				this->read_common_dictionary_all(data_map);
				this->logger_obj->log_data(data_map, this->list_of_keys, this->experiment_time);
			}
		}
		catch (std::exception& e) {
			this->logger_obj->close_connection();
		}
	}
}

void ParallelRun::data_collector_callback() {


	using namespace std::chrono;
	auto begin = std::chrono::system_clock::now();
	float sampling_interval = 1000000. / 600.00; //  rate is 600 Hz
	int sampling_counter = 0;

	std::map<std::string, std::vector<float>> read_entire_map;
	std::vector < float > write_flag;
	std::vector < float > read_flag;
	std::vector <float> to_write;
	std::vector<float> teensy_sample;
	this->write_common_dictionary("write_flag", { -1 });
	this->write_common_dictionary("read_flag", { 1 });
	bool start_of_processing_FLAG = false;

	std::vector < float > arm_data{ 0,0,0 };
	std::vector < float > pivot_data{ 0,0,0 };
	std::vector < float > Teensy_data{ 0,0,0,0 };
	std::vector < float > input_cmd{ 0,0,0 };

	/////////// How do we initialize this part? They should contain the memory for the next iteration////////////////////
	float Ts = 0.003333; // The global sampling rate used for both real-time filtering and control
	std::vector < float> first_TeensyData{ 0,0,0,0 };
	std::vector < float> TeensyData_previous{ 0,0,0,0 };
	std::vector < float> arm_data_previous{ 0,0,0 };
	std::vector < float> pivot_data_previous{ 0,0,0 };
	std::vector <float> vec_psi;      // Angular position obtained from the rigid body coordinates
	std::vector <float> angular_vel_wheel_raw; // Derivative of vec_psi: raw angular velocity of the wheel
	float angular_vel_wheel;     // Filtered angular velocity of the wheel
	std::vector <float> vect_temp;    // a temporary vector for processing

	float xra_lc, yra_lc, psi;

	int ref_counter = 0;
	float reference_vel = 0;
	//std::vector < float > steps{ 2, 2.4, 2.8, 2, 2.6, 2.2}; // in rad/s
	//std::vector < float > steps{ 2, 2.5, 1.8, 2.3, 2, 1.5}; // in rad/s
	//std::vector < float > steps{ 2.6, 2.6, 2.6, 2.6, 2.2, 2.2, 2.2, 2.2}; // in rad/s
	std::vector < float > steps{ 2.6, 2.6, 2.6, 2.6, 1.8, 1.8, 1.8, 1.8 }; // in rad/s
	//std::vector < float > steps{ 2.6, 2.6, 2.6, 2.6, 1.4, 1.4, 1.4, 1.4 }; // in rad/s
	//std::vector < float > steps{ 2.6, 2.6, 2.6, 2.6, 1, 1, 1, 1}; // in rad/s
	//std::vector < float > steps{ 2.6, 2.6, 2.6, 2.6, 0, 0, 0, 0}; // in rad/s
	float off_time = 3;
	float on_time = 8;

	auto begin_of_time = std::chrono::system_clock::now();
	while (1) {
		//auto start = std::chrono::high_resolution_clock::now();
		auto time_stamp = std::chrono::system_clock::now();
		float elapsed = float(std::chrono::duration_cast<std::chrono::microseconds>(time_stamp - begin).count());

		if (elapsed >= sampling_interval) {
			begin = time_stamp;

			//Reference signal generation here
			reference_vel = reference_generator(ref_counter, steps, off_time, on_time, 600);
			ref_counter++;
 
			// To get the data from mpc thread and  write to Serial port
			this->read_from_common_dictionary("write_flag", write_flag);
			if (write_flag[0] > 0.0) {
				this->teensy->send_data(this->shared_data_map["teensy_write"]);
				this->write_common_dictionary("write_flag", { -1 });
			}

			// To receive data from the mocap and the Serial port and write the data to keys
			teensy_sample = this->teensy->get_sample();
			this->natnet->collect_data();
			this->write_common_dictionary("arm", this->natnet->rb_data_map["Arm"]);
			this->write_common_dictionary("pivot", this->natnet->rb_data_map["Pivot"]);
			this->write_common_dictionary("teensy_read", teensy_sample);
			this->read_from_common_dictionary("teensy_write", to_write);

			sampling_counter = (sampling_counter + 1) % 2; // Change 2 to 3 if we want the processing code below running from 1/2 to 1/3 the rate of sampling loop

			float current_time = float(std::chrono::duration_cast<std::chrono::microseconds>(time_stamp - begin_of_time).count());
			// If the current_time is more than 1 second from the begin_of_time, then start the 300Hz sampling rate
			if (sampling_counter == 0) {
				float this_time = float(std::chrono::duration_cast<std::chrono::microseconds>(time_stamp - begin_of_time).count());
				arm_data_previous = arm_data;
				pivot_data_previous = pivot_data;
				TeensyData_previous = Teensy_data;

				// Read the entire data map here!
				this->read_common_dictionary_all(read_entire_map);
				arm_data = read_entire_map["arm"];
				pivot_data = read_entire_map["pivot"];
				Teensy_data = read_entire_map["teensy_read"];

				if (start_of_processing_FLAG == false && current_time >= 2000000) { // This only runs one time when current_time exceeds 1 second
					first_TeensyData = Teensy_data;

					TeensyData_previous = Teensy_data;

					arm_data_previous = arm_data;
					pivot_data_previous = pivot_data;
					start_of_processing_FLAG = true;
				}
				// Handling bad data here
				// Note that the data logger will not update what goes into the MPC code.			
				if (arm_data.size() < 3 || pivot_data.size() < 3 || Teensy_data.size() < 4 || Teensy_data[0] < 1000 || Teensy_data[1] < 1 || Teensy_data[1] > 16) {
					arm_data = arm_data_previous; // We only update arm_data because only arm_data goes bad (losing markers)
					pivot_data = pivot_data_previous; // We only update arm_data because only arm_data goes bad (losing markers)
					Teensy_data = TeensyData_previous;
				}

				// Computing the new psi 		
				xra_lc = arm_data[0] - pivot_data[0];
				yra_lc = arm_data[2] - pivot_data[2];
				psi = atan2((yra_lc), (xra_lc)); // No need filtering, but this will loop 0 - 360 degree (doesn't really matter)
				// Adding new psi to the vector
				vec_psi.push_back(psi);

				// If there are more than 200 data in each vector
				// We perform a moving median of 2oo-point window the angular_vel_wheel
				if (vec_psi.size() >= 2) {
					angular_vel_wheel_raw.push_back(-0.64173 / 0.15677 * (vec_psi[1] - vec_psi[0]) / (Ts)); // computing the raw angular velocity
					if (angular_vel_wheel_raw.size() > 200) {
						angular_vel_wheel_raw.erase(angular_vel_wheel_raw.begin());
						// Assigning the vector angular_vel_wheel_raw to something before we sort them
						vect_temp = angular_vel_wheel_raw;
						std::nth_element(vect_temp.begin(), vect_temp.begin() + vect_temp.size() / 2, vect_temp.end()); // computing the median value of the angular_vel_wheel vector
						// The median is
						angular_vel_wheel = vect_temp[vect_temp.size() / 2]; // Adding the latest element to the angular_vel_wheel vector
					}
					vec_psi.erase(vec_psi.begin());
				}

				this->read_from_common_dictionary("read_flag", read_flag);
				if (read_flag[0] > 0.0 && current_time >= 2000000) {
					this->write_common_dictionary("first_Teensy_data", first_TeensyData);
					this->write_common_dictionary("previous_Teensy_data", TeensyData_previous);
					this->write_common_dictionary("processed_vel", { angular_vel_wheel });
					this->write_common_dictionary("reference_vel", { reference_vel });
					this->write_common_dictionary("read_flag", { -1 });
				}
			}
		}
	}
}

void ParallelRun::mpc_controller_callback() {

	/////////// Declare variables for data tranmissions between threads ////////////////////////////////////////////////////////
	std::map<std::string, std::vector<float>> read_entire_map;
	std::vector < float > write_flag{1};
	std::vector < float > read_flag{1};

	std::vector < float > current_Teensy_data{0,0,0,0};
	std::vector < float > previous_Teensy_data{ 0,0,0,0 };
	std::vector < float > first_Teensy_data{0,0,0,0};
	std::vector < float > angular_vel_wheel;
	std::vector < float > reference_vel{0};
	
	MPC_out MPC_Output;
	MPC_Output.idx_pre = 0;
	MPC_Output.err_ss = 0;
	MPC_Output.currentInput[0] = 0; MPC_Output.currentInput[1] = 0; MPC_Output.currentInput[2] = 0;
	MPC_Output.vel_sim = { 0 };
	MPC_Output.first_cycle = true;

	std::vector < float > input_cmd{ 0,0,0 };

	int counter = 0;
	bool mpc_condition_flag = false; //Helps initialize the mpc code
	Sleep(1000);
	while (1) {
		//std::this_thread::sleep_for(std::chrono::microseconds(33333));
		// MPC keeps coping data from the data_collector_callback at very fast rate, until MPC condition is satisfied
		
		this->read_from_common_dictionary("read_flag", read_flag);
		if (read_flag[0] < 0.0) {
			this->read_common_dictionary_all(read_entire_map);
			this->write_common_dictionary("read_flag", { 1 });
			first_Teensy_data = read_entire_map["first_Teensy_data"];
			previous_Teensy_data = read_entire_map["previous_Teensy_data"];
			current_Teensy_data = read_entire_map["teensy_read"];
			angular_vel_wheel = read_entire_map["processed_vel"];
			reference_vel = read_entire_map["reference_vel"];
			// Handle the index loopback: The condition flag allows the code to continue to run once the actuator index (TeensyData[TeensyData.size()-1][1] - first_TeensyData[1] >= 5) is met
			if ((current_Teensy_data[1] - first_Teensy_data[1] >= 5) && (current_Teensy_data[3] - previous_Teensy_data[3] == 1) || (mpc_condition_flag == true) && (current_Teensy_data[3] - previous_Teensy_data[3] == 1)) { //if actuator index is above 5 && the wheel just switches from 0 to 1	
				auto start = std::chrono::high_resolution_clock::now();
				mpc_condition_flag = true;
				std::cout << "Counter is: " << counter << std::endl;
				std::cout << "Current Wheel angle velocity is: " << angular_vel_wheel[0] << std::endl;
				std::cout << "Teensy_serial is " << "[" << current_Teensy_data[0] << ", " << current_Teensy_data[1] <<  ", " << current_Teensy_data[2] << ", " << current_Teensy_data[3]  << "]" << std::endl;

				MPC_Output = mpc(current_Teensy_data, previous_Teensy_data, angular_vel_wheel, first_Teensy_data[1], MPC_Output,reference_vel); // Will need to update the mpc.cpp, mpc, hpp, and the remaining files

				input_cmd[0] = MPC_Output.currentInput[0];
				input_cmd[1] = MPC_Output.currentInput[1] * 180 / 3.14156;
				input_cmd[2] = MPC_Output.currentInput[2] * 180 / 3.14156;

				std::cout << "Input command is: " << "[" << input_cmd[0] << ", " << input_cmd[1] << ", " << input_cmd[2] << "]" << std::endl;

				this->read_from_common_dictionary("write_flag", write_flag);
				if (write_flag[0] < 0) { // If writing to the shared map is possible,
					this->write_common_dictionary("teensy_write", input_cmd); // writing the input command
					this->write_common_dictionary("write_flag", { 1 }); // toggling write flag to 1
				}
				auto end = std::chrono::high_resolution_clock::now();
				double time_taken = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count(); time_taken *= 1e-9;
				std::cout << "Time taken by MPC is : " << std::fixed << time_taken << std::setprecision(9); std::cout << " sec" << std::endl;
				std::cout << std::endl;
				//std::cout << "MPC Loop entered..." << std::endl << std::endl;
				counter = counter + 1;
			}
		}
	}
}

void ParallelRun::run_parallel_controller() {
	this->data_thread->join();
	this->control_thread->join();
	if (this->log) {
		this->logger->join();
	}
}


float reference_generator(int ref_counter, std::vector < float > steps, float off_time, float on_time, float LOOP_FREQUENCY) {
	float reference_velocity = 0;
	if (ref_counter < (off_time * LOOP_FREQUENCY)) {
		reference_velocity = 0;
	}
	else if (ref_counter < (off_time + on_time) * LOOP_FREQUENCY) {
		reference_velocity = steps[0];
	}
	else if (ref_counter < (off_time + 2 * on_time) * LOOP_FREQUENCY) {
		reference_velocity = steps[1];
	}
	else if (ref_counter < (off_time + 3 * on_time) * LOOP_FREQUENCY) {
		reference_velocity = steps[2];
	}
	else if (ref_counter < (off_time + 4 * on_time) * LOOP_FREQUENCY) {
		reference_velocity = steps[3];
	}
	else if (ref_counter < (off_time + 5 * on_time) * LOOP_FREQUENCY) {
		reference_velocity = steps[4];
	}
	else if (ref_counter < (off_time + 6 * on_time) * LOOP_FREQUENCY) {
		reference_velocity = steps[5];
	}
	else if (ref_counter < (off_time + 7 * on_time) * LOOP_FREQUENCY) {
		reference_velocity = steps[6];
	}
	else if (ref_counter < (off_time + 8 * on_time) * LOOP_FREQUENCY) {
		reference_velocity = steps[7];
	}
	else {
		return 0;
	}
	return reference_velocity;
}