#ifndef __LOGGER_H__
#define __LOGGER_H__

#pragma warning(disable : 4996) //_CRT_SECURE_NO_WARNINGS

#include <iostream>
#include <fstream>
#include <chrono>
#include <string>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <map>
#include <vector>

class Logger {
	public:
		Logger(std::string, std::string, float);
		~Logger();
		std::string format;
		std::string file_name;
		
		std::ofstream log_file;
		float frequency_in_hz;
		float experiment_time_in_seconds;
		
		std::map<std::string, std::vector<float>> data;
		void log_data(std::map<std::string, std::vector<float>>, std::vector<std::string>, float);
		std::string stringify_data_to_write(std::map<std::string, std::vector<float>>, std::vector<std::string>, float);
		void close_connection();
		void open_connection();
};

#endif