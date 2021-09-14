#include "logger/logger.hpp"

Logger::Logger(std::string format, std::string file_name, float hz) {

	this->format = format;
	this->file_name.assign(file_name);
	this->frequency_in_hz = hz;

	std::time_t const now_c = std::time(NULL);
	std::stringstream transTime;
	std::string today_and_time;
	
	transTime << std::put_time(std::localtime(&now_c), "_%a_%d_%b_%Y_%I_%M_%S%p");
	today_and_time = transTime.str();
	this->file_name.append(today_and_time);
	this->file_name.append(".");
	this->file_name.append(format);
}

Logger::~Logger() {
	if (this->log_file.is_open()) {
		this->log_file.close();
	}
}

void Logger::close_connection() {
	this->log_file.close();
}

void Logger::open_connection() {
	this->log_file.open(this->file_name);
}

std::string Logger::stringify_data_to_write(std::map<std::string, std::vector<float>> shared_data, std::vector<std::string> list_of_keys, float exp_time) 
{
	std::string data_string = "";
	data_string.append(std::to_string(exp_time));
	data_string.append(",");
	for (int i = 0; i < list_of_keys.size(); i++) {
		if (shared_data.find(list_of_keys[i]) != shared_data.end()) {
			for (int j = 0; j < shared_data[list_of_keys[i]].size(); j++) {
				data_string.append(std::to_string(shared_data[list_of_keys[i]][j]));
				data_string.append(",");
			}
		}
	}
	data_string.append("\n");

	return data_string;
}

void Logger::log_data(std::map<std::string, std::vector<float>> shared_map, std::vector<std::string> list_of_keys, float experiment_time)
{
	this->data = shared_map;
	std::string data_string = this->stringify_data_to_write(this->data, list_of_keys, experiment_time);
	if (!this->log_file.is_open()) {
		this->log_file.open(this->file_name);
	} else {
		this->log_file << data_string;
	}
}

