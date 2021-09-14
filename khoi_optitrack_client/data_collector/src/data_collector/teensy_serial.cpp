#include "data_collector/teensy_serial.hpp"

std::map<std::string, std::vector<float>> TeensyCollector::get_sample(std::vector<std::string> list_of_keys)
{
	/*
	*  May get queried at any rate and should always return buffer data
	*  So far it returns the whole map, but later I'll set it up so it
	*  can return the maps of the selected rigid bodies
	*/
	this->collect();
	std::map<std::string, std::vector<float>> map;
	map.insert(std::pair < std::string, std::vector<float>>(list_of_keys[0], this->data));
	return map;
}

TeensyCollector::TeensyCollector(std::string serial_port, int data_length, int baudrate)
{
	this->port_id.assign(serial_port);
	this->data_length = data_length;
	for (int i = data_length - 1; i >= 0; i--) {
		this->data.push_back(0.0);
	}
	
	this->serial_port = new Serial(this->port_id.c_str(), baudrate);
	this->first_line_toggle = true;
}

TeensyCollector::~TeensyCollector()
{
	//this->f_stream.close();
	this->serial_port->close_serial();
}

std::vector<float> TeensyCollector::get_sample()
{
	this->collect();
	return this->data;
}

void TeensyCollector::send_data(std::vector<float> to_send)
{
	// Rule of string conversion
	// 1. Empty string "" contains 0 bytes
	// 2. "\n" Contains 1 byte
	// 3. "," contains 1 byte
	// 4. An int contains exactly # of bytes as numbers (e.g. 321 -> 3 bytes)
	// 5. A float contains # of bytes equal to numbers with 6 decimal places (e.g. 32.1 -> 32.100000 -> 9 bytes)
	
	std::string stringify = "";
	if (to_send.size() > 1) {
		for (int i = 0; i < to_send.size() - 1; i++) { 
			stringify.append(std::to_string(to_send[i]));
			stringify.append(",");
		}
		stringify.append(std::to_string(to_send[to_send.size() - 1]));
		stringify.append("\n");
	}
	this->serial_port->WriteData(stringify.c_str(), stringify.size());
}

void TeensyCollector::collect()
{

	std::vector<float> new_data;
	float value;
	bool all_zeros = true;
	std::string line;
	

	try{
		int readResult = 0;
		char incomingData[23] = "";
		int dataLength = 22;
		readResult = serial_port->ReadData(incomingData, dataLength);
		incomingData[readResult] = 0;
		line = incomingData;
		
		std::stringstream s_stream(line);
		std::string substring;
		new_data.clear();
		while (s_stream.good()) {
			std::getline(s_stream, substring, ',');
			value = std::stof(substring);
			if (value > 1e-15) all_zeros = false;
			new_data.push_back(value);
		}
		if (!all_zeros) {
			this->data = new_data;
		}
		//for (int i = 0; i < this->data_length; i++) std::cout << this->data[i] << " ";
		//std::cout << std::endl;
	}
	catch (std::exception& e) {
	}
}
