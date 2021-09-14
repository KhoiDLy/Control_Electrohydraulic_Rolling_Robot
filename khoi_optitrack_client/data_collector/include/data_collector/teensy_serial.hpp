#ifndef __TEENSY_COLLECTOR_H__
#define __TEENSY_COLLECTOR_H__

#include "data_collector/collector.hpp"
#include "data_collector/SerialClass.h"
#include <stdio.h>
#include <sstream>
#include <string.h>
#include <string>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <codecvt>
#include <locale>

class TeensyCollector : public Collector{
    public:
        int data_length;
        std::string port_id;
        Serial * serial_port;
        bool first_line_toggle;
        TeensyCollector(std::string, int, int);
        TeensyCollector(const TeensyCollector&) = default;
        TeensyCollector& operator=(const TeensyCollector&) = default;
        ~TeensyCollector();
        std::vector<float> get_sample();
        std::map<std::string, std::vector<float>> get_sample(std::vector<std::string> name_of_the_key);
        void send_data(std::vector<float>);
        std::vector<float> GetValues(std::wstring s, wchar_t delim);
        void collect();
};

#endif