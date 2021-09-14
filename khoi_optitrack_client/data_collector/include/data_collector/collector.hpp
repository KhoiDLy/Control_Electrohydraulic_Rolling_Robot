#ifndef __COLLECTOR_H__
#define __COLLECTOR_H__

#include <iostream>
#include <string>
#include <vector>
#include <map>

class Collector{
    public:
        std::vector<float> data;
        virtual std::vector<float> get_sample() = 0; // if it gets called without arguments, then just return a vector of floats
        virtual std::map<std::string, std::vector<float>> get_sample(std::vector<std::string>) = 0; // if it gets called with a list of strings, return a map to each field's value
        virtual ~Collector() = default; 
};

#endif