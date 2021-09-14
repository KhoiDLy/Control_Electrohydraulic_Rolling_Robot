#ifndef __NATNETCLILENT_H__
#define __NATNETCLILENT_H__

#include <iostream>
#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "data_collector/collector.hpp"
#include <conio.h>
#include <vector>
#include <map>
#include <NatNetTypes.h>
#include <NatNetCAPI.h>
#include <NatNetClient.h>


class NatNetCollector : public Collector {
    public:
        std::vector<float> data; // data that will get ultimately queried
        std::map<std::string, std::vector<float>> rb_data_map; // data for each individual rigid body

        std::string host_ip; // ip of the server (i.e. mocap system)
        std::string client_ip;
        int buffer_size; // expected buffer size (3 for only xyz positions)
        std::vector<std::string> rigid_body_list; // list of rigid bodies we're trying to track

        std::map<std::string, std::vector<float>> get_sample(std::vector<std::string>); // overloaded virtual method
        std::vector<float> get_sample();

        NatNetCollector(std::string, std::string, int); // constructor
        ~NatNetCollector(); // destructor
    
        NatNetClient* g_pClient = NULL; // client object
        void add_rigid_body(std::string);
        sNatNetClientConnectParams g_connectParams;
    
        std::map<int, std::string> rb_ids;
        sDataDescriptions* pDataDefs;
        std::vector<int> rb_indx; // where rigid bodies are in the data structure of the messenger
       
        int initialize_client_thread(); // main of NatNetClient
        void reset_client(); 
        int connect_client();
        void collect_data();
 };

#endif