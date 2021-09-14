#include "data_collector/natnet_client.hpp"

void NATNET_CALLCONV DataHandler(sFrameOfMocapData*, void*);

static const ConnectionType kDefaultConnectionType = ConnectionType_Multicast;
sServerDescription g_serverDescription;
int g_analogSamplesPerMocapFrame = 0;
std::map<int, std::vector<float>> map_on_callback;


std::vector<float> NatNetCollector::get_sample() {
    // This function doesn't do more than allow this abstract class to be instantiated
    //this->collect_data(); // this will extract data from the client thread
    this->collect_data();
    return this->rb_data_map.begin()->second; // data from the first rigid body within the map
}

std::map<std::string, std::vector<float>> NatNetCollector::get_sample(std::vector<std::string> list_of_rigid_bodies) 
{
    /*
    *  May get queried at any rate and should always return buffer data
    *  So far it returns the whole map, but later I'll set it up so it 
    *  can return the maps of the selected rigid bodies
    */
    this->collect_data(); // this will extract data from the client thread
    return this->rb_data_map;
}

void NatNetCollector::add_rigid_body(std::string new_rigid_body) 
{
    /*
    *  Appends a new rigid body id to the known rigid bodies to look for
    */
    this->rigid_body_list.push_back(new_rigid_body);
    std::vector<float> data0;
    for (int i = buffer_size - 1; i >= 0; i--) {
        this->data.push_back(0.0);
        data0.push_back(0.0);
    }
    // new value gets added to our std::map:
    this->rb_data_map.insert(std::pair<std::string, std::vector<float>>(new_rigid_body, data0));
}

NatNetCollector::NatNetCollector(std::string host_ip, std::string client_ip, int buffer_size)
{
    /*
    *   Constructor: initialize this object using host ip and client ip
    */
    this->host_ip = host_ip;
    this->client_ip = client_ip;
    this->buffer_size = buffer_size;

    //initialize data_buffer just in case it gets queried at t = 0
    for (int i = buffer_size - 1; i >= 0; i--) data.push_back(0.0);
}

NatNetCollector::~NatNetCollector() 
{
    if (this->g_pClient)
    {
        this->g_pClient->Disconnect();
        delete g_pClient;
        this->g_pClient = NULL;
    }
}

void NatNetCollector::collect_data() {
    try {
        if (map_on_callback.begin()->second.size() > 0) {
            for (std::map<int, std::vector<float>>::iterator it = map_on_callback.begin(); it != map_on_callback.end(); it++) {
                if (this->rb_data_map.find(this->rb_ids[it->first]) != this->rb_data_map.end())
                    this->rb_data_map[this->rb_ids[it->first]] = it->second;

                else
                    this->rb_data_map.insert(std::pair<std::string, std::vector<float>>(this->rb_ids[it->first], it->second));
            }
        }
    }
    catch (std::exception& e) {
        std::cout << "missed that one" << std::endl;
    }
}

int NatNetCollector::initialize_client_thread()
{
    this->g_pClient = new NatNetClient();

    // The following line creates a new thread and assigns the fram handler function
    this->g_pClient->SetFrameReceivedCallback(DataHandler, this->g_pClient); 

    this->g_connectParams.connectionType = kDefaultConnectionType;
    this->g_connectParams.serverAddress = this->host_ip.c_str();
    this->g_connectParams.localAddress = this->client_ip.c_str();
    
    int iResult = this->connect_client(); //initialize natnet client
    if (iResult != ErrorCode_OK)
    {
        std::cout << "Error initializing client.  See log for details.  Exiting" << std::endl;
        return 1;
    }

    // This finds the index of the description list that corresponds to rigid bodies
    // Only has to be done once in the code and once it's found, it doesn't change
    this->pDataDefs = NULL;

    iResult = this->g_pClient->GetDataDescriptionList(&this->pDataDefs);

    for (int i = 0; i < this->pDataDefs->nDataDescriptions; i++)
    {
        if (this->pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody) {

            this->rb_ids.insert(std::pair<int, std::string>(i, this->pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription->szName));
        }
    }
    if (this->pDataDefs) {
        NatNet_FreeDescriptions(this->pDataDefs);
        this->pDataDefs = NULL;
    }

    bool bExit = false;
    g_connectParams.connectionType = ConnectionType_Multicast;
   // iResult = this->connect_client();

    if (iResult != ErrorCode_OK)
        std::cout << "Error changing client connection type to Multicast." << std::endl;

    return ErrorCode_OK;
}

int NatNetCollector::connect_client()
{
    /*
    * This function initializes the NatNetClient connection and provides some 
    * initial data about it
    */

    g_pClient->Disconnect(); // release previous server
    int retCode = g_pClient->Connect(this->g_connectParams); // init client
    if (retCode != ErrorCode_OK) // connection failed
    {
        std::cout << "Unable to connect to server.  Error code: " << retCode << " Exiting";
        return ErrorCode_Internal;
    }
    else // connection established
    {
        void* pResult;
        int nBytes = 0;
        ErrorCode ret = ErrorCode_OK;

        // print server info
        memset(&g_serverDescription, 0, sizeof(g_serverDescription));
        ret = g_pClient->GetServerDescription(&g_serverDescription);
        if (ret != ErrorCode_OK || !g_serverDescription.HostPresent)
        {
            std::cout << "Unable to connect to server. Host not present. Exiting." << std::endl;
            return 1;
        }
        std::cout << "[SampleClient] Server application info:" << std::endl;
        std::cout << "Client IP: " << this->g_connectParams.localAddress << std::endl;
        std::cout << "Server IP: " <<  this->g_connectParams.serverAddress << std::endl;
        std::cout << "Server Name: " << g_serverDescription.szHostComputerName << std::endl;

        
    }



    return ErrorCode_OK;
}

void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData)
{
    NatNetClient* pClient = (NatNetClient*)pUserData;

    // Software latency here is defined as the span of time between:
    //   a) The reception of a complete group of 2D frames from the camera system (CameraDataReceivedTimestamp)
    // and
    //   b) The time immediately prior to the NatNet frame being transmitted over the network (TransmitTimestamp)
    //
    // This figure may appear slightly higher than the "software latency" reported in the Motive user interface,
    // because it additionally includes the time spent preparing to stream the data via NatNet.
    const uint64_t softwareLatencyHostTicks = data->TransmitTimestamp - data->CameraDataReceivedTimestamp;
    const double softwareLatencyMillisec = (softwareLatencyHostTicks * 1000) / static_cast<double>(g_serverDescription.HighResClockFrequency);

    // Transit latency is defined as the span of time between Motive transmitting the frame of data, and its reception by the client (now).
    // The SecondsSinceHostTimestamp method relies on NatNetClient's internal clock synchronization with the server using Cristian's algorithm.
    const double transitLatencyMillisec = pClient->SecondsSinceHostTimestamp(data->TransmitTimestamp) * 1000.0;

   /* if (g_outputFile)
    {
        _WriteFrame(g_outputFile, data);
    }*/

    int i = 0;
  
    /*printf("FrameID : %d\n", data->iFrame);
    printf("Timestamp : %3.2lf\n", data->fTimestamp);
    printf("Software latency : %.2lf milliseconds\n", softwareLatencyMillisec);
    */
    // Only recent versions of the Motive software in combination with ethernet camera systems support system latency measurement.
    // If it's unavailable (for example, with USB camera systems, or during playback), this field will be zero.
    const bool bSystemLatencyAvailable = data->CameraMidExposureTimestamp != 0;

    if (bSystemLatencyAvailable)
    {
        // System latency here is defined as the span of time between:
        //   a) The midpoint of the camera exposure window, and therefore the average age of the photons (CameraMidExposureTimestamp)
        // and
        //   b) The time immediately prior to the NatNet frame being transmitted over the network (TransmitTimestamp)
        const uint64_t systemLatencyHostTicks = data->TransmitTimestamp - data->CameraMidExposureTimestamp;
        const double systemLatencyMillisec = (systemLatencyHostTicks * 1000) / static_cast<double>(g_serverDescription.HighResClockFrequency);

        // Client latency is defined as the sum of system latency and the transit time taken to relay the data to the NatNet client.
        // This is the all-inclusive measurement (photons to client processing).
        const double clientLatencyMillisec = pClient->SecondsSinceHostTimestamp(data->CameraMidExposureTimestamp) * 1000.0;

        // You could equivalently do the following (not accounting for time elapsed since we calculated transit latency above):
        //const double clientLatencyMillisec = systemLatencyMillisec + transitLatencyMillisec;

      /*  printf("System latency : %.2lf milliseconds\n", systemLatencyMillisec);
        printf("Total client latency : %.2lf milliseconds (transit time +%.2lf ms)\n", clientLatencyMillisec, transitLatencyMillisec);*/
    }
   /* else
    {
        printf("Transit latency : %.2lf milliseconds\n", transitLatencyMillisec);
    }*/

    // FrameOfMocapData params
    bool bIsRecording = ((data->params & 0x01) != 0);
    bool bTrackedModelsChanged = ((data->params & 0x02) != 0);
   /* if (bIsRecording)
        printf("RECORDING\n");
    if (bTrackedModelsChanged)
        printf("Models Changed.\n");
*/

    // timecode - for systems with an eSync and SMPTE timecode generator - decode to values
    int hour, minute, second, frame, subframe;
    NatNet_DecodeTimecode(data->Timecode, data->TimecodeSubframe, &hour, &minute, &second, &frame, &subframe);
    // decode to friendly string
    char szTimecode[128] = "";
    NatNet_TimecodeStringify(data->Timecode, data->TimecodeSubframe, szTimecode, 128);
   // printf("Timecode : %s\n", szTimecode);

    // Rigid Bodies
   // printf("Rigid Bodies [Count=%d]\n", data->nRigidBodies);
    for (i = 0; i < data->nRigidBodies; i++)
    {
        // params
        // 0x01 : bool, rigid body was successfully tracked in this frame
        bool bTrackingValid = data->RigidBodies[i].params & 0x01;

        /*printf("Rigid Body [ID=%d  Error=%3.2f  Valid=%d]\n", data->RigidBodies[i].ID, data->RigidBodies[i].MeanError, bTrackingValid);
        printf("\tx\ty\tz\tqx\tqy\tqz\tqw\n");
        printf("\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
            data->RigidBodies[i].x,
            data->RigidBodies[i].y,
            data->RigidBodies[i].z,
            data->RigidBodies[i].qx,
            data->RigidBodies[i].qy,
            data->RigidBodies[i].qz,
            data->RigidBodies[i].qw);*/
        if (bTrackingValid) {
            if (map_on_callback.find(i) != map_on_callback.end()) {
                map_on_callback[i].clear();
                map_on_callback[i].push_back(data->RigidBodies[i].x);
                map_on_callback[i].push_back(data->RigidBodies[i].y);
                map_on_callback[i].push_back(data->RigidBodies[i].z);
            }
            else {
                std::vector<float> data_to_insert;
                data_to_insert.push_back(data->RigidBodies[i].x);
                data_to_insert.push_back(data->RigidBodies[i].y);
                data_to_insert.push_back(data->RigidBodies[i].z);
                map_on_callback.insert(std::pair<int, std::vector<float>>(i, data_to_insert));
            }
        }
    }

   /* // Skeletons
    printf("Skeletons [Count=%d]\n", data->nSkeletons);
    for (i = 0; i < data->nSkeletons; i++)
    {
        sSkeletonData skData = data->Skeletons[i];
        printf("Skeleton [ID=%d  Bone count=%d]\n", skData.skeletonID, skData.nRigidBodies);
        for (int j = 0; j < skData.nRigidBodies; j++)
        {
            sRigidBodyData rbData = skData.RigidBodyData[j];
            printf("Bone %d\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
                rbData.ID, rbData.x, rbData.y, rbData.z, rbData.qx, rbData.qy, rbData.qz, rbData.qw);
        }
    }

    // labeled markers - this includes all markers (Active, Passive, and 'unlabeled' (markers with no asset but a PointCloud ID)
    bool bOccluded;     // marker was not visible (occluded) in this frame
    bool bPCSolved;     // reported position provided by point cloud solve
    bool bModelSolved;  // reported position provided by model solve
    bool bHasModel;     // marker has an associated asset in the data stream
    bool bUnlabeled;    // marker is 'unlabeled', but has a point cloud ID that matches Motive PointCloud ID (In Motive 3D View)
    bool bActiveMarker; // marker is an actively labeled LED marker

    printf("Markers [Count=%d]\n", data->nLabeledMarkers);
    for (i = 0; i < data->nLabeledMarkers; i++)
    {
        bOccluded = ((data->LabeledMarkers[i].params & 0x01) != 0);
        bPCSolved = ((data->LabeledMarkers[i].params & 0x02) != 0);
        bModelSolved = ((data->LabeledMarkers[i].params & 0x04) != 0);
        bHasModel = ((data->LabeledMarkers[i].params & 0x08) != 0);
        bUnlabeled = ((data->LabeledMarkers[i].params & 0x10) != 0);
        bActiveMarker = ((data->LabeledMarkers[i].params & 0x20) != 0);

        sMarker marker = data->LabeledMarkers[i];

        // Marker ID Scheme:
        // Active Markers:
        //   ID = ActiveID, correlates to RB ActiveLabels list
        // Passive Markers: 
        //   If Asset with Legacy Labels
        //      AssetID 	(Hi Word)
        //      MemberID	(Lo Word)
        //   Else
        //      PointCloud ID
        int modelID, markerID;
        NatNet_DecodeID(marker.ID, &modelID, &markerID);

        char szMarkerType[512];
       /* if (bActiveMarker)
            strcpy(szMarkerType, "Active");
        else if (bUnlabeled)
            strcpy(szMarkerType, "Unlabeled");
        else
            strcpy(szMarkerType, "Labeled");

        printf("%s Marker [ModelID=%d, MarkerID=%d, Occluded=%d, PCSolved=%d, ModelSolved=%d] [size=%3.2f] [pos=%3.2f,%3.2f,%3.2f]\n",
            szMarkerType, modelID, markerID, bOccluded, bPCSolved, bModelSolved, marker.size, marker.x, marker.y, marker.z);
    }

    // force plates
    printf("Force Plate [Count=%d]\n", data->nForcePlates);
    for (int iPlate = 0; iPlate < data->nForcePlates; iPlate++)
    {
        printf("Force Plate %d\n", data->ForcePlates[iPlate].ID);
        for (int iChannel = 0; iChannel < data->ForcePlates[iPlate].nChannels; iChannel++)
        {
            printf("\tChannel %d:\t", iChannel);
            if (data->ForcePlates[iPlate].ChannelData[iChannel].nFrames == 0)
            {
                printf("\tEmpty Frame\n");
            }
            else if (data->ForcePlates[iPlate].ChannelData[iChannel].nFrames != g_analogSamplesPerMocapFrame)
            {
                printf("\tPartial Frame [Expected:%d   Actual:%d]\n", g_analogSamplesPerMocapFrame, data->ForcePlates[iPlate].ChannelData[iChannel].nFrames);
            }
            for (int iSample = 0; iSample < data->ForcePlates[iPlate].ChannelData[iChannel].nFrames; iSample++)
                printf("%3.2f\t", data->ForcePlates[iPlate].ChannelData[iChannel].Values[iSample]);
            printf("\n");
        }
    }

    printf("Device [Count=%d]\n", data->nDevices);
    for (int iDevice = 0; iDevice < data->nDevices; iDevice++)
    {
        printf("Device %d\n", data->Devices[iDevice].ID);
        for (int iChannel = 0; iChannel < data->Devices[iDevice].nChannels; iChannel++)
        {
            printf("\tChannel %d:\t", iChannel);
            if (data->Devices[iDevice].ChannelData[iChannel].nFrames == 0)
            {
                printf("\tEmpty Frame\n");
            }
            else if (data->Devices[iDevice].ChannelData[iChannel].nFrames != g_analogSamplesPerMocapFrame)
            {
                printf("\tPartial Frame [Expected:%d   Actual:%d]\n", g_analogSamplesPerMocapFrame, data->Devices[iDevice].ChannelData[iChannel].nFrames);
            }
            for (int iSample = 0; iSample < data->Devices[iDevice].ChannelData[iChannel].nFrames; iSample++)
                printf("%3.2f\t", data->Devices[iDevice].ChannelData[iChannel].Values[iSample]);
            printf("\n");
        }
    }*/
}



void NatNetCollector::reset_client()
{
    int iSuccess;
    std::cout << "Resetting Client" << std::endl;
    iSuccess = this->g_pClient->Disconnect();
    iSuccess = this->g_pClient->Connect(g_connectParams);
    if (iSuccess != 0)
        std::cout << "Reset Failed" << std::endl;
    else std::cout << "Successful Reset!" << std::endl;
}
