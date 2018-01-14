#include <optitrack/optitrack.hpp>
#include <optitrack/optitrack_channels.h>
#include <optitrack/common/getopt.h>
#include <optitrack/common/timestamp.h>
#include <unistd.h>
#include <lcmtypes/pose_xyt_t.hpp>
#include <lcm/lcm-cpp.hpp>
#include <netinet/in.h>
#include <sys/socket.h>
#include <iostream>
#include <cmath>



int main(int argc, char** argv)
{
    const char* kInterfaceArg = "interface";
    const char* kMBRigidBodyArg = "mobilebot";
    
    getopt_t* gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Display this help message.\n");
    getopt_add_string(gopt, 'i', kInterfaceArg, "192.168.3.0", "Local network interface used when connecting to the Optitrack network. (Ex. inet as shown by ifconfig on the current machine.)");
    getopt_add_int(gopt, 'r', kMBRigidBodyArg, "5", "Id of mobilebot rigid body to publish pose for.");

    
    
    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt, "help")) {
        printf("Usage: %s [options]", argv[0]);
        getopt_do_usage(gopt);
        return 1;
    }
    
    std::string interface = getopt_get_string(gopt, kInterfaceArg);
    int MBrigidBodyId = getopt_get_int(gopt, kMBRigidBodyArg);

    
    // If there's no interface specified, then we'll need to guess
    if (interface.length() == 0)
    {
        interface = guess_optitrack_network_interface();
    }
    // If there's still no interface, we have a problem
    if (interface.length() == 0)
    {
        printf("[optitrack_driver] error could not determine network interface for receiving multicast packets.\n");
        return -1;
    }
    
    SOCKET dataSocket = create_optitrack_data_socket(interface, PORT_DATA);
    
    if (dataSocket == -1) {
        printf("[optitrack_driver] error failed to create socket for interface %s:%d\n", interface.c_str(), PORT_DATA);
        return -1;
    } else {
        printf("[optitrack_driver] successfully created socket for interface %s:%d\n", interface.c_str(), PORT_DATA);
    }
    
    // Code from DataListenThread function in PacketClient.cpp
    char packet[20000];
    socklen_t addrLen = sizeof(sockaddr);
    sockaddr_in incomingAddress;
    std::vector<optitrack_message_t> incomingMessages;
    
    lcm::LCM lcmInstance;
    // MAIN THREAD LOOP

    printf("\nOptitrack Driver Running...\n");
    pose_xyt_t Pose;
    while (1) {
        // Block until we receive a datagram from the network
        recvfrom(dataSocket, packet, sizeof(packet), 0, (sockaddr*)&incomingAddress, &addrLen);
        incomingMessages = parse_optitrack_packet_into_messages(packet, sizeof(packet));
        for(auto& msg : incomingMessages) {
            
            if(msg.id == MBrigidBodyId) {
                Pose.utime = utime_now();
                Pose.x = msg.x;
                Pose.y = msg.z;
                double roll;
                double pitch;
                double yaw;
                toEulerAngle(msg, roll, pitch, yaw);
                Pose.theta = pitch;
            }            
        }
        lcmInstance.publish(OPTITRACK_CHANNEL, &Pose);
        usleep(1000);
    }
    
    // Cleanup options now that we've parsed everything we need
    getopt_destroy(gopt);

    return 0;
}
