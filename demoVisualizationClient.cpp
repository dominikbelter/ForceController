#include "include/defs/defs.h"
#include "include/board/boardDynamixel.h"
#include "include/kinematic/kinematicLie.h"
#include "include/legModel/insectLeg.h"
#include "include/robotModel/robotMessor2.h"
#include "include/visualization/visualizerGL.h"
#include "include/visualization/visualizerIrrlicht.h"
#include <iostream>
#include <stdio.h>
#include <thread>
#include <time.h>
#include <string>
#include <boost/array.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::udp;

/*
 Paulina Jankowska
 Tomasz Chrosniak
 */

using namespace std;

Visualizer* visualizer;
vector<float_type> configuration = {0+PI/180,24*PI/180,-114*PI/180};
std::string serverAddress;
Mat34 robotPose;

void setPosition()
{
    boost::asio::io_service io_service;
    for(;;configuration.clear())
    {
        udp::resolver resolver(io_service);
        udp::resolver::query query(udp::v4(), serverAddress.c_str(), "daytime");

        udp::endpoint receiver_endpoint = *resolver.resolve(query);
        udp::socket socket(io_service);
        socket.open(udp::v4());

        boost::array<char, 1> send_buf  = {{ 0 }};
        socket.send_to(boost::asio::buffer(send_buf), receiver_endpoint);
        boost::array<float_type, 18> recv_buf;
        udp::endpoint sender_endpoint;
        socket.receive_from(boost::asio::buffer(recv_buf), sender_endpoint);
        if(recv_buf.size()!=18) continue;
        cout<<"[";
        for(int i=0;i<18;++i)
        {
            configuration.push_back(recv_buf[i]);
            cout<<(float_type)recv_buf[i]<<" ";
        }
        cout<<"]"<<endl;
        visualizer->setPosition(configuration);
        cout<<"Done."<<endl;
        timespec requestedTime;
        requestedTime.tv_nsec = 200000;
        requestedTime.tv_sec = 0;
        nanosleep(&requestedTime,NULL);
        cout<<"Sleeping..."<<endl;
    }
}

int main( int argc, const char** argv )
{
    try
    {
        std::cout<<"Input server address:"<<endl;
        std::cin>>serverAddress;


       for(int i=0 ; i<4 ; i++) {
            for(int j=0 ; j<4 ; j++) {
                robotPose(i,j) = 0;
            }
        }
        robotPose(0,0) = 1;
        robotPose(1,1) = 1;
        robotPose(2,2) = 1;
        robotPose(3,3) = 1;

        try {
            visualizer = createVisualizerIrrlicht("VisualizerWindow", 1920, 1024, 0.01, false);
        }
        catch (const std::exception& ex) {
            try{
                visualizer = createVisualizerIrrlicht("configVisualization.xml", "TEST");
            }
            catch(const std::exception &ex2)
            {
                std::cerr << ex.what() << std::endl;
                std::cerr << ex2.what() << std::endl;
                std::cout<<"Cannot start the visualizer"<<endl;
                return 1;
            }
        }

        std::thread visualizerThread(setPosition);
        visualizer->drawRobot(robotPose,configuration);


    }catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
    return 0;
}
