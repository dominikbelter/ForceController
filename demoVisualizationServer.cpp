#include "include/defs/defs.h"
#include "include/board/boardDynamixel.h"
#include "include/kinematic/kinematicLie.h"
#include "include/legModel/insectLeg.h"
#include "include/robotModel/robotMessor2.h"
#include "include/visualization/visualizerGL.h"
#include "include/visualization/visualizerIrrlicht.h"
#include <ctime>
#include <iostream>
#include <string>
#include <stdio.h>
#include <thread>
#include <time.h>
#ifdef __USE_VISUALIZER__
#include <boost/array.hpp>
#include <boost/asio.hpp>


/*
 Paulina Jankowska
 Tomasz Chrosniak
 */

using boost::asio::ip::udp;
Board* board;

vector<float_type> give_position()
{
    std::vector<float_type> configuration;
    std::vector<float_type> configSingleLeg;

    for(int i=0;i<6;++i)
    {
        configSingleLeg.clear();
        board->readPositions(i,configSingleLeg);
        for(int j=0;j<configSingleLeg.size();++j)
        {
            configuration.push_back(configSingleLeg[j]);
        }
    }
    return configuration;
}
#endif

int main()
{
    #ifdef __USE_VISUALIZER__
    board = createBoardDynamixel();

    try
    {
        boost::asio::io_service io_service;

        udp::socket socket(io_service, udp::endpoint(udp::v4(), 13));

        for(;;)
        {
            boost::array<char, 1> recv_buf;
            udp::endpoint remote_endpoint;
            boost::system::error_code error;
            socket.receive_from(boost::asio::buffer(recv_buf),
                                remote_endpoint, 0, error);

            if (error && error != boost::asio::error::message_size)
                throw boost::system::system_error(error);

            boost::system::error_code ignored_error;
            socket.send_to(boost::asio::buffer(give_position()),
                           remote_endpoint, 0, ignored_error);

        }

    }

    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        }
#endif
    return 0;
}
