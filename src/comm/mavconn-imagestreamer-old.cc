/*=====================================================================

MAVCONN Micro Air Vehicle Flying Robotics Toolkit
Please see our website at <http://MAVCONN.ethz.ch>

(c) 2009 MAVCONN PROJECT  <http://MAVCONN.ethz.ch>

This file is part of the MAVCONN project

    MAVCONN is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    MAVCONN is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MAVCONN. If not, see <http://www.gnu.org/licenses/>.

======================================================================*/

/**
 * @file
 *   @brief Process for capturing an image and send it to the groundstation.
 *
 *   @author Fabian Brun <mavteam@pixhawk.ethz.ch>
 */

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <inttypes.h>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <glib.h>
// BOOST includes
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// Latency Benchmarking
// #include <sys/time.h>
#include <time.h>

//Async stop, wish me luck! Alternatively, just wrap the whole thing in an if that checks whether a timer has expired, then reset the timer as you start
#include <thread>
#include <chrono>

//#include "PxSharedMemClient.h"
#include "interface/shared_mem/SHMImageClient.h"
#include "mavconn.h"


namespace config = boost::program_options;
namespace bfs = boost::filesystem;

int sysid;
int compid;
uint64_t camno;
bool silent, verbose, debug;

using namespace std;

#define PACKET_PAYLOAD		253
bool captureImage = false;

lcm_t* lcmImage;
lcm_t* lcmMavlink;
mavlink_message_t tmp;
mavlink_data_transmission_handshake_t req, ack;

bool quit = false;
/**
 * @brief Handle incoming MAVLink packets containing images
 */
static void image_handler (const lcm_recv_buf_t *rbuf, const char * channel, const mavconn_mavlink_msg_container_t* container, void * user)
{
	const mavlink_message_t* msg = getMAVLinkMsgPtr(container);

	// Pointer to shared memory data
	std::vector<px::SHMImageClient>* clientVec = reinterpret_cast< std::vector<px::SHMImageClient>* >(user);

	// Temporary memory for raw camera image
	cv::Mat img( 480, 640, CV_8UC1 );
	cv::Mat img2( 480, 640, CV_8UC1 );	// img2 not used.

	for(size_t i = 0; i < clientVec->size(); ++i){
		px::SHMImageClient& client = clientVec->at(i);
                if ((client.getCameraConfig() & px::SHMImageClient::getCameraNo(msg)) != px::SHMImageClient::getCameraNo(msg))
                         continue;

	// Check if there are images
	uint64_t camId = client.getCameraID(msg);

	if (camId >= 0)
	{
		// Copy one image from shared buffer
		if (!client.readStereoImage(msg, img, img2))
		{
			if(!client.readMonoImage(msg,img))
			{
				continue;
			}
		}

        //Only execute the stuff below every now and then: from here

        //if (captureImage){

		captureImage = false;

		// Check for valid jpg_quality in request and adjust if necessary
		if (req.jpg_quality < 1 || req.jpg_quality > 100)
		{
			req.jpg_quality = 60;
		}

		// Encode image as JPEG
		vector<uint8_t> jpg; ///< container for JPEG image data
		vector<int> p (2); ///< params for cv::imencode. Sets the JPEG quality.
		p[0] = CV_IMWRITE_JPEG_QUALITY;
		p[1] = req.jpg_quality;
		cv::imencode(".jpg", img, jpg, p);

		// Prepare and send acknowledgment packet
		ack.type = static_cast<uint8_t>( DATA_TYPE_JPEG_IMAGE );
		ack.size = static_cast<uint32_t>( jpg.size() );
		ack.packets = static_cast<uint8_t>( ack.size/PACKET_PAYLOAD );
		if (ack.size % PACKET_PAYLOAD) { ++ack.packets; } // one more packet with the rest of data
		ack.payload = static_cast<uint8_t>( PACKET_PAYLOAD );
		ack.jpg_quality = req.jpg_quality;
		ack.width = 640;
		ack.height = 480;

		mavlink_msg_data_transmission_handshake_encode(sysid, compid, &tmp, &ack);
		sendMAVLinkMessage(lcmMavlink, &tmp);

		// Send image data (split up into smaller chunks first, then sent over MAVLink)
		uint8_t data[PACKET_PAYLOAD];
		uint16_t byteIndex = 0;
		if (verbose) printf("there are %02d packets waiting to be sent (%05d bytes). start sending...\n", ack.packets, ack.size);

		for (uint8_t i = 0; i < ack.packets; ++i)
		{
			// Copy PACKET_PAYLOAD bytes of image data to send buffer
			for (uint8_t j = 0; j < PACKET_PAYLOAD; ++j)
			{
				if (byteIndex < ack.size)
				{
					data[j] = (uint8_t)jpg[byteIndex];
				}
				// fill packet data with padding bits
				else
				{
					data[j] = 0;
				}
				++byteIndex;
			}
			// Send ENCAPSULATED_IMAGE packet
			mavlink_msg_encapsulated_data_pack(sysid, compid, &tmp, i, data);
			sendMAVLinkMessage(lcmMavlink, &tmp);
			if (verbose) printf("sent packet %02d successfully\n", i+1);
		}

        //Only execute the stuff below every now and then: to here
        //}
	}
	}
    //std::this_thread::sleep_for(std::chrono::seconds(1));
}

/**
 * @brief Handle incoming MAVLink packets containing ACTION messages
 */
static void mavlink_handler (const lcm_recv_buf_t *rbuf, const char * channel, const mavconn_mavlink_msg_container_t* container, void * user)
{
	const mavlink_message_t* msg = getMAVLinkMsgPtr(container);

	if(msg->sysid == 42)
		//captureImage = true;

	if (msg->msgid == MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE)
	{
		mavlink_msg_data_transmission_handshake_decode(msg, &req);

		if (req.type == DATA_TYPE_JPEG_IMAGE && msg->sysid != 42) // TODO: use getSystemID()
		{
			// start recording image data
			//captureImage = true;
		}
	}


	if (msg->msgid == MAVLINK_MSG_ID_COMMAND_LONG)
	{
		mavlink_command_long_t cmd;
		mavlink_msg_command_long_decode(msg, &cmd);
        std::cout << "Received message COMMAND_LONG, what could it be? (Message command: "<<cmd.command<<", target "<<(uint8_t) cmd.target_system<<" (comparison with 1: "<<(cmd.target_system == 1)<<", and I am "<<getSystemID()<<")\n";
		if (cmd.target_system == 1 && cmd.command == MAV_CMD_IMAGE_START_CAPTURE){
            std::cout << "Received message CMD_IMAGE_START_CAPTURE!\n";
            captureImage= true;
        }   //TODO Note that getSystemID is not working unless we specify a mavconn.conf file (and I don't know its format). Above, we hardcode the system ID as 1. This is ugly, fix 
	}

}

void
signalHandler(int signal)
{
        if (signal == SIGINT)
        {
                fprintf(stderr, "# INFO: Quitting...\n");
                quit = true;
                exit(EXIT_SUCCESS);
        }
}

void* lcm_wait(void* lcm_ptr)
{
	lcm_t* lcm = (lcm_t*) lcm_ptr;
	// Blocking wait for new data
	while (true)
	{
		lcm_handle (lcm);
	}
	return NULL;
}

void* lcm_image_wait(void* lcm_ptr)
{
	lcm_t* lcm = (lcm_t*)lcm_ptr;
	while(1)
	{	
		lcm_handle(lcm);
	}
	return NULL;
}

/*
 * @brief Main: registers at mavlink channel for images; starts image handler
 */
int main(int argc, char* argv[])
{
	// ----- Handling Program options
	config::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("sysid,a", config::value<int>(&sysid)->default_value(42), "ID of this system, 1-256")
		("compid,c", config::value<int>(&compid)->default_value(30), "ID of this component")
		("camno,c", config::value<uint64_t>(&camno)->default_value(0), "ID of the camera to read")
		("silent,s", config::bool_switch(&silent)->default_value(false), "suppress outputs")
		("verbose,v", config::bool_switch(&verbose)->default_value(false), "verbose output")
		("debug,d", config::bool_switch(&debug)->default_value(false), "Emit debug information")
		;
	config::variables_map vm;
	config::store(config::parse_command_line(argc, argv, desc), vm);
	config::notify(vm);

	if (vm.count("help")) {
		std::cout << desc << std::endl;
		return 1;
	}

	// ----- Setting up communication and data for images
	// Creating LCM network provider
	lcmImage = lcm_create ("udpm://");
	lcmMavlink = lcm_create ("udpm://");
	if (!lcmImage || !lcmMavlink)
		exit(EXIT_FAILURE);

	std::vector<px::SHMImageClient> clientVec;
	clientVec.resize(4);

        clientVec.at(0).init(true, px::SHM::CAMERA_FORWARD_LEFT);
        clientVec.at(1).init(true, px::SHM::CAMERA_FORWARD_LEFT, px::SHM::CAMERA_FORWARD_RIGHT);
        clientVec.at(2).init(true, px::SHM::CAMERA_DOWNWARD_LEFT);
        clientVec.at(3).init(true, px::SHM::CAMERA_DOWNWARD_LEFT, px::SHM::CAMERA_DOWNWARD_RIGHT);
	mavconn_mavlink_msg_container_t_subscription_t * img_sub  = mavconn_mavlink_msg_container_t_subscribe (lcmImage, "IMAGES", &image_handler, &clientVec);
	mavconn_mavlink_msg_container_t_subscription_t * comm_sub = mavconn_mavlink_msg_container_t_subscribe (lcmMavlink, "MAVLINK", &mavlink_handler, lcmMavlink);

	cout << "MAVLINK client ready, waiting for data..." << endl;

	// Creating thread for MAVLink handling
	GThread* lcm_mavlinkThread;
	GThread* lcm_imageThread;
	GError* err;

	if( !g_thread_supported() ) {
		g_thread_init(NULL);
		// Only initialize g thread if not already done
	}

	// Start thread for handling messages on the MAVLINK channel
	cout << "Starting thread for mavlink handling..." << endl;
	if( (lcm_mavlinkThread = g_thread_create((GThreadFunc)lcm_wait, (void *)lcmMavlink, TRUE, &err)) == NULL)
	{
		cout << "Thread create failed: " << err->message << "!!" << endl;
		g_error_free(err) ;
	}

	if( (lcm_imageThread = g_thread_create((GThreadFunc)lcm_image_wait, (void*)lcmImage, TRUE, &err)) == NULL)
	{
		cout << "Thread create failed: " << err->message << "!!" << endl;
		g_error_free(err);
	}
		cout << "MAVLINK client ready, waiting for requests..." << endl;

	signal(SIGINT, signalHandler);

	while(!quit)
		lcm_handle(lcmMavlink);

	// Clean up
	cout << "Stopping thread for image capture..." << endl;
	g_thread_join(lcm_mavlinkThread);

	cout << "Everything done successfully - Exiting" << endl;

	mavconn_mavlink_msg_container_t_unsubscribe (lcmImage, img_sub);
	mavconn_mavlink_msg_container_t_unsubscribe (lcmMavlink, comm_sub);
	lcm_destroy (lcmImage);
	lcm_destroy (lcmMavlink);

	exit(EXIT_SUCCESS);
}
