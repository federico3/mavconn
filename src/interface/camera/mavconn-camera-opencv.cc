/*=====================================================================

PIXHAWK Micro Air Vehicle Flying Robotics Toolkit

(c) 2009-2011 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>

This file is part of the PIXHAWK project

    PIXHAWK is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    PIXHAWK is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with PIXHAWK. If not, see <http://www.gnu.org/licenses/>.

======================================================================*/

/**
* @file
*   @brief Camera driver.
*
*   This camera driver currently supports the Point Grey Firefly MV and
*   mvBlueFOX cameras, and the OpenCV camera interface.
*
*   @author Lorenz Meier <mavteam@student.ethz.ch>
*   @author Lionel Heng  <hengli@inf.ethz.ch>
*
*/

#include <boost/program_options.hpp>
#include <boost/circular_buffer.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <glibmm.h>
#include <sys/time.h>

#include "interface/shared_mem/SHMImageServer.h"
#include "mavconn.h"
#include "core/MAVConnParamClient.h"

#include "PxCameraManagerFactory.h"

bool verbose = false;
bool emitDelay = false;
int compid = 2;
std::string configFile;		///< Configuration file for parameters

MAVConnParamClient* paramClient;
uint32_t interval = 0;

typedef struct _bufferIMU
{
	mavlink_image_triggered_t msg;
	//uint64_t delay;
} bufferIMU_t;

typedef struct
{
	lcm_t* lcm;
	MAVConnParamClient* client;
	boost::circular_buffer<bufferIMU_t>* dataBuffer;
} camera_thread_context_t;

const int MAGIC_MAX_BUFFER_AND_RETRY = 100;		// Size of the message buffer for LCM messages and maximum number of skipped/dropped frames before stopping when a mismatch happens
const int MAGIC_MIN_SEQUENCE_DIFF = 150;		// *has to be > than MAGIC_MAX_BUFFER_AND_RETRY!* Minimum difference between two consecutively processed images to assume a sequence mismatch
												// In other words: the maximum number of skippable frames
const int MAGIC_MESSAGE_TIMEOUT_US = 5000000;	// Time the process waits for messages if the buffer is empty
const int MAGIC_IMAGE_TIMEOUT_US = 5000000;		// Time the process waits for images
const int MAGIC_MAX_IMAGE_DELAY_US = 5000000;	// Maximum delay allowed between shutter time and start of image processing
const int MAGIC_HARD_RETRY_MUTEX = 2;			// Maximum number of times a mutex is tried to timed lock

Glib::StaticMutex messageMutex;		//mutex controlling the access to the message buffer
Glib::StaticMutex imageMutex;			//mutex controlling the access to the image data
Glib::StaticMutex imageGrabbedMutex;  //separate mutex for the image grabbed condition because libdc grab blocks

Glib::Cond* imageGrabbedCond = 0;
Glib::Cond* processingDoneCond = 0;
Glib::Cond* messageQueueNotEmptyCond = 0;

bool imageGrabbed = false;					//variable to check the validity of the imageGrabbed condition (guarded by imageGrabbed_mutex)
bool processingDone = false;				//variable to check the validity of the processingDone condition (guarded by image_mutex)

namespace config = boost::program_options;

bool quit = false;

void
signalHandler(int signal)
{
	if (signal == SIGINT)
	{
		fprintf(stderr, "# INFO: Quitting...\n");
		quit = true;
	}
}

/**
* @brief Handle a MAVLINK message received from LCM
*
* The message is forwarded to the serial port.
*
* @param rbuf LCM receive buffer
* @param channel LCM channel
* @param msg MAVLINK message
* @param user LCM user
*/
void
mavlinkHandler(const lcm_recv_buf_t* rbuf, const char* channel,
			   const mavconn_mavlink_msg_container_t* container, void* user)
{
	
	boost::circular_buffer<bufferIMU_t>* dataBuffer =
			reinterpret_cast<boost::circular_buffer<bufferIMU_t>*>(user);

	const mavlink_message_t* msg = getMAVLinkMsgPtr(container);

	// Handle param messages
	paramClient->handleMAVLinkPacket(msg);

	//if (Verbose)
	//	printf("Heard message %d from %d!\n",msg->msgid,msg->sysid);

	if (msg->msgid == MAVLINK_MSG_ID_IMAGE_TRIGGERED)
	{

		mavlink_image_triggered_t trigger;
		mavlink_msg_image_triggered_decode(msg, &trigger);
		bufferIMU_t data;
		memcpy(&data.msg, &trigger, sizeof(mavlink_image_triggered_t));

//			gettimeofday(&tv, NULL);
//			uint64_t tt = ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
//			data->delay = tt - trigger.timestamp;
//			printf("got message %u - %llu\n", trigger.seq, (long long unsigned) data->delay);

		//if (verbose) printf("got message %u\n", trigger.seq);

		//get exclusive access to the dataBuffer
		messageMutex.lock();
		//dataBuffer is now locked, so don't waste time
		if (dataBuffer->full())
		{
			if (!verbose)
			{
				fprintf(stderr, "*** CRITICAL PROBLEM: PROCESSING TOO SLOW! MESSAGE BUFFER FULL ***\n");
			}
		}
		else
		{
			dataBuffer->push_back(data);
		}
		messageQueueNotEmptyCond->signal();
		messageMutex.unlock();
		Glib::Thread::yield();
	}
	// TODO implement listener for CMD_DO_CONTROL_VIDEO (check) that listens for new resolution and changes camera parameters.
}

void lcmWait(lcm_t* lcm)
{
	// Blocking wait for new data
	while (!quit)
	{
		lcm_handle(lcm);
	}
}

static inline int getCameraResW(void)   //Modeled after GetSystemID
{
	static bool isCached = false;
	// Return 1280 on error or no config file present
	static int CameraResW = 1280;

	if (!isCached)
	{
		// Read config file
		std::string line;
		std::ifstream configFile("/etc/mavconn/mavconn.conf");
		if (configFile.is_open())
		{
			while (configFile.good())
			{
				std::string key;
				int value;
				configFile >> key;
				configFile >> value;

				key = trimString(key);

				if (key == "cameraresw" && value > 0)
				{
					CameraResW = value;
                    isCached = true;
                    //std::cout<<"Reading SysID from mavconn.conf successful, sysID "<<systemId<<"\n";
				}
			}
		}

	}/*else{
        std::cout<<"Reading cached SysID "<<systemId<<"\n";
    }*/

	return CameraResW;
}

static inline int getCameraResH(void)   //Modeled after GetSystemID
{
	static bool isCached = false;
	// Return 720 on error or no config file present
	static int CameraResH = 720;

	if (!isCached)
	{
		// Read config file
		std::string line;
		std::ifstream configFile("/etc/mavconn/mavconn.conf");
		if (configFile.is_open())
		{
			while (configFile.good())
			{
				std::string key;
				int value;
				configFile >> key;
				configFile >> value;

				key = trimString(key);

				if (key == "cameraresh" && value > 0)
				{
					CameraResH = value;
                    isCached = true;
                    //std::cout<<"Reading SysID from mavconn.conf successful, sysID "<<systemId<<"\n";
				}
			}
		}

	}/*else{
        std::cout<<"Reading cached SysID "<<systemId<<"\n";
    }*/

	return CameraResH;
} 

void
cameraGrab(PxCameraPtr& pxCam, cv::Mat *frame, uint32_t *skippedFrames, uint32_t *sequenceNum)
{
	imageMutex.lock();
	while (!quit)
	{
		if (pxCam->grabFrame(*frame, *skippedFrames, *sequenceNum))
		{
			imageGrabbedMutex.lock();
			imageGrabbed = true;
			imageGrabbedCond->signal();
			imageGrabbedMutex.unlock();
			while (!processingDone && !quit)	// go waiting (releases the lock)
			{
				processingDoneCond->wait(imageMutex);
			}

			processingDone = false;	//reset assert variable
		}
	}
}

void cameraStereoGrab(PxStereoCameraPtr& pxStereoCam,
					  cv::Mat *frame, cv::Mat *frameRight,
					  uint32_t *skippedFrames, uint32_t *sequenceNum)
{
	imageMutex.lock();
	while (!quit)
	{
		if (pxStereoCam->grabFrame(*frame, *frameRight, *skippedFrames, *sequenceNum))
		{
			imageGrabbedMutex.lock();
			imageGrabbed = true;
			imageGrabbedCond->signal();
			imageGrabbedMutex.unlock();
			while (!processingDone && !quit)	// go waiting (releases the lock)
			{
				processingDoneCond->wait(imageMutex);
			}

			processingDone = false;	//reset assert variable
		}
	}
}

int main(int argc, char* argv[])
{
	std::string camType;
	std::string camOrientation;

	uint32_t exposure;	///< Exposure in microseconds
	uint32_t gain;		///< Gain in the internal camera scaling
	uint32_t gamma;		///< Camera gamma
	bool automode;		///< Use auto brightness/gain/exposure/gamma
	float frameRate;	///< Frame rate in Hz

	bool trigger = false;
	bool triggerslave = false;

	uint64_t camSerial = 0;			///< Camera unique id, from hardware
	uint64_t camSerialRight = 0;	///< Right camera unique id, from hardware (non-zero if stereo mode is activated)

	bool detectHorizontal = false;
	uint32_t detectThreshold = 75;

	//========= Handling Program options =========
	config::options_description desc("Allowed options");
	desc.add_options()
									("help", "produce help message")
									("exposure,e", config::value<uint32_t>(&exposure)->default_value(2000), "Exposure in microseconds")
									("gain,g", config::value<uint32_t>(&gain)->default_value(0), "Gain in FIXME")
									("gamma", config::value<uint32_t>(&gamma)->default_value(0), "Gamma in FIXME")
									("fps", config::value<float>(&frameRate)->default_value(60.0f), "Camera fps")
									("trigger,t", config::bool_switch(&trigger)->default_value(false), "Enable hardware trigger (Firefly MV: INPUT: GPIO0, OUTPUT: GPIO2)")
									("triggerslave", config::bool_switch(&triggerslave)->default_value(false), "Enable if another px_camera process is already controlling the imu trigger settings")
									("automode,a", config::bool_switch(&automode)->default_value(false), "Enable auto brightness/gain/exposure/gamma")
									("type", config::value<std::string>(&camType)->default_value("opencv"), "Camera type: {opencv|bluefox|firefly]")
									("serial_right", config::value<uint64_t>(&camSerialRight)->default_value(0), "Enable stereo camera mode. Expects serial # of the right camera as argument. This will also enable (and only work with) the hardware trigger. Left cam is master.")
									("serial", config::value<uint64_t>(&camSerial)->default_value(0), "Serial # of the camera to select")
									("orientation", config::value<std::string>(&camOrientation)->default_value("downward"), "Orientation of camera: [downward|forward]")
									("detectHorizontal,h", config::bool_switch(&detectHorizontal)->default_value(false), "Tries to detect horizontally scrambled images")
									("detectThreshold,h", config::value<uint32_t>(&detectThreshold)->default_value(75), "Threshold for horizontal detector (Pixel count)")
									("verbose,v", config::bool_switch(&verbose)->default_value(false), "Verbose output")
									("delay", config::bool_switch(&emitDelay)->default_value(false), "emit Delays as debug message")
									("config", config::value<std::string>(&configFile)->default_value("config/parameters_camera.cfg"), "Config file for parameters")
									;
	config::variables_map vm;
	config::store(config::parse_command_line(argc, argv, desc), vm);
	config::notify(vm);

	if (vm.count("help"))
	{
		std::cout << desc << std::endl;
		return 1;
	}

	signal(SIGINT, signalHandler);

	//========= Initialize LCM =========
	lcm_t* lcm = lcm_create("udpm://");
	if (!lcm)
	{
		exit(EXIT_FAILURE);
	}

	//========= Initialize threading =========
	Glib::Thread* lcmThread = 0;
	Glib::Thread* imageThread = 0;
	Glib::TimeVal waitTime;

	// Only initialize g thread if not already done
	if (!Glib::thread_supported())
	{
		Glib::thread_init();
	}

	// Init LCM message handler mutex/condition
	if (!messageQueueNotEmptyCond)
	{
		messageQueueNotEmptyCond = new Glib::Cond;
	}

	//guarded by messageMutex -->
	boost::circular_buffer<bufferIMU_t> dataBuffer(MAGIC_MAX_BUFFER_AND_RETRY);
	boost::circular_buffer<bufferIMU_t>::iterator dataIterator = dataBuffer.begin();
	//<-- guarded by messageMutex

	mavconn_mavlink_msg_container_t_subscription_t* mavlinkSub = NULL;
	// Modified by Federico on 150714: subscribes to MAVLINK UDP stream even if no trigger
	
	// Initialize paramClient early
    paramClient = new MAVConnParamClient(getSystemID(), compid, lcm, configFile, verbose);
    paramClient->setParamValue("MINIMGINTERVAL", 0);
    paramClient->setParamValue("EXPOSURE", exposure);
    paramClient->setParamValue("GAIN", gain);
	paramClient->setParamValue("RESW", getCameraResW());
	paramClient->setParamValue("RESH", getCameraResH());
    paramClient->readParamsFromFile(configFile);
	
	/*// Create a data structure to connect parameter client to LCM
	camera_thread_context_t thread_context;
	thread_context.lcm = lcm;
	thread_context.client = paramClient;
	thread_context.dataBuffer = &dataBuffer;*/
	
	//if (trigger)
	//{
	
		mavlinkSub = mavconn_mavlink_msg_container_t_subscribe(lcm, MAVLINK_MAIN, &mavlinkHandler, &dataBuffer);
		if (!verbose)
		{
			fprintf(stderr, "# INFO: Subscribed to %s LCM channel.\n", MAVLINK_MAIN);
		}
	
		try
		{
			lcmThread = Glib::Thread::create(sigc::bind(sigc::ptr_fun(lcmWait), lcm), true);
		}
		catch (const Glib::ThreadError& e)
		{
			fprintf(stderr, "# ERROR: Cannot create LCM handling thread.\n");
			exit(EXIT_FAILURE);
		}
		//}
	// End modification

	// Init image grabbing mutex/conditions
	// they are used only with libdc1394 capturing
	// the capturing thread is started only if

	if (!imageGrabbedCond)
	{
		imageGrabbedCond = new Glib::Cond;
	}

	if (!processingDoneCond)
	{
		processingDoneCond = new Glib::Cond;
	}



	//========= Initialize capture devices =========
	fprintf(stderr, "# INFO: Creating capture...\n");

	PxCameraManagerPtr camManager = PxCameraManagerFactory::generate(camType);
	if (camManager.get() == 0)
	{
		fprintf(stderr, "# ERROR: Unknown camera type: %s\n. Please choose ONLY opencv.\n", camType.c_str());
		exit(EXIT_FAILURE);
	}

	PxCameraPtr pxCam;
	PxStereoCameraPtr pxStereoCam;

	bool useStereo = false;
	int cameraCount = camManager->getCameraCount();

	if (camSerialRight == 0)
	{
		if (cameraCount > 0)
		{
			pxCam = camManager->generateCamera(camSerial);
			if (pxCam.get() == 0)
			{
				exit(EXIT_FAILURE);
			}
		}
		else
		{
			fprintf(stderr, "# ERROR: no camera present.\n");
			exit(EXIT_FAILURE);
		}
	}
	else
	{
		if (cameraCount > 1)
		{
			pxStereoCam = camManager->generateStereoCamera(camSerial, camSerialRight);
			if (pxStereoCam.get() == 0)
			{
				exit(EXIT_FAILURE);
			}

			useStereo = true;
		}
		else
		{
			fprintf(stderr, "# ERROR: only %d camera(s) present. 2 cameras are required for stereo.\n", cameraCount);
			exit(EXIT_FAILURE);
		}
	}

	if (trigger && camType.compare("opencv") == 0)
	{
		fprintf(stderr, "# ERROR: OpenCV camera does not support triggering.\n");
		exit(EXIT_FAILURE);
	}

	// Disable trigger
	if (!verbose)
	{
		fprintf(stderr, "# INFO: Disabling trigger until camera is initialized..\n");
	}

	//========= Initialize Shared memory =========
	px::SHM::Camera cam = px::SHM::CAMERA_NONE;
	px::SHM::Camera camRight = px::SHM::CAMERA_NONE;
	if (camOrientation.compare("downward") == 0)
	{
		if (useStereo)
		{
			camRight = px::SHM::CAMERA_DOWNWARD_RIGHT;
		}

		cam = px::SHM::CAMERA_DOWNWARD_LEFT;
	}
	else if (camOrientation.compare("forward") == 0)
	{
		if (useStereo)
		{
			camRight = px::SHM::CAMERA_FORWARD_RIGHT;
		}

		cam = px::SHM::CAMERA_FORWARD_LEFT;
	}
	else
	{
		fprintf(stderr, "# ERROR: Unknown camera orientation: %s\n.", camOrientation.c_str());
		exit(EXIT_FAILURE);
	}

	px::SHMImageServer server;
	server.init(getSystemID(), PX_COMP_ID_CAMERA, lcm, cam, camRight);

	if(trigger && !triggerslave)
	{
		// send multiple times as a soft mean of forward error correction
		for (int i = 0; i < 2; i++)
		{
			mavlink_message_t triggerMsg;
			mavlink_msg_image_trigger_control_pack(getSystemID(), PX_COMP_ID_CAMERA, &triggerMsg, 0);
			sendMAVLinkMessage(lcm, &triggerMsg);
		}
	}

	PxCameraConfig::Mode mode = PxCameraConfig::MANUAL_MODE;
	if (automode)
	{
		mode = PxCameraConfig::AUTO_MODE;
	}
	PxCameraConfig config(mode, frameRate, trigger, exposure, gain, gamma,2047,12000,getCameraResW(),getCameraResH());

	if (useStereo)
	{
		fprintf(stderr, "# INFO: Opening stereo with serial #%llu and #%llu, trigger is: enabled\n", (long long unsigned) camSerial, (long long unsigned) camSerialRight);
		if (!pxStereoCam->init())
		{
			fprintf(stderr, "# ERROR: Cannot initialize stereo setup.\n");
			exit(EXIT_FAILURE);
		}

		pxStereoCam->setConfig(config);

		if (!pxStereoCam->start())
		{
			fprintf(stderr, "# ERROR: Cannot start stereo setup.\n");
			exit(EXIT_FAILURE);
		}
	}
	else
	{
		fprintf(stderr, "# INFO: Opening camera with serial #%llu, trigger is: %s\n", (long long unsigned) camSerial, (trigger) ? "enabled" : "disabled");
		if (!pxCam->init())
		{
			fprintf(stderr, "# ERROR: Cannot initialize camera setup.\n");
			exit(EXIT_FAILURE);
		}

		pxCam->setConfig(config);

		if (!pxCam->start())
		{
			fprintf(stderr, "# ERROR: Cannot start camera setup.\n");
			exit(EXIT_FAILURE);
		}
	}

	if (trigger)
	{
		// clear buffer of messages that came in before IMU stopped triggering
		messageMutex.lock();
		while (!dataBuffer.empty())
		{
			dataIterator = dataBuffer.begin();
			dataBuffer.pop_front();
		}
		messageMutex.unlock();
	}

	uint64_t lastShutter = 0;
//	uint64_t lastMessageDelay = 0;
	uint64_t timestamp = 0;
	uint64_t lastTimestamp = 0;
	mavlink_image_triggered_t image_data;
	uint32_t lastSequenceNum = 0;		// the embedded sequence number of the image before
	uint32_t lastMessageSequence = 0;		// the sequence number of the last used message
	uint32_t messageSequence = 0;			// the sequence number of the current message
	uint32_t recoverNumberOfFrames = 0;	// this variable is always 0 if running without errors, if a mismatch between skipped frames and image sequence numbers happen this variable stores the number of frames in buffer since the last trusted image
	bool firstFrameRubbishCheck = false;	// will be set to true after first frame

	fprintf(stderr, "# INFO: Grabbing one frame to get image size...\n");
	fflush(stderr);

	//guarded by image_mutex -->
	cv::Mat frame;
	cv::Mat frameRight;
	uint32_t skippedFrames = 0;			// this variable will be written by the grabbing function: number of frames skipped in the buffer to get the newest image
	uint32_t sequenceNum = 0;			// the embedded sequence number of the current image
	//<-- guarded by image_mutex

	//========= Grab first frame to get the image size =========
	if (trigger)
	{
		imageGrabbedMutex.lock(); // guard the imageGrabbed condition with the separate mutex to be able to catch the blocking libdc grab function

		//start the libdc1394 grabbing thread
		try
		{
			if (useStereo)
			{
				imageThread = Glib::Thread::create(sigc::bind(sigc::ptr_fun(cameraStereoGrab), pxStereoCam, &frame, &frameRight, &skippedFrames, &sequenceNum), true);
			}
			else
			{
				imageThread = Glib::Thread::create(sigc::bind(sigc::ptr_fun(cameraGrab), pxCam, &frame, &skippedFrames, &sequenceNum), true);
			}
		}
		catch (const Glib::ThreadError& e)
		{
			fprintf(stderr, "# ERROR: Cannot create frame grabbing thread.\n");
			exit(EXIT_FAILURE);
		}

		if(!triggerslave)
		{
			// Re-enable trigger, as the camera interface is now ready to capture frames
			mavlink_message_t triggerMsg;
			mavlink_msg_image_trigger_control_pack(getSystemID(), PX_COMP_ID_CAMERA, &triggerMsg, 1);
			sendMAVLinkMessage(lcm, &triggerMsg);
			if (!verbose)
			{
				fprintf(stderr, "# INFO: Trigger re-enabled, using hardware triggering.\n");
			}
		}

		// now the grabbing thread is started and we can start waiting for the first frame to arrive
		waitTime.assign_current_time();
		waitTime.add(Glib::TimeVal(MAGIC_IMAGE_TIMEOUT_US / 1000000, MAGIC_IMAGE_TIMEOUT_US % 1000000));	//wait 1 second for the first frame
		if (triggerslave)
		{
			//wait longer as trigger slave for the first image
			waitTime.add(Glib::TimeVal(MAGIC_IMAGE_TIMEOUT_US / 1000000, MAGIC_IMAGE_TIMEOUT_US % 1000000));	//wait 2 seconds for the first frame
		}

		bool wait = true;
		while (wait && !imageGrabbed)
		{
			wait = imageGrabbedCond->timed_wait(imageGrabbedMutex, waitTime);
		}

		imageGrabbed = false;	//reset the assert variable
		imageGrabbedMutex.unlock();
		if (wait == false)	//timed out?
		{
			fprintf(stderr, "# ERROR: Waiting for frame timed out! Is the Link OK and px_mavlinkserial running?\n");
			exit(EXIT_FAILURE);
		}
	}
	else
	{
		if (useStereo)
		{
			if (!pxStereoCam->grabFrame(frame, frameRight, skippedFrames, sequenceNum))
			{
				fprintf(stderr, "# ERROR: Cannot grab first frame.\n");
				exit(EXIT_FAILURE);
			}
		}
		else
		{
			if (!pxCam->grabFrame(frame, skippedFrames, sequenceNum))
			{
				fprintf(stderr, "# ERROR: Cannot grab first frame.\n");
				exit(EXIT_FAILURE);
			}
		}
	}

	struct timeval tv;
	gettimeofday(&tv, NULL);
	timestamp = ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;

	lastSequenceNum = sequenceNum;
	fprintf(stderr, "# INFO: skipped %u / image seq: %u ", skippedFrames, sequenceNum);
	fflush(stderr);

	//========= if the trigger is activated initialize the sequence numbers =========
	if (trigger)
	{
		//here we have the initialization problem, we assume that the camera started with sequence number
		//lastSequenceNum - skippedFrames
		//this means that we have to skip to the trigger message with ID skippedFrames+1 because the IMU started
		//with message sequence number 1

		// get the trigger message for the first frame
		bool found = false;			//true if the matching message was found
		while (!found)
		{
			// if the buffer is empty, wait until there is something
			messageMutex.lock();
			if (dataBuffer.empty())
			{
				waitTime.assign_current_time();
				waitTime.add(Glib::TimeVal(MAGIC_MESSAGE_TIMEOUT_US / 1000000, MAGIC_MESSAGE_TIMEOUT_US % 1000000));
				bool wait = true;
				while (wait && dataBuffer.empty())
				{
					wait = messageQueueNotEmptyCond->timed_wait(messageMutex, waitTime);
				}
				if (wait == false)
				{
					fprintf(stderr, "# ERROR: No more messages from the IMU - cable error or mavlinkserial dead?\n");
					exit(EXIT_FAILURE);
				}
			}

			// there is something in the buffer, read the sequence number of the message
			dataIterator = dataBuffer.begin();
			lastMessageSequence = ((bufferIMU_t)(*dataIterator)).msg.seq;

			// three cases are possible:
			// the message has a lower ID then we have to skip this message
			if (lastMessageSequence < skippedFrames+1)
			{
				dataBuffer.pop_front();
			}
			// the message has the right sequence number, skip the message and we're done
			else if (lastMessageSequence == skippedFrames+1)
			{
				lastShutter = ((bufferIMU_t)(*dataIterator)).msg.timestamp;
//				lastMessageDelay = ((bufferIMU_t)(*dataIterator)).delay;
				dataBuffer.pop_front();
				found = true;
				fprintf(stderr, " / message seq: %u / timestamp: %llu - ", lastMessageSequence, (long long unsigned) lastShutter);
			}
			// the message has a higher sequence number, this means, assuming that all messages
			// are chronologically ordered (no flipping in order that CAN happen with UDP!), that
			// the trigger message for this image was lost, or mavlinkserial was not running.
			// Because this happened in the crucial initialization we assume nothing and exit the program here.
			else
			{
				fprintf(stderr, "# ERROR: Error getting first message! Is the Link OK and px_mavlinkserial running?\n");
				exit(EXIT_FAILURE);
			}
			messageMutex.unlock();
		}
	}
	else
	{
		// Initialize lastTime
		// this initialization will make the first valid_until interval
		// extremely short, effectively eliminating the first frame
		struct timeval tv;
		gettimeofday(&tv, NULL);
		lastShutter = ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
	}

	cv::Size frameSize = frame.size();
	fprintf(stderr, " %ix%i ", frameSize.width, frameSize.height);
	fflush(stderr);
	cv::Mat gray(frameSize, CV_8UC1);
	cv::Mat gray2(frameSize, CV_8UC1);

	/*if (trigger && (timestamp < lastShutter+lastMessageDelay))
	{
		g_mutex_lock(image_mutex);
		processingDone = true;
		g_cond_signal(processingDone_cond);
		g_mutex_unlock(image_mutex);
		g_thread_yield();
		printf(" First image arrived before Shutter - this is an error, regrab first frame!\n");
		g_get_current_time(&wait_time);
		g_time_val_add (&wait_time, MAGIC_IMAGE_TIMEOUT_US);	//wait 1 second for the first frame
		bool wait = true;
		while (wait && !imageGrabbed)
			wait = g_cond_timed_wait(imageGrabbed_cond, imageGrabbed_mutex, &wait_time);

		imageGrabbed = false;	//reset the assert variable
		g_mutex_unlock(imageGrabbed_mutex);
		if (wait == false)	//timed out?
		{
			printf("Waiting for frame timed out! Is the Link OK and px_mavlinkserial running?\n");
			exit(EXIT_FAILURE);
		}
	}*/

	fprintf(stderr, " done, camera running ");
	if (trigger)
	{
		fprintf(stderr, "with trigger...\n");
	}
	else
	{
		fprintf(stderr, "without trigger...\n");
	}

	fprintf(stderr, "\nYou can view the camera image by launching mavconn-view\n");

	uint8_t grabFailCount = 0;

	//========= MAIN LOOP =========
	while (!quit)
	{
		//First check if parameters changed:
		bool changed = false;
		uint32_t newExposureTime = (uint32_t)paramClient->getParamValue("EXPOSURE");
		uint32_t newGain = (uint32_t)paramClient->getParamValue("GAIN");
		uint32_t newResH = (uint32_t)paramClient->getParamValue("RESH");
		uint32_t newResW = (uint32_t)paramClient->getParamValue("RESW");
			
		if (newExposureTime != config.getExposureTime())
		{
			config.setExposureTime(newExposureTime);
			changed = true;
		}
		if (newGain != config.getGain())
		{
			config.setGain(newGain);
			changed = true;
		}
		// TODO add check for image resolution along the lines of what we have above
		if (newResH != config.getResH() || newResW != config.getResW())
		{
			config.setResW(newResW);
			config.setResH(newResH);
			changed = true;
		}
		
		
		if (changed)
		{
			if (useStereo)
			{
				pxStereoCam->setConfig(config);
			}
			else
			{
				pxCam->setConfig(config);
			}
		}

		if (trigger)
		{
			imageMutex.lock();
			processingDone = true;
			processingDoneCond->signal();
			imageMutex.unlock();
			Glib::Thread::yield();

			// signal the grabbing thread that the next image can be grabbed
			waitTime.assign_current_time();
			waitTime.add(Glib::TimeVal(MAGIC_IMAGE_TIMEOUT_US / 1000000, MAGIC_IMAGE_TIMEOUT_US % 1000000));	//wait 1 second for the frame
			bool wait = true;
			while (wait && !imageGrabbed)
			{
				wait = imageGrabbedCond->timed_wait(imageGrabbedMutex, waitTime);
			}

			imageGrabbed = false;	//reset the assert variable
			imageGrabbedMutex.unlock();

			if (quit)
			{
				break;
			}

			if (wait == false)	//timed out?
			{
				fprintf(stderr, "# ERROR: Waiting for frame timed out! Possible problems:\n-* The data link to the IMU is not OK\n* px_mavlinkserial is not running\n* If in BlueFox stereo mode: cameras desynchronized, camera process restarts to fix the problem.");
				exit(EXIT_FAILURE);
			}
		}
		else
		{
			if (useStereo)
			{
				if (!pxStereoCam->grabFrame(frame, frameRight, skippedFrames, sequenceNum))
				{
					grabFailCount++;
					if (!verbose)
					{
						fprintf(stderr, "# INFO: Cannot grab frame.\n");
					}
				}
				else
				{
					grabFailCount = 0;
				}
			}
			else
			{
				if (!pxCam->grabFrame(frame, skippedFrames, sequenceNum))
				{
					grabFailCount++;
					if (!verbose)
					{
						fprintf(stderr, "# INFO: Cannot grab frame.\n");
					}
				}
				else
				{
					grabFailCount = 0;
				}
			}
		}

		if(grabFailCount > 4)
		{
			fprintf(stderr, "# ERROR: Too many consecutive grab fails, exiting.\n");
			exit(-1);
		}

		// 	Get timestamp immediately after image capture
		struct timeval tv;
		gettimeofday(&tv, NULL);
		timestamp = ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;

//		if (detectHorizontal)
//		{
//			//check for horizontally scrambled images
//			//uint32_t diffs[height];
//			//memset(diffs, 0, height*sizeof(uint32_t));
//			for (int y = 0; y < frame.rows-1; y++)
//			{
//				uint32_t diff = 0;
//				for (int x = 0; x < frame.cols; x++)
//				{
//					diff += abs(frame.at<uint8_t>(y,x) - frame.at<uint8_t>(y+1,x));
//				}
//				if (diff > (detectThreshold * 255))	// 100 pixels with big difference
//				{
//					if (verbose)
//					{
//						fprintf(stderr, "# WARNING: Image scrambled, dropping frame!\n");
//					}
//					continue;
//				}
//			}
//		}

		if (trigger)
		{
			//check if every is fine with the image sequence
			if ((sequenceNum - lastSequenceNum) == (skippedFrames + 1))
			{
				// everything is fine
				if (verbose)
				{
					fprintf(stderr, "# INFO: Image sequence ok: Current: %u Last: %u Skipped: %u\n", sequenceNum, lastSequenceNum, skippedFrames);
				}
				lastSequenceNum = sequenceNum;
				recoverNumberOfFrames = 0;
			}
			else	// the image sequence difference does not match the number of skipped frames
			{		// there are 2 possibilities: either the image scrambled due to a transmission error
					// or the libdc buffer was full so a frame got dropped
					// consequence 1: we will drop this frame
					// consequence 2: the image sequence number cannot be trusted

					// actually we can't recover at this moment anymore, because when transmission errors happen,
					// more images are taken but they are not transmitted. The sequence number grows but no images are received
					// so the next correct image has a sequence number greater by 1 or more

					// the only way to recover is to accept images with a sequence number difference smaller than a threshold

				if (!verbose)
				{
					fprintf(stderr, "# INFO: Image sequence mismatch! Image scrambled or buffer overflow, trying to recover...");
				}
				if (sequenceNum-lastSequenceNum > 150 || (int32_t)sequenceNum == -1)
				{
					if (!firstFrameRubbishCheck) //this is the second frame, give it a second chance
					{
						firstFrameRubbishCheck = true;
						if (verbose)
						{
							fprintf(stderr, "# INFO: Sequence mismatch between first and second frame - just forget the first, retry with the second as first frame\n");
						}
						recoverNumberOfFrames = 0;
						lastSequenceNum = sequenceNum;
					}
					else
					{
						recoverNumberOfFrames += skippedFrames+1;
						if (recoverNumberOfFrames > static_cast<uint32_t>(MAGIC_MAX_BUFFER_AND_RETRY))
						{
							fprintf(stderr, " OK, enough now, try again Sam...\n");
							exit(EXIT_FAILURE);
						}
					}
				}
				else
				{
					skippedFrames = sequenceNum-lastSequenceNum-1;
					if (!verbose)
					{
						fprintf(stderr, "Recovered: %u problematic frames were skipped during the time and sequence number raised by %u (number of lost frames)\n", recoverNumberOfFrames, skippedFrames);
					}
					recoverNumberOfFrames = 0;
					lastSequenceNum = sequenceNum;
				}
			}
		}

		if (recoverNumberOfFrames > 0)
		{
			if (!verbose)
			{
				fprintf(stderr, " Dropping problematic frame / skipped %u frames, Seq: %u\n", skippedFrames, sequenceNum);
			}
		}
		else
		{
			bool not_in_list = false;
			if (trigger)
			{
				bool found = false;
				while (!found && !not_in_list)
				{
					// wait while message buffer is empty
					messageMutex.lock();
					if (dataBuffer.empty())
					{
						waitTime.assign_current_time();
						waitTime.add(Glib::TimeVal(MAGIC_MESSAGE_TIMEOUT_US / 1000000, MAGIC_MESSAGE_TIMEOUT_US % 1000000));
						bool wait = true;
						while (wait && dataBuffer.empty())
						{
							wait = messageQueueNotEmptyCond->timed_wait(messageMutex, waitTime);
						}
						if (wait == false)
						{
							fprintf(stderr, "# ERROR: No more messages from the IMU - cable error or mavlinkserial dead?\n");
							exit(EXIT_FAILURE);
						}
					}

					// there is something in the buffer, read the sequence number of the message
					dataIterator = dataBuffer.begin();
					messageSequence = ((bufferIMU_t)(*dataIterator)).msg.seq;

					uint32_t neededMessageSequence = lastMessageSequence + skippedFrames + 1;

					// three cases are possible:
					// the message has a lower sequence number then we have to skip this message
					if (messageSequence < neededMessageSequence)
					{
						// check if we get the same sequence number over and over again
						dataBuffer.pop_front();
					}
					// the message has the right sequence number, read the data do stuff and so on
					else if (messageSequence == neededMessageSequence)
					{
					    bufferIMU_t dibIMU = (bufferIMU_t)(*dataIterator);
						lastShutter = ((bufferIMU_t)(*dataIterator)).msg.timestamp;
						memcpy(&image_data, &(dibIMU.msg), sizeof(mavlink_image_triggered_t));
						if (timestamp > lastShutter)
						{
							if (verbose)
							{
								fprintf(stderr, "# INFO: Delay: %llu ms\n", (long long unsigned) (timestamp - lastShutter)/1000);
							}

							if (emitDelay)
							{
								mavlink_message_t msg;
								mavlink_msg_debug_vect_pack(getSystemID(), 9, &msg, "CAM", lastShutter, (float)(timestamp - lastShutter)/1000.f, 0.f, 0.f);
								sendMAVLinkMessage(lcm, &msg);
							}

							//sanity check, timestamp has to be < 1000 ms in any case, otherwise there is an sync error with the IMU timer
							/*if (timestamp - lastShutter > MAGIC_MAX_IMAGE_DELAY_US)
							{
								printf("Delay too long (%llu ms) - IMU time propably not synchronized correctly. Closing...\n", (long long unsigned) (timestamp - lastShutter)/1000);
								exit(EXIT_FAILURE);
							}*/
						}
						else	// catch the negative case
						{
							if (verbose)
							{
								fprintf(stderr, "# INFO: Delay: -%llu ms\n", (long long unsigned) (lastShutter - timestamp)/1000);
							}

							if (emitDelay)
							{
								mavlink_message_t msg;
								mavlink_msg_debug_vect_pack(getSystemID(), 9, &msg, "CAM", lastShutter, -(float)(lastShutter - timestamp)/1000.f, 0.f, 0.f);
								sendMAVLinkMessage(lcm, &msg);
							}

							//printf("WARNING: IMU Time before Systemtime!\n");
							//exit(EXIT_FAILURE);

							//sanity check, timestamp has to be < 1000 ms in any case, otherwise there is an sync error with the IMU timer
							/*if (lastShutter - timestamp  > MAGIC_MAX_IMAGE_DELAY_US)
							{
								printf("Delay too long (-%llu ms) - IMU time propably not synchronized correctly. Closing...\n", (long long unsigned) (lastShutter - timestamp)/1000);
								exit(EXIT_FAILURE);
							}*/
						}

						timestamp = lastShutter;
						dataBuffer.pop_front();
						found = true;
						lastMessageSequence = neededMessageSequence;
					}
					// the message has a higher sequence number, this means, assuming that all messages
					// are chronologically ordered (no flipping in order that CAN happen with UDP!), that
					// we trigger message for this image was lost. We leave the message in the buffer and stop here.
					else
					{
						if (verbose)
						{
							fprintf(stderr, "# INFO: No matching trigger message for image %u. (expected: %u - message seq: %u - last message seq: %u) Dropping frame.\n", sequenceNum, neededMessageSequence, messageSequence, lastMessageSequence);
						}
						not_in_list = true;
						lastMessageSequence = neededMessageSequence;
					}
					messageMutex.unlock();
				}
			}
			else
			{
				// no trigger, do nothing - the timestamp is the one directly after grabbing the current frame
			}

			if (!not_in_list) //if trigger activated, only if a matching message has been found, without trigger do always
			{
				if (verbose)
				{
					fprintf(stderr, "# INFO: Skipped %u frames / image seq: %u / message seq: %u\n", skippedFrames, sequenceNum, lastMessageSequence);
				}
				else
				{
					if (skippedFrames > 0)
					{
						if (!verbose)
						{
							fprintf(stderr, "# INFO: Skipped %u frames!\n", skippedFrames);
						}
					}
				}

				//Skipping frames to obey to minimum interval given as parameter
				if(lastTimestamp == 0 || (timestamp - lastTimestamp) > (uint64_t)paramClient->getParamValue("MINIMGINTERVAL"))
				{
					lastTimestamp = timestamp;
					if (useStereo)
					{
						server.writeStereoImage(frame, camSerial, frameRight, camSerialRight, timestamp, image_data, exposure);
					}
					else
					{
						if (frame.channels() > 1)
						{
							cv::cvtColor(frame, gray, CV_RGB2GRAY);
						}
						else
						{
							frame.copyTo(gray);
						}
                        server.writeMonoImage(gray, camSerial, timestamp, image_data, exposure);
						//server.writeMonoImage(frame, camSerial, timestamp, image_data, exposure);
					}
				}
			}
		} // if matched sequence or no trigger
		firstFrameRubbishCheck = true;
	} // main loop

	if (useStereo)
	{
		pxStereoCam->stop();
	}
	else
	{
		pxCam->stop();
	}

	if (trigger)
	{
		mavconn_mavlink_msg_container_t_unsubscribe(lcm, mavlinkSub);
		lcmThread->join();
		//imageThread->join();
		usleep(1000000); //instead of joining which can hang forever when camera crashed just sleep 100ms
	}

	// Disconnect from LCM
	lcm_destroy(lcm);

	exit(EXIT_SUCCESS);
}
