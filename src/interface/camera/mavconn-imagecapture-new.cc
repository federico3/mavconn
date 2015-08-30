/*=====================================================================

PIXHAWK Micro Air Vehicle Flying Robotics Toolkit

(c) 2009, 2010 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>

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
 *   @brief LCM/SYS-V Shared memory image client
 *
 *   Process for capturing image and GPS data.
 *
 *   @author Bernhard Zeisl
 *
 */

#include <inttypes.h>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <glib.h>
#include <time.h>
#include <signal.h>
#include <pwd.h>
#include <sys/stat.h>

// BOOST includes
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "interface/shared_mem/SHMImageClient.h"
#include "mavconn.h"

namespace config = boost::program_options;
namespace bfs = boost::filesystem;

int sysid = getSystemID();
int compid = 30;
bool silent, verbose, debug, stereo_front;

int imuid = 200; ///< Component ID to read attitude from
float backupRoll;
float backupPitch;
float backupYaw;

#define CAPTURE_DIR			"dataset_capture/"
#define DIRECTUION_0_DIR		"downward/"
#define DIRECTUION_1_DIR		"forward/"
#define IMAGE_CAPTURE_FILE		"imagedata.txt"
#define PLAIN_CAPTURE_FILE		"imagedata_extended.txt"
#define MULTI_COMPONENT_CAPTURE_FILE	"imagedata_extended_multi_component.txt"

using namespace std;

string captureDir;
ofstream imageDataFileDirection0;
ofstream imageDataFileDirection1;
ofstream plainLogFileDirection0;
ofstream plainLogFileDirection1;
ofstream plainLogFileMultiDirection0;
ofstream plainLogFileMultiDirection1;
ofstream mavlinkFile;

string calibStrDirection0;
string calibStrLeftDirection0;
string calibStrRightDirection0;
string calibStrDirection1;
string calibStrLeftDirection1;
string calibStrRightDirection1;

uint64_t imgNum = 0;

bool recordData = false;
bool bPause = false;

bool calib_problems = false;

GAsyncQueue *image_write_queue = NULL;

//image information to store
uint64_t timestamp;
float roll = 0.f, pitch = 0.f, yaw = 0.f, ground_dist = 0.f, pres_alt = 0.f, local_x = 0.f, local_y = 0.f, local_z = 0.f, vx = 0.f, vy = 0.f, vz = 0.f, gx = 0.f, gy = 0.f, gz = 0.f, gvx = 0.f, gvy = 0.f, gvz = 0.f;
float local_x_gps_raw = 0.f, local_y_gps_raw = 0.f, local_z_gps_raw = 0.f;
double lat = 0.f, lon = 0.f, alt = 0.f, vdop = 0.f, hdop = 0.f, satcount = 0.f;

bool gotFirstImage = false;
bool quit = false;

typedef struct _writeData
{
	int direction;
	bool stereo;
	cv::Mat imgLeft;
	cv::Mat imgRight;
	uint64_t timestamp;
	float roll, pitch, yaw, pres_alt, ground_dist, local_x, local_y, local_z, vx, vy, vz, gx, gy, gz, gvx, gvy, gvz;
	float local_x_gps_raw, local_y_gps_raw, local_z_gps_raw;
	double lat, lon, alt, vdop, hdop, satcount;
} writeData;

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

static void image_writer (void)
{
	while(!quit)
	{
		//TODO: use timed_pop() here so that the process can be killed nicely...
		writeData *data = (writeData *) g_async_queue_pop(image_write_queue);

		char fileName[128];
		sprintf(fileName, "%llu.bmp", (long long unsigned)data->timestamp);

		std::string strDirection;
		if (data->direction == 0)
		{
			strDirection = DIRECTUION_0_DIR;
			imageDataFileDirection0.precision(32);
			imageDataFileDirection0 << data->timestamp << ", " << data->roll << ", " << data->pitch << ", " << data->yaw << ", " << data->lat << ", " << data->lon << ", " << data->alt << ", " << data->ground_dist << ", " << data->gx << ", " << data->gy << ", " << data->gz << endl;
			plainLogFileDirection0.precision(32);
			plainLogFileDirection0 << data->timestamp << ", " << data->roll << ", " << data->pitch << ", " << data->yaw << ", " << data->lat << ", " << data->lon << ", " << data->alt << ", " << data->pres_alt << ", " << data->ground_dist << ", " << data->vdop << ", " << data->hdop << ", " << data->satcount << ", " << data->local_x_gps_raw << ", " << data->local_y_gps_raw << ", " << data->local_z_gps_raw << ", " << data->vx << ", " << data->vy << ", " << data->vz << ", " << data->gx << ", " << data->gy << ", " << data->gz << ", " << data->gvx << ", " << data->gvy << ", " << data->gvz << endl;
			plainLogFileMultiDirection0.precision(32);
			plainLogFileMultiDirection0 << data->timestamp << ", " << data->roll << ", " << data->pitch << ", " << data->yaw << ", " << data->lat << ", " << data->lon << ", " << data->alt << ", " << data->pres_alt << ", " << data->ground_dist << ", " << data->vdop << ", " << data->hdop << ", " << data->satcount << ", " << data->local_x_gps_raw << ", " << data->local_y_gps_raw << ", " << data->local_z_gps_raw << ", " << data->local_x << ", " << data->local_y << ", " << data->local_z << ", " << data->vx << ", " << data->vy << ", " << data->vz << ", " << data->gx << ", " << data->gy << ", " << data->gz << ", " << data->gvx << ", " << data->gvy << ", " << data->gvz << endl;
		}
		else
		{
			strDirection = DIRECTUION_1_DIR;
			imageDataFileDirection1.precision(32);
			imageDataFileDirection1 << data->timestamp << ", " << data->roll << ", " << data->pitch << ", " << data->yaw << ", " << data->lat << ", " << data->lon << ", " << data->alt << ", " << data->ground_dist << ", " << data->gx << ", " << data->gy << ", " << data->gz << endl;
			plainLogFileDirection1.precision(32);
			plainLogFileDirection1 << data->timestamp << ", " << data->roll << ", " << data->pitch << ", " << data->yaw << ", " << data->lat << ", " << data->lon << ", " << data->alt << ", " << data->pres_alt << ", " << data->ground_dist << ", " << data->vdop << ", " << data->hdop << ", " << data->satcount << ", " << data->local_x_gps_raw << ", " << data->local_y_gps_raw << ", " << data->local_z_gps_raw << ", " << data->vx << ", " << data->vy << ", " << data->vz << ", " << data->gx << ", " << data->gy << ", " << data->gz << ", " << data->gvx << ", " << data->gvy << ", " << data->gvz << endl;
			plainLogFileMultiDirection1.precision(32);
			plainLogFileMultiDirection1 << data->timestamp << ", " << data->roll << ", " << data->pitch << ", " << data->yaw << ", " << data->lat << ", " << data->lon << ", " << data->alt << ", " << data->pres_alt << ", " << data->ground_dist << ", " << data->vdop << ", " << data->hdop << ", " << data->satcount << ", " << data->local_x_gps_raw << ", " << data->local_y_gps_raw << ", " << data->local_z_gps_raw << ", " << data->local_x << ", " << data->local_y << ", " << data->local_z << ", " << data->vx << ", " << data->vy << ", " << data->vz << ", " << data->gx << ", " << data->gy << ", " << data->gz << ", " << data->gvx << ", " << data->gvy << ", " << data->gvz << endl;
		}

		if (data->stereo)
		{
			cv::imwrite((captureDir + strDirection + std::string("left/") + fileName).c_str(), data->imgLeft);
			cv::imwrite((captureDir + strDirection + std::string("right/") + fileName).c_str(), data->imgRight);
		}
		else
		{
			cv::imwrite((captureDir + strDirection + std::string("left/") + fileName).c_str(), data->imgLeft);
		}

		++imgNum;
		if (debug) printf("image written. \n");

		data->imgLeft.release();
		data->imgRight.release();
		free(data);
	}
}

void image_handler(const lcm_recv_buf_t* rbuf, const char* channel, const mavconn_mavlink_msg_container_t* container, void* user)
{
	if (recordData && !bPause)
	{
		const mavlink_message_t* msg = getMAVLinkMsgPtr(container);

		// Pointer to shared memory data
		std::vector<px::SHMImageClient>* clientVec = reinterpret_cast< std::vector<px::SHMImageClient>* >(user);

		for (size_t i = 0; i < clientVec->size(); ++i)
		{
			px::SHMImageClient& client = clientVec->at(i);
			if ((client.getCameraConfig() & px::SHMImageClient::getCameraNo(msg)) != px::SHMImageClient::getCameraNo(msg))
			{
				continue;
			}

			writeData *data = new writeData;
			data->pres_alt = 0.f;	//TODO: fix this
			bool success = false;

			//cv::Mat imgLeft, imgRight;
			if (client.readStereoImage(msg, data->imgLeft, data->imgRight))
			{
				success = true;
				data->stereo = true;
				if ((client.getCameraConfig() & px::SHM::CAMERA_FORWARD_LEFT) == px::SHM::CAMERA_FORWARD_LEFT)
				{
					//forward
					data->direction = 1;
				}
				else
				{
					//downward
					data->direction = 0;
				}
			}
			else
			{
				// Try with left mono image if stereo is not available
				// read mono image data
				if (client.readMonoImage(msg, data->imgLeft))
				{
					success = true;
					data->stereo = false;
					if ((client.getCameraConfig() & px::SHM::CAMERA_FORWARD_LEFT) == px::SHM::CAMERA_FORWARD_LEFT)
					{
						//forward
						data->direction = 1;
					}
					else
					{
						//downward
						data->direction = 0;
					}
				}
			}

			if (!success)
			{
				data->imgLeft.release();
				data->imgRight.release();
				delete data;
				continue;
			}

			//cache the meta data
			data->timestamp = client.getTimestamp(msg);

			//always take roll and pitch from compid 200
			client.getRollPitchYaw(msg, data->roll, data->pitch, data->yaw);
			//if imuid is not 200 take yaw from there
			if (imuid != 200)
			{
                            data->roll = backupRoll;
                            data->pitch = backupPitch;
                            data->yaw = backupYaw;
			}
			client.getLocalHeight(msg, data->ground_dist);
			client.getGroundTruth(msg, data->gx, data->gy, data->gz);

			float camlat, camlon, camalt;
			client.getGPS(msg, camlat, camlon, camalt);

			// FIXME should be parametrized differently,
			// add a client->isGPSKnown() function call
//			if (camlat != 0.0 || camlon != 0.0 || camalt != 0.0)
//			{
//				data->lat = camlat;
//				data->lon = camlon;
//				data->alt = camalt;
//				data->hdop = 0.;
//				data->vdop = 0.;
//				data->satcount = 0.;
//			}
//			else
			{
				data->lat = lat;
				data->lon = lon;
				data->alt = alt;
				data->hdop = hdop;
				data->vdop = vdop;
				data->satcount = satcount;
			}

			//store the additional data too...
			data->local_x = local_x;
			data->local_y = local_y;
			data->local_z = local_z;
			data->vx = vx;
			data->vy = vy;
			data->vz = vz;

			data->local_x_gps_raw = local_x_gps_raw;
			data->local_y_gps_raw = local_y_gps_raw;
			data->local_z_gps_raw = local_z_gps_raw;			

			gotFirstImage = true;

			g_async_queue_push(image_write_queue, data);
			if (debug) printf("image pushed. \n");
		}
	}
}

/*
 * @brief Creates data directory based on current date and time, if not existent so far
 */
string createCaptureDirectory( string baseDir, struct tm* timeinfo )
{
	// create current data as string
	// date is used as name for a new folder where the data will be stored
	char dateBuf[80];
	strftime( dateBuf, 80, "%Y%m%d_%H%M%S\0", timeinfo );

	// create directory where to safe the images and data file
	std::string dir = baseDir + string(dateBuf) + "/";
	try
	{
		bfs::create_directories( bfs::path(dir) );
		bfs::create_directories( bfs::path(dir + std::string(DIRECTUION_0_DIR)) );
		bfs::create_directories( bfs::path(dir + std::string(DIRECTUION_1_DIR)) );
		bfs::create_directories( bfs::path(dir + std::string(DIRECTUION_0_DIR) + std::string("left/")) );
		bfs::create_directories( bfs::path(dir + std::string(DIRECTUION_0_DIR) + std::string("right/")) );
		bfs::create_directories( bfs::path(dir + std::string(DIRECTUION_1_DIR) + std::string("left/")) );
		bfs::create_directories( bfs::path(dir + std::string(DIRECTUION_1_DIR) + std::string("right/")) );

		if (calibStrDirection0.length() > 0)
		{
			bfs::create_directories( bfs::path(dir + std::string(DIRECTUION_0_DIR) + std::string("config/")) );
			std::string calibInfoFilePath = dir + std::string(DIRECTUION_0_DIR) + std::string("calibInfo.txt");
			ofstream calibInfoFile(calibInfoFilePath.c_str());
			calibInfoFile << "Original calibration file names:" << endl << calibStrDirection0 << endl << calibStrLeftDirection0 << endl << calibStrRightDirection0;
			calibInfoFile.close();

			//copy the calib files
			std::string syscmd;
			int ret;
			if (calibStrDirection0.compare(calibStrDirection0.length()-5, 4, ".scf"))
			{
				syscmd = (std::string("cp ") + calibStrDirection0 + std::string(" ") + dir + std::string(DIRECTUION_0_DIR) + std::string("config/") + std::string("calib.scf"));
				ret = system(syscmd.c_str());
				if(calibStrLeftDirection0.length() > 0)
				{
					std::string filename = "";
					size_t upto = calibStrLeftDirection0.find_last_of('/');
					if (upto < calibStrLeftDirection0.npos)
					{
						filename = calibStrLeftDirection0.substr(upto+1, calibStrLeftDirection0.length()-1);
					}
					syscmd = (std::string("cp ") + calibStrLeftDirection0 + std::string(" ") + dir + std::string(DIRECTUION_0_DIR) + std::string("config/") + filename);
					ret = system(syscmd.c_str());
				}
				if(calibStrRightDirection0.length() > 0)
				{
					std::string filename = "";
					size_t upto = calibStrRightDirection0.find_last_of('/');
					if (upto < calibStrRightDirection0.npos)
					{
						filename = calibStrRightDirection0.substr(upto+1, calibStrRightDirection0.length()-1);
					}
					syscmd = (std::string("cp ") + calibStrRightDirection0 + std::string(" ") + dir + std::string(DIRECTUION_0_DIR) + std::string("config/") + filename);
					ret = system(syscmd.c_str());
				}
			}
			else
			{
				syscmd = (std::string("cp ") + calibStrDirection0 + std::string(" ") + dir + std::string(DIRECTUION_0_DIR) + std::string("config/") + std::string("calib.cal"));
				ret = system(syscmd.c_str());
			}
		}
		if (calibStrDirection1.length() > 0)
		{
			bfs::create_directories( bfs::path(dir + std::string(DIRECTUION_1_DIR) + std::string("config/")) );
			ofstream calibInfoFile(std::string(dir + std::string(DIRECTUION_1_DIR) + std::string("calibInfo.txt")).c_str());
			calibInfoFile << "Original calibration file names:" << endl << calibStrDirection1 << endl << calibStrLeftDirection1 << endl << calibStrRightDirection1;
			calibInfoFile.close();

			//copy the calib files
			std::string syscmd;
			int ret;
			if (calibStrDirection1.compare(calibStrDirection1.length()-5, 4, ".scf"))
			{
				syscmd = (std::string("cp ") + calibStrDirection1 + std::string(" ") + dir + std::string(DIRECTUION_1_DIR) + std::string("config/") + std::string("calib.scf"));
				ret = system(syscmd.c_str());
				if(calibStrLeftDirection1.length() > 0)
				{
					std::string filename = "";
					size_t upto = calibStrLeftDirection1.find_last_of('/');
					if (upto < calibStrLeftDirection1.npos)
					{
						filename = calibStrLeftDirection1.substr(upto+1, calibStrLeftDirection1.length()-1);
					}
					syscmd = (std::string("cp ") + calibStrLeftDirection1 + std::string(" ") + dir + std::string(DIRECTUION_1_DIR) + std::string("config/") + filename);
					ret = system(syscmd.c_str());
				}
				if(calibStrRightDirection1.length() > 0)
				{
					std::string filename = "";
					size_t upto = calibStrRightDirection1.find_last_of('/');
					if (upto < calibStrRightDirection1.npos)
					{
						filename = calibStrRightDirection1.substr(upto+1, calibStrRightDirection1.length()-1);
					}
					syscmd = (std::string("cp ") + calibStrRightDirection1 + std::string(" ") + dir + std::string(DIRECTUION_1_DIR) + std::string("config/") + filename);
					ret = system(syscmd.c_str());
				}
			}
			else
			{
				syscmd = (std::string("cp ") + calibStrDirection1 + std::string(" ") + dir + std::string(DIRECTUION_1_DIR) + std::string("config/") + std::string("calib.cal"));
				ret = system(syscmd.c_str());
			}
		}
	} catch (...) {
		cout << "Directory " << dir << " for data capture couldn't be created. EXITING!!" << endl;
		exit(EXIT_FAILURE);
	}

	return dir;
}

/*
 * @brief Writes header into capture file
 */
void prepareCaptureFile( ofstream& file, const string& dir, const string& filename, const string& structure, const string& type, struct tm* timeinfo )
{
	cout << "Writing " << type << " capture data into " << dir << filename << endl;
	file.open((dir + filename).c_str(), ios::out);
	file << "# Data capture for camera head on MAVCONN vehicle." << endl;
	char dayBuf[20], timeBuf[20];
	strftime( dayBuf, 20, "%Y-%m-%d\0", timeinfo );
	strftime( timeBuf, 20, "%H:%M\0", timeinfo );
	file << "# Recording date: " << dayBuf << ", " << timeBuf << endl;
	file << "##########" << endl;
	file << "# Structure:" << endl;
	file << "# " << structure << endl;
	file << "##########" << endl << endl;
}

/**
 * @brief Handle incoming MAVLink packets
 */
// Timer for benchmarking
struct timeval tv;

static void mavlink_handler (const lcm_recv_buf_t *rbuf, const char * channel, const mavconn_mavlink_msg_container_t* container, void * user)
{
	lcm_t* lcmMavlink = (lcm_t*) user;
	const mavlink_message_t* mavlink_msg = getMAVLinkMsgPtr(container);

	switch(mavlink_msg->msgid)
	{
		case MAVLINK_MSG_ID_COMMAND_LONG:
		{
			mavlink_command_long_t command;
			mavlink_msg_command_long_decode(mavlink_msg, &command);
            std::cout<<"Received MAVLINK message, command_long, target compares to 42: "<< (command.target_system == sysid)<<", command "<<command.command<<"\n";
			if (command.target_system != sysid && command.target_system != 0) break;
			if (command.command == MAV_CMD_DO_CONTROL_VIDEO)
			{
				if (command.param4 == 1 || command.param4 == 2)
				{
					mavlink_message_t msg;
					mavlink_statustext_t statustext;
					if (!recordData)
					{
						// start recording image data
						if (verbose) printf("Starting recording.\n");

						// preparing data folder and data file
						time_t rawtime;
						time( &rawtime );
						struct tm* timeinfo = localtime(&rawtime);

						//get the home directory of the current user, first check env HOME if the user set a specific directory, otherwise take the systems user home dir
						const char *homedir = getenv("HOME");
						if (!homedir)
						{
							struct passwd *pw = getpwuid(getuid());
							homedir = pw->pw_dir;
						}
						std::string homeDir(homedir + std::string("/"));
						std::string capDir(CAPTURE_DIR);
						captureDir = createCaptureDirectory(homeDir + capDir, timeinfo);
						prepareCaptureFile(imageDataFileDirection0, captureDir + std::string(DIRECTUION_0_DIR) , IMAGE_CAPTURE_FILE,              "timestamp, roll, pitch, yaw, lat, lon, alt, local_z, ground truth X, ground truth Y, ground truth Z", "IMAGE", timeinfo);
						prepareCaptureFile(imageDataFileDirection1, captureDir + std::string(DIRECTUION_1_DIR), IMAGE_CAPTURE_FILE,               "timestamp, roll, pitch, yaw, lat, lon, alt, local_z, ground truth X, ground truth Y, ground truth Z", "IMAGE", timeinfo);
						prepareCaptureFile(plainLogFileDirection0, captureDir + std::string(DIRECTUION_0_DIR), PLAIN_CAPTURE_FILE,                "timestamp, roll, pitch, yaw, lat, lon, alt, pressure_alt, ground_distance, vdop, hdop, satcount, local_x, local_y, local_z, speedx, speedy, speedz, ground truth X, ground truth Y, ground truth Z, ground truth speed X, ground truth speed Y, ground truth speed Z", "IMAGE", timeinfo);
						prepareCaptureFile(plainLogFileDirection1, captureDir + std::string(DIRECTUION_1_DIR), PLAIN_CAPTURE_FILE,                "timestamp, roll, pitch, yaw, lat, lon, alt, pressure_alt, ground_distance, vdop, hdop, satcount, local_x, local_y, local_z, speedx, speedy, speedz, ground truth X, ground truth Y, ground truth Z, ground truth speed X, ground truth speed Y, ground truth speed Z", "IMAGE", timeinfo);
						prepareCaptureFile(plainLogFileMultiDirection0, captureDir + std::string(DIRECTUION_0_DIR), MULTI_COMPONENT_CAPTURE_FILE, "timestamp, roll, pitch, yaw, lat, lon, alt, pressure_alt, ground_distance, vdop, hdop, satcount, local_x_gps_raw, local_y_gps_raw, local_z_gps_raw, local_x_system, local_y_system, local_z_system, speedx_system, speedy_system, speedz_system, ground truth X, ground truth Y, ground truth Z, ground truth speed X, ground truth speed Y, ground truth speed Z", "IMAGE", timeinfo);
						prepareCaptureFile(plainLogFileMultiDirection1, captureDir + std::string(DIRECTUION_1_DIR), MULTI_COMPONENT_CAPTURE_FILE, "timestamp, roll, pitch, yaw, lat, lon, alt, pressure_alt, ground_distance, vdop, hdop, satcount, local_x_gps_raw, local_y_gps_raw, local_z_gps_raw, local_x_system, local_y_system, local_z_system, speedx_system, speedy_system, speedz_system, ground truth X, ground truth Y, ground truth Z, ground truth speed X, ground truth speed Y, ground truth speed Z", "IMAGE", timeinfo);
						char dateBuf[80];
						strftime( dateBuf, 80, "%Y%m%d_%H%M%S\0", timeinfo );
						mavlinkFile.open((captureDir + string(dateBuf) + string(".mavlink")).c_str(), ios::binary | ios::out);

						sprintf((char*)&statustext.text, "MAVCONN: imagecapture: STARTING RECORDING");
						mavlink_msg_statustext_encode(sysid, compid, &msg, &statustext);
						sendMAVLinkMessage(lcmMavlink, &msg);
						snprintf((char*)&statustext.text, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN, "%s%s%s", captureDir.c_str(), dateBuf, ".mavlink");
						mavlink_msg_statustext_encode(sysid, compid, &msg, &statustext);
						sendMAVLinkMessage(lcmMavlink, &msg);
						recordData = true;
						bPause = false;
					}
					else if(bPause)
					{
						bPause = false;
						sprintf((char*)&statustext.text, "MAVCONN: imagecapture: CONTINUE RECORDING");
						mavlink_msg_statustext_encode(sysid, compid, &msg, &statustext);
						sendMAVLinkMessage(lcmMavlink, &msg);
					}
				}
				else if (command.param4 == 0)
				{
					if (recordData)
					{
						// stop recording image data
						if (verbose) printf("Stop recording.\n");

						imageDataFileDirection0 << endl << "### EOF" << endl;
						imageDataFileDirection0.close();
						imageDataFileDirection1 << endl << "### EOF" << endl;
						imageDataFileDirection1.close();
						plainLogFileDirection0 << endl << "### EOF" << endl;
						plainLogFileDirection0.close();
						plainLogFileDirection1 << endl << "### EOF" << endl;
						plainLogFileDirection1.close();
						plainLogFileMultiDirection0 << endl << "### EOF" << endl;
						plainLogFileMultiDirection0.close();
						plainLogFileMultiDirection1 << endl << "### EOF" << endl;
						plainLogFileMultiDirection1.close();
						mavlinkFile.close();

						mavlink_message_t msg;
						mavlink_statustext_t statustext;
						sprintf((char*)&statustext.text, "MAVCONN: imagecapture: STOPPED RECORDING");
						mavlink_msg_statustext_encode(sysid, compid, &msg, &statustext);
						sendMAVLinkMessage(lcmMavlink, &msg);
						recordData = false;
						bPause = false;
					}
				}
			}
		}
		break;
		case MAVLINK_MSG_ID_GPS_RAW_INT:
		{
			mavlink_gps_raw_int_t gps;
			mavlink_msg_gps_raw_int_decode(mavlink_msg, &gps);
			lat = gps.lat/(double)1E7;
			lon = gps.lon/(double)1E7;
			alt = gps.alt/(double)1E3;
			hdop = gps.eph/(double)1E3;
			vdop = gps.epv/(double)1E3;
			satcount = gps.satellites_visible;
		}
		break;
		case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
		{
			if (mavlink_msg->compid == imuid)
			{
				mavlink_local_position_ned_t pos;
				mavlink_msg_local_position_ned_decode(mavlink_msg, &pos);
				local_x = pos.x;
				local_y = pos.y;
				local_z = pos.z;
				vx = pos.vx;
				vy = pos.vy;
				vz = pos.vz;
			}
		}
		break;
			
		case MAVLINK_MSG_ID_ATTITUDE:
		{
			if (mavlink_msg->compid == imuid)
			{
				mavlink_attitude_t att;
				mavlink_msg_attitude_decode(mavlink_msg, &att);
				backupRoll = att.roll;
				backupPitch = att.pitch;
				backupYaw = att.yaw;
			}
		}
		break;

	default:
		break;
	}

	if(recordData && !bPause)
	{
		//write into mavlink logfile
		const int len = MAVLINK_MAX_PACKET_LEN+sizeof(uint64_t);
		uint8_t buf[len];
		uint64_t time = getSystemTimeUsecs();
		memcpy(buf, (void*)&time, sizeof(uint64_t));
		mavlink_msg_to_send_buffer(buf+sizeof(uint64_t), mavlink_msg);
		mavlinkFile.write((char *)buf, len);

		if (mavlink_msg->msgid == MAVLINK_MSG_ID_EXTENDED_MESSAGE)
		{
			mavlinkFile.write(reinterpret_cast<char*>(container->extended_payload),
							  container->extended_payload_len);
		}
	}
}


void* lcm_image_wait(void* lcm_ptr)
{
	lcm_t* lcm = (lcm_t*) lcm_ptr;
	// Blocking wait for new data
	while(1)
	{
		lcm_handle (lcm);
	}
	return NULL;
}

/*
 * @brief Main
 */
int main(int argc, char* argv[])
{
	// ----- Handling Program options
	config::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("sysid,a", config::value<int>(&sysid)->default_value(sysid), "ID of this system, 1-255")
		("compid,c", config::value<int>(&compid)->default_value(compid), "ID of this component")
		("imuid,i", config::value<int>(&imuid)->default_value(imuid), "ID of paired IMU")
		("calib_down", config::value<string>(&calibStrDirection0)->default_value(""), "Path to calibration of the down camera (*.scf for stereo, *.cal for mono)")
		("calib_front", config::value<string>(&calibStrDirection1)->default_value(""), "Path to calibration of the front camera (*.scf for stereo, *.cal for mono)")
		("silent,s", config::bool_switch(&silent)->default_value(false), "suppress outputs")
		("verbose,v", config::bool_switch(&verbose)->default_value(false), "verbose output")
		("debug,d", config::bool_switch(&debug)->default_value(false), "debug output")
		/*("stereo_front", config::bool_switch(&stereo_front)->default_value(false), "record front stereo")*/
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
	lcm_t* lcmImage = lcm_create ("udpm://");
	lcm_t* lcmMavlink = lcm_create ("udpm://");
	if (!lcmImage || !lcmMavlink)
		exit(EXIT_FAILURE);

	std::vector<px::SHMImageClient> clientVec;
	clientVec.resize(4);

	clientVec.at(0).init(true, px::SHM::CAMERA_FORWARD_LEFT);
	clientVec.at(1).init(true, px::SHM::CAMERA_FORWARD_LEFT, px::SHM::CAMERA_FORWARD_RIGHT);
	clientVec.at(2).init(true, px::SHM::CAMERA_DOWNWARD_LEFT);
	clientVec.at(3).init(true, px::SHM::CAMERA_DOWNWARD_LEFT, px::SHM::CAMERA_DOWNWARD_RIGHT);

	mavconn_mavlink_msg_container_t_subscription_t* img_sub = mavconn_mavlink_msg_container_t_subscribe(lcmImage, MAVLINK_IMAGES, &image_handler, &clientVec);
	mavconn_mavlink_msg_container_t_subscription_t * comm_sub = mavconn_mavlink_msg_container_t_subscribe (lcmMavlink, MAVLINK_MAIN, &mavlink_handler, lcmMavlink);

	// ----- Creating thread for image handling
	GThread* lcm_imageThread;
	GThread* lcm_imageWriterThread;
	GError* err;

	// Only initialize g thread if not already done
	if(!g_thread_supported())
	{
		g_thread_init(NULL);
	}

	if (!image_write_queue)
	{
		image_write_queue = g_async_queue_new();
	}

	// thread for IMAGE channel
	if( (lcm_imageThread = g_thread_create((GThreadFunc)lcm_image_wait, (void *)lcmImage, TRUE, &err)) == NULL)
	{
		cout << "Thread create failed: " << err->message << "!!" << endl;
		g_error_free(err);
		exit(EXIT_FAILURE);
	}

	// thread for image_writer
	if( (lcm_imageWriterThread = g_thread_create((GThreadFunc)image_writer, NULL, TRUE, &err)) == NULL)
	{
		cout << "Thread create failed: " << err->message << "!!" << endl;
		g_error_free(err);
		exit(EXIT_FAILURE);
	}


	//check calibration files
	if (calibStrDirection0.length() > 0)
	{
		struct stat st;
		if(stat(calibStrDirection0.c_str(),&st) == 0)
		{
			if (calibStrDirection0.compare(calibStrDirection0.length()-5, 4, ".scf"))
			{
				// load the stereo calibration file
			    std::ifstream fileCalib(calibStrDirection0.c_str(), std::ifstream::in);
			    if (!fileCalib)
			    {
			    	fprintf(stderr, "# WARNING: Unable to open down scf file %s.\n", calibStrDirection0.c_str());
			    	calibStrDirection0 = "";
			    	calib_problems = true;
			    }
			    else
			    {
					std::string relPath = "";
					size_t upto = calibStrDirection0.find_last_of('/');
					if (upto < calibStrDirection0.npos)
					{
						relPath = calibStrDirection0.substr(0, upto+1);
					}

					// read the filename of the left camera calibration file
					std::string filename;
					std::getline(fileCalib, filename);
					calibStrLeftDirection0 = (relPath + filename);
					if(stat(calibStrLeftDirection0.c_str(),&st) != 0)
					{
						fprintf(stderr, "# WARNING: Unable to open left calibration file %s.\n", calibStrLeftDirection0.c_str());
						calibStrLeftDirection0 = "";
						calibStrRightDirection0 = "";
						calib_problems = true;
					}

					// read the filename of the right camera calibration file
					std::getline(fileCalib, filename);
					calibStrRightDirection0 = (relPath + filename);
					if(stat(calibStrRightDirection0.c_str(),&st) != 0)
					{
						fprintf(stderr, "# WARNING: Unable to open right calibration file %s.\n", calibStrRightDirection0.c_str());
						calibStrLeftDirection0 = "";
						calibStrRightDirection0 = "";
						calib_problems = true;
					}

					fileCalib.close();
			    }
			}
			else if (calibStrDirection0.compare(calibStrDirection0.length()-5, 4, ".cal"))
			{
				//in this case we have a very easy life
				calibStrLeftDirection0 = "";
				calibStrRightDirection0 = "";
			}
			else
			{
				calibStrDirection0 = "";
				printf("unrecognized down calibration file format, I'm not logging it, sorry.\n");
				calib_problems = true;
			}
		}
		else
		{
			fprintf(stderr, "# WARNING: Unable to open down scf file %s.\n", calibStrDirection0.c_str());
			calibStrDirection0 = "";
			calib_problems = true;
		}
	}
	else
	{
		printf("No downward calibration file given, I won't do anything about it.\n");
	}

	if (calibStrDirection1.length() > 0)
	{
		struct stat st;
		if(stat(calibStrDirection1.c_str(),&st) == 0)
		{
			if (calibStrDirection1.compare(calibStrDirection1.length()-5, 4, ".scf"))
			{
				// load the stereo calibration file
				std::ifstream fileCalib(calibStrDirection1.c_str(), std::ifstream::in);
				if (!fileCalib)
				{
					fprintf(stderr, "# ERROR: Unable to open front scf file %s.\n", calibStrDirection1.c_str());
					calibStrDirection0 = "";
					calib_problems = true;
				}

				std::string relPath = "";
				size_t upto = calibStrDirection1.find_last_of('/');
				if (upto < calibStrDirection1.npos)
				{
					relPath = calibStrDirection1.substr(0, upto+1);
				}

				// read the filename of the left camera calibration file
				std::string filename;
				std::getline(fileCalib, filename);
				calibStrLeftDirection1 = (relPath + filename);
				if(stat(calibStrLeftDirection1.c_str(),&st) != 0)
				{
					fprintf(stderr, "# WARNING: Unable to open left calibration file %s.\n", calibStrLeftDirection1.c_str());
					calibStrLeftDirection1 = "";
					calibStrRightDirection1 = "";
					calib_problems = true;
				}

				// read the filename of the right camera calibration file
				std::getline(fileCalib, filename);
				calibStrRightDirection1 = (relPath + filename);
				if(stat(calibStrRightDirection1.c_str(),&st) != 0)
				{
					fprintf(stderr, "# WARNING: Unable to open right calibration file %s.\n", calibStrRightDirection1.c_str());
					calibStrLeftDirection1 = "";
					calibStrRightDirection1 = "";
					calib_problems = true;
				}
			}
			else if (calibStrDirection1.compare(calibStrDirection1.length()-5, 4, ".cal"))
			{
				//in this case we have a very easy life
				calibStrLeftDirection1 = "";
				calibStrRightDirection1 = "";
			}
			else
			{
				calibStrDirection1 = "";
				printf("unrecognized front calibration file format, I'm not logging it, sorry.\n");
			}
		}
		else
		{
			fprintf(stderr, "# WARNING: Unable to open front scf file %s.\n", calibStrDirection1.c_str());
			calibStrDirection1 = "";
			calib_problems = true;
		}
	}
	else
	{
		printf("No forward calibration file given, I won't do anything about it.\n");
	}

	mavlink_message_t msg;
	mavlink_statustext_t statustext;

	if(!calib_problems)
	{
		printf("Found all given calibration files correctly.\n");
	}
	else
	{
		sprintf((char*)&statustext.text, "MAVCONN: imagecapture: *** WARNING ***");
		mavlink_msg_statustext_encode(sysid, compid, &msg, &statustext);
		sendMAVLinkMessage(lcmMavlink, &msg);
		sprintf((char*)&statustext.text, "there might be problems with the calib files!");
		mavlink_msg_statustext_encode(sysid, compid, &msg, &statustext);
		sendMAVLinkMessage(lcmMavlink, &msg);
	}

	printf("IMAGE client ready, waiting.\n");
	printf("IMU: %d, SYS: %d, COMP: %d\n", imuid, sysid, compid);

	sprintf((char*)&statustext.text, "MAVCONN: imagecapture: Ready for capture");
	mavlink_msg_statustext_encode(sysid, compid, &msg, &statustext);
	sendMAVLinkMessage(lcmMavlink, &msg);

	signal(SIGINT, signalHandler);

	while (!quit)
	{
		// blocking wait for MAVLINK channel
		lcm_handle(lcmMavlink);
	}

	mavconn_mavlink_msg_container_t_unsubscribe (lcmImage, img_sub);
	mavconn_mavlink_msg_container_t_unsubscribe (lcmMavlink, comm_sub);
	lcm_destroy (lcmImage);
	lcm_destroy (lcmMavlink);

	return EXIT_SUCCESS;
}
