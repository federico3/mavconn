/*======================================================================
========================================================================*/

/**
*   @file
*   @brief a program to manage waypoints and exchange them with the ground station
*
*   @author Benjamin Knecht <bknecht@student.ethz.ch>
*   @author Christian Schluchter <schluchc@ee.ethz.ch>
*   @author Petri Tanskanen <mavteam@student.ethz.ch>
*   @author Alex Trofimov <mavteam@student.ethz.ch>
*/

#include <boost/program_options.hpp>
#include <iostream>
#include <vector>

#include <pixhawk/mavlink.h>

#include "PxVector3.h"

#include "mavconn.h"
#include "core/MAVConnParamClient.h"
#include <glib.h>

//UGLINESS (Federico, fix mismatch between Mavconn and Mavlink by including legacy Mavlink header)
#include "../../../mavlink/build/include/v1.0/legacy/mavlink_msg_set_local_position_setpoint.h"
//END UGLINESS

namespace config = boost::program_options;

lcm_t* lcm;
mavconn_mavlink_msg_container_t_subscription_t* comm_sub;


bool debug;             	///< boolean for debug output or behavior
bool verbose;           	///< boolean for verbose output
bool nosetpointonhold;		///< boolean to stop sending setpoints while in HOLD state
std::string configFile;		///< Configuration file for parameters

//=== struct for storing the current destination ===
typedef struct _mav_destination
{
	uint8_t frame; ///< The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
	float x; //local: x position, global: longitude
	float y; //local: y position, global: latitude
	float z; //local: z position, global: altitude
	float yaw; //Yaw orientation in degrees, [0..360] 0 = NORTH
	float rad; //Radius in which the destination counts as reached
	float holdtime;
} mav_destination;

typedef struct _sweep_parameters
{
	float z; // Height of sweep flight
	float r; // Radius of area on the ground, that is visible while MAV floats at fixed position at height z. Should be large for high altitude and/or broad camera viewing angle.

	float x0; // (x,y)-coordinates of the corner of the sweep area rectangle, where the sweep starts;
	float y0;

	float long_side; // length of a longer side of the rectangle
	float short_side; // length of a shorter side of the rectangle
	float d; // = long_side - 2*r; Not required, but introduced for shorter notation

	float u1; // (u1,v1) is a unit vector that points from corner 0 to corner 2 (direction of long side)
	float v1;
	float u2; // (u2,v2) is a unit vector that points from corner 0 to corner 1 (direction of short side)
	float v2;
} sweep_parameters;

//==== variables for reading from files ===
std::vector<std::string> _image_list;
std::vector<std::string>* image_list = &_image_list;

//==== variables for the planner ====
uint16_t current_active_wp_id = -1;		///< id of current waypoint
uint16_t next_wp_id = -1;				///< id of next waypoint, after current is reached.
bool ready_to_continue = false;			///< this marker is set "true" when all necessary conditions of the current waypoint are fulfilled and wpp is ready to proceed to the next waypoint.
bool valid_destination_available = false;
uint64_t timestamp_lastoutside_orbit = 0;///< timestamp when the MAV was last outside the orbit or had the wrong yaw value
uint64_t timestamp_firstinside_orbit = 0;///< timestamp when the MAV was the first time after a waypoint change inside the orbit and had the correct yaw value
uint64_t timestamp_delay_started = 0;	 ///< timestamp when the current delay command was initiated
bool permission_to_land = false;		 ///< this marker is used for MAV_CMD_NAV_LAND waypoint.
mav_destination cur_dest;				 ///< current flight destination
mavlink_local_position_ned_t last_known_pos; ///< latest received position of MAV
mavlink_attitude_t last_known_att;		 ///< latest received attitude of MAV

std::vector<mavlink_mission_item_t*> waypoints1;	///< vector1 that holds the waypoints
std::vector<mavlink_mission_item_t*> waypoints2;	///< vector2 that holds the waypoints

std::vector<mavlink_mission_item_t*>* waypoints = &waypoints1;					///< pointer to the currently active waypoint vector
std::vector<mavlink_mission_item_t*>* waypoints_receive_buffer = &waypoints2;	///< pointer to the receive buffer waypoint vector

//==== variables for the search thread ====
pid_t patternrec_pid = -1; ///< process id of patternrec. -1 until initialized by fork()
mavlink_local_position_ned_t search_success_pos; ///< position of MAV when it succeeded in search
mavlink_attitude_t search_success_att;		 ///< attitude of MAV when it succeeded in search
mavlink_pattern_detected_t last_detected_pattern; ///< latest successful pattern detection
float min_conf = 0;                        ///< minimum confidence for pattern to be detected successfully.

//==== variables for the sweep thread ====
mavlink_mission_item_t* next_sweep_wp = NULL;

//==== Thread and mutex declarations ====
GError *error = NULL;
GThread* waypoint_lcm_thread = NULL;

static GMutex* main_mutex = NULL;
static GCond* cond_position_received = NULL;
static GCond* cond_pattern_detected = NULL;

GThread* search_thread = NULL;
GThread* sweep_thread = NULL;
bool terminate_threads = false;

//==== variables needed for communication protocol ====
uint8_t systemid = getSystemID();          		///< indicates the ID of the system
uint8_t compid = MAV_COMP_ID_MISSIONPLANNER;	///< indicates the component ID of the waypointplanner

MAVConnParamClient* paramClient;

enum PX_WAYPOINTPLANNER_STATES
{
	PX_WPP_IDLE = 0,
	PX_WPP_RUNNING,
	PX_WPP_ON_HOLD
};

enum PX_WAYPOINTPLANNER_COMMUNICATION_STATES
{
    PX_WPP_COMM_IDLE = 0,
    PX_WPP_COMM_SENDLIST,
    PX_WPP_COMM_SENDLIST_SENDWPS,
    PX_WPP_COMM_GETLIST,
    PX_WPP_COMM_GETLIST_GETWPS
};

enum PX_WAYPOINTPLANNER_SEARCH_STATES
{
	PX_WPP_SEARCH_IDLE = 0, // No search threads are running
	PX_WPP_SEARCH_RUNNING,	// The search thread is running, but required number of detections not yet reached
	PX_WPP_SEARCH_SUCCESS,	// The search thread is running and has been successful
	PX_WPP_SEARCH_RESET,	// Resetting the number of detections to 0 and continue search
	PX_WPP_SEARCH_END		// MAV_CMD_DO_FINISH_SEARCH has no jumps left and the search thread is about to be terminated
};

enum PX_WAYPOINTPLANNER_SWEEP_STATES
{
	PX_WPP_SWEEP_IDLE = 0,
	PX_WPP_SWEEP_RUNNING,
	PX_WPP_SWEEP_FINISHED
};
/*
enum PX_WAYPOINT_CMD_ID
{
	//These are unofficial waypoint types, defined specially for PixHawk project
	MAV_CMD_DO_START_SEARCH = 237,
	MAV_CMD_DO_FINISH_SEARCH = 238,
	MAV_CMD_DO_SEND_MESSAGE = 239,
	//MAV_CMD_DO_SWEEP = 82
};
*/

PX_WAYPOINTPLANNER_STATES wpp_state = PX_WPP_IDLE;
PX_WAYPOINTPLANNER_COMMUNICATION_STATES comm_state = PX_WPP_COMM_IDLE;
PX_WAYPOINTPLANNER_SEARCH_STATES search_state = PX_WPP_SEARCH_IDLE;
PX_WAYPOINTPLANNER_SWEEP_STATES sweep_state = PX_WPP_SWEEP_IDLE;
uint16_t protocol_current_wp_id = 0;
uint16_t protocol_current_count = 0;
uint8_t protocol_current_partner_systemid = 0;
uint8_t protocol_current_partner_compid = 0;

uint64_t protocol_timestamp_lastaction = 0;
uint64_t timestamp_last_send_setpoint = 0;
uint64_t timestamp_last_handle_mission = 0;


uint16_t load_mission_from_file(std::string waypointfile)
{
        std::ifstream wpfile;
        wpfile.open(waypointfile.c_str());
        if (!wpfile)
        {
            printf("Unable to open waypoint file\n");
            return 1; // terminate with error
        }
        if (!wpfile.eof())
        {
            std::string check;
			int ver;
			bool good = false;
			wpfile >> check;
			if (!strcmp(check.c_str(),"QGC"))
			{
				wpfile >> check;
				if (!strcmp(check.c_str(),"WPL"))
				{
					wpfile >> ver;

					char c = (char)wpfile.peek();
					if(c == '\r' || c == '\n')
					{
						good = true; //the structure of the first line in the file is correct: QGC WPL ###
					}
				}
			}

            if (!good)
            {
            	printf("Invalid waypoint file\n");
            	return 1;
            }
            if (verbose) printf("Version: %u\n", ver);
            switch (ver)
            {
            case 120:
             {
             	printf("Loading waypoint file...\n");
             	while (!wpfile.eof())
 	            {
 	                mavlink_mission_item_t *wp = new mavlink_mission_item_t();

 	                uint16_t temp;

 	                wpfile >> wp->seq; //waypoint id
 	                wpfile >> temp; wp->current = temp;
 	                wpfile >> temp; wp->frame = temp;
 	                wpfile >> temp; wp->command = temp;
 	                wpfile >> wp->param1;
 	                wpfile >> wp->param2;
 	                wpfile >> wp->param3; //old "orbit"
 	                wpfile >> wp->param4; //old "yaw"
 	                wpfile >> wp->x;
 	                wpfile >> wp->y;
 	                wpfile >> wp->z;
 	                wpfile >> temp; wp->autocontinue = temp;

 	                char c = (char)wpfile.peek();
 	                if(c != '\r' && c != '\n')
 	                {
 	                    delete wp;
 	                    break;
 	                }

 	                printf("WP %3u%s: Frame: %u\tCommand: %3u\tparam1: %6.2f\tparam2: %7.2f\tparam3: %6.2f\tparam4: %7.2f\tX: %7.2f\tY: %7.2f\tZ: %7.2f\tAuto-Cont: %u\t\n", wp->seq, (wp->current?"*":" "), wp->frame, wp->command, wp->param1, wp->param2, wp->param3, wp->param4, wp->x, wp->y, wp->z, wp->autocontinue);
 	                waypoints->push_back(wp);
 	            } //end while
               	break;
             }
             default:
             {
                 printf("Waypoint file version %u is not supported!\n",ver);
                 return 1; // terminate with error
             }

            } //end version switch
        } // end reading file
        else
        {
            printf("Empty waypoint file!\n");
            //return 1; // terminate with error
        }
        wpfile.close();
        return 0;
}

uint16_t load_imagelist_from_file(std::string imagelistfile)
{
std::ifstream ilfile;
ilfile.open(imagelistfile.c_str());
if (!ilfile) {
    printf("Unable to open image list file\n");
    return 1; // terminate with error
}

if (!ilfile.eof())
{
    printf("Loading image list file...\n");
    std::string image_path;
    while (!ilfile.eof())
    {
    	getline(ilfile,image_path);
    	if (image_path.size() > 0)
    	{
    		if(verbose) printf("%s\n", image_path.c_str());
    		image_list->push_back(image_path);
    	}
    	else
    	{
    		break;
    	}
    }
}
else
{
    printf("Empty image list file!\n");
}
ilfile.close();
//Test if filenames have been saved correctly
//printf("Number of lines: %u\n", image_list->size());
//std::string temp;
//temp = image_list->at(1);
//printf("Second line: %s\n", temp.c_str());
return 0;
}

void handle_mission (uint16_t seq, uint64_t now);

void send_mission_ack(uint8_t target_systemid, uint8_t target_compid, uint8_t type)
/*
*  @brief Sends an waypoint ack message
*/
{
    mavlink_message_t msg;
    mavlink_mission_ack_t wpa;

    wpa.target_system = target_systemid;
    wpa.target_component = target_compid;
    wpa.type = type;

    mavlink_msg_mission_ack_encode(systemid, compid, &msg, &wpa);
   sendMAVLinkMessage(lcm, &msg);

    usleep(paramClient->getParamValue("PROTDELAY"));

    if (verbose) printf("Sent waypoint ack (%u) to ID %u\n", wpa.type, wpa.target_system);
}

void send_command_ack(uint16_t cmd_id, uint8_t result)
/*
*  @brief Sends a command ack message
*
*  @param feedback some float value. The interpretation depends on the command
*  @param result 0: Action ACCEPTED and EXECUTED, 1: Action TEMPORARY REJECTED/DENIED, 2: Action PERMANENTLY DENIED, 3: Action UNKNOWN/UNSUPPORTED, 4: Requesting CONFIRMATION
*/
{
	mavlink_message_t msg;
    mavlink_command_ack_t cmda;

    cmda.command = (float) cmd_id;
    cmda.result = (float) result;

    mavlink_msg_command_ack_encode(systemid, compid, &msg, &cmda);
   sendMAVLinkMessage(lcm, &msg);
    if (verbose) printf("Sent ack to command(%u) with code %u\n", cmd_id, result);

    usleep(paramClient->getParamValue("PROTDELAY"));
}

void send_mission_current(uint16_t seq)
/*
*  @brief Broadcasts the new target waypoint and directs the MAV to fly there
*
*  This function broadcasts its new active waypoint sequence number
*
*  @param seq The sequence number of currently active waypoint.
*/
{
    if(seq < waypoints->size())
    {
        mavlink_mission_item_t *cur = waypoints->at(seq);

        mavlink_message_t msg;
        mavlink_mission_current_t wpc;

        wpc.seq = cur->seq;

        mavlink_msg_mission_current_encode(systemid, compid, &msg, &wpc);
       sendMAVLinkMessage(lcm, &msg);

        usleep(paramClient->getParamValue("PROTDELAY"));

        if (verbose) printf("Broadcasted new current waypoint %u\n", wpc.seq);
    }
    else
    {
        if (verbose) printf("ERROR: index out of bounds 1\n");
    }
}

void send_setpoint(void)
/*
*  @brief Directs the MAV to fly to a position
*
*  Sends a message to the controller, advising it to fly to the coordinates
*  of the waypoint with a given orientation
*
*  @param seq The waypoint sequence number the MAV should fly to.
*/
{
	if(valid_destination_available==true)
	{
        mavlink_message_t msg;
        mavlink_set_local_position_setpoint_t PControlSetPoint;

        // send new set point to local IMU
        if (cur_dest.frame ==  MAV_FRAME_LOCAL_NED)
        {
            PControlSetPoint.target_system = systemid;
            PControlSetPoint.target_component = MAV_COMP_ID_IMU;
            PControlSetPoint.x = cur_dest.x;
            PControlSetPoint.y = cur_dest.y;
            PControlSetPoint.z = cur_dest.z;
            PControlSetPoint.yaw = cur_dest.yaw;

            mavlink_msg_set_local_position_setpoint_encode(systemid, compid, &msg, &PControlSetPoint);
           sendMAVLinkMessage(lcm, &msg);

            if (verbose) printf("Send setpoint: x: %.2f | y: %.2f | z: %.2f | yaw: %.3f\n", cur_dest.x, cur_dest.y, cur_dest.z, cur_dest.yaw);
            usleep(paramClient->getParamValue("PROTDELAY"));
        }
        else
        {
            if (verbose) printf("No new set point sent to IMU because the new waypoint had no local coordinates\n");
        }

        struct timeval tv;
        gettimeofday(&tv, NULL);
        uint64_t now = ((uint64_t)tv.tv_sec)*1000000 + tv.tv_usec;
        timestamp_last_send_setpoint = now;
	}
	else
	{
		 if (debug) printf("No new set point sent, because no valid destination available yet\n");
	}
}

void send_mission_count(uint8_t target_systemid, uint8_t target_compid, uint16_t count)
{
    mavlink_message_t msg;
    mavlink_mission_count_t wpc;

    wpc.target_system = target_systemid;
    wpc.target_component = target_compid;
    wpc.count = count;

    mavlink_msg_mission_count_encode(systemid, compid, &msg, &wpc);
    sendMAVLinkMessage(lcm, &msg);

    if (verbose) printf("Sent waypoint count (%u) to ID %u\n", wpc.count, wpc.target_system);

    usleep(paramClient->getParamValue("PROTDELAY"));
}

void send_mission(uint8_t target_systemid, uint8_t target_compid, uint16_t seq)
{
	if (seq < waypoints->size())
	{
		mavlink_message_t msg;
		mavlink_mission_item_t *wp = waypoints->at(seq);
		wp->target_system = target_systemid;
		wp->target_component = target_compid;
		mavlink_msg_mission_item_encode(systemid, compid, &msg, wp);
		sendMAVLinkMessage(lcm, &msg);
		if (verbose) printf("Sent waypoint %u to ID %u\n", wp->seq, wp->target_system);

		usleep(paramClient->getParamValue("PROTDELAY"));
	}
	else
	{
		if (verbose) printf("ERROR: index out of bounds 2\n");
	}
}

void send_mission_request(uint8_t target_systemid, uint8_t target_compid, uint16_t seq)
{
	if (seq < protocol_current_count) //changed this from if(seq < waypoints->size())
	{
		mavlink_message_t msg;
        mavlink_mission_request_t wpr;
        wpr.target_system = target_systemid;
        wpr.target_component = target_compid;
        wpr.seq = seq;
        mavlink_msg_mission_request_encode(systemid, compid, &msg, &wpr);
       sendMAVLinkMessage(lcm, &msg);
        if (verbose) printf("Sent waypoint request %u to ID %u\n", wpr.seq, wpr.target_system);

        usleep(paramClient->getParamValue("PROTDELAY"));
    }

    else
    {
        if (verbose) printf("ERROR: index out of bounds. seq = %u ,size = %u\n",seq,(uint16_t) waypoints->size());
    }
}

void send_mission_reached(uint16_t seq)
/*
*  @brief emits a message that a waypoint reached
*
*  This function broadcasts a message that a waypoint is reached.
*
*  @param seq The waypoint sequence number the MAV has reached.
*/
{
    mavlink_message_t msg;
    mavlink_mission_item_reached_t wp_reached;

    wp_reached.seq = seq;

    mavlink_msg_mission_item_reached_encode(systemid, compid, &msg, &wp_reached);
   sendMAVLinkMessage(lcm, &msg);

    if (verbose) printf("Sent waypoint %u reached message\n", wp_reached.seq);

    usleep(paramClient->getParamValue("PROTDELAY"));
}

void set_destination(mavlink_mission_item_t* wp)
{
	// Assume MAV_CMD_NAV_WAYPOINT and set parameters
	valid_destination_available = true;
	cur_dest.frame = wp->frame;
	cur_dest.x = wp->x;
	cur_dest.y = wp->y;
	cur_dest.z = wp->z;
	cur_dest.yaw = wp->param4;
	cur_dest.rad = wp->param2;
	cur_dest.holdtime = wp->param1;

	if(wp->command != MAV_CMD_NAV_WAYPOINT)
	{
		if (verbose) printf("Warning: New destination coordinates do not origin from MAV_CMD_NAV_WAYPOINT waypoint.\n");
	}

}

mavlink_mission_item_t* get_wp_of_current_position ()
{
	mavlink_mission_item_t* wp = new mavlink_mission_item_t;
	wp->autocontinue = true;
	wp->command = MAV_CMD_NAV_WAYPOINT;
	wp->current = false;
	wp->frame = cur_dest.frame;
	wp->param1 = 0;
	wp->param2 = 0;
	wp->param3 = 0;
	wp->param4 = last_known_att.yaw;
	wp->x = last_known_pos.x;
	wp->y = last_known_pos.y;
	wp->z = last_known_pos.z;
	wp->seq = 0;
	return wp;
}

float distanceToSegment(float x, float y, float z , uint16_t next_NAV_wp_id)
{
    	const PxVector3 A(cur_dest.x, cur_dest.y, cur_dest.z);
        const PxVector3 C(x, y, z);

        // next_NAV_wp_id not the second last waypoint
        if ((uint16_t)(next_NAV_wp_id) < waypoints->size())
        {
            mavlink_mission_item_t *next = waypoints->at(next_NAV_wp_id);
            const PxVector3 B(next->x, next->y, next->z);
            const float r = (B-A).dot(C-A) / (B-A).lengthSquared();
            if (r >= 0 && r <= 1)
            {
                const PxVector3 P(A + r*(B-A));
                return (P-C).length();
            }
            else if (r < 0.f || next->command != MAV_CMD_NAV_WAYPOINT)
            {
                return (C-A).length();
            }
            else
            {
                return (C-B).length();
            }
        }
        else
        {

            return (C-A).length();

        }
}

float distanceToPoint(float x, float y, float z)
{
	const PxVector3 A(cur_dest.x, cur_dest.y, cur_dest.z);
    const PxVector3 C(x, y, z);

    return (C-A).length();
}

void check_if_reached_dest(bool* posReached, bool* yawReached, uint16_t next_wp_id)
{
	float dist;
	if (cur_dest.holdtime == 0 && next_wp_id < waypoints->size() && waypoints->at(next_wp_id)->command == MAV_CMD_NAV_WAYPOINT)
	{
		//if (debug) printf("Both current and next waypoint (%u) are MAV_CMD_NAV_WAYPOINT. Using distanceToSegment.\n", next_wp_id);
	    dist = distanceToSegment(last_known_pos.x, last_known_pos.y, last_known_pos.z, next_wp_id);
	}
	else
	{
	    dist = distanceToPoint(last_known_pos.x, last_known_pos.y, last_known_pos.z);
	}

	if (dist >= 0.f && dist <= cur_dest.rad)
	{
		//if (debug) printf("Check dest_reach: dist = %f, rad = %f\n",dist,cur_dest.rad);
	    *posReached = true;
	}

	// yaw reached?
	float yaw_tolerance = paramClient->getParamValue("YAWTOLERANCE");
	//compare last known yaw with current desired yaw
	if (last_known_att.yaw - yaw_tolerance >= 0.0f && last_known_att.yaw + yaw_tolerance < 2.f*M_PI)
	{
	    if (last_known_att.yaw - yaw_tolerance <= cur_dest.yaw*M_PI/180 && last_known_att.yaw + yaw_tolerance >= cur_dest.yaw*M_PI/180)
	        *yawReached = true;
	}
	else if(last_known_att.yaw - yaw_tolerance < 0.0f)
	{
	    float lowerBound = 2.f*M_PI + last_known_att.yaw - yaw_tolerance;
	    if (lowerBound < cur_dest.yaw*M_PI/180 || cur_dest.yaw*M_PI/180 < last_known_att.yaw + yaw_tolerance)
	        *yawReached = true;
	}
	else
	{
	    float upperBound = last_known_att.yaw + yaw_tolerance - 2.f*M_PI;
	    if (last_known_att.yaw - yaw_tolerance < cur_dest.yaw*M_PI/180 || cur_dest.yaw*M_PI/180 < upperBound)
	        *yawReached = true;
	}
}

bool above_landing(mavlink_mission_item_t* land_wp)
{
	if ((last_known_pos.x - land_wp->x)*(last_known_pos.x - land_wp->x) + (last_known_pos.y - land_wp->y)*(last_known_pos.y - land_wp->y) <= land_wp->param2)
	{
		return true;
	}
	else
	{
		return false;
	}
}

uint16_t calculate_sweep_parameters (const mavlink_mission_item_t* sweep_wp, mavlink_local_position_ned_t cur_pos, sweep_parameters* sw)
{

	float corner[4][2];
	// Copy data from waypoint
	corner[0][0] = sweep_wp->param3;
	corner[0][1] = sweep_wp->param4;
	corner[3][0] = sweep_wp->x;
	corner[3][1] = sweep_wp->y;
	sw->z = sweep_wp->z;
	sw->r = sweep_wp->param1;

	//sanity check
	if (fabs(corner[0][0] - corner[3][0]) < 2*sw->r || fabs(corner[0][1] - corner[3][1]) < 2*sw->r || sw->r<=0)
	{
		printf("Error: Invalid sweep parameters.\n");
		return -1;
	}
	// Calculate two other corners
	corner[1][0] = corner[3][0];
	corner[1][1] = corner[0][1];
	corner[2][0] = corner[0][0];
	corner[2][1] = corner[3][1];

	// In the implementation above, the corners are chosen such that sides of the rectangle are parallel to the x and y axes.
	// One additional parameter is needed to overcome this constraint (i.e. allow arbitrary orientation of rectangle).
	// The following works for any rectangle:

	uint8_t i,j;
	float d[4];
	uint8_t i_min;
	float d_temp;
	float corner_temp[2];


	for (i=0;i<=3;i++)
	{
		d[i]=sqrt((cur_pos.x - corner[i][0])*(cur_pos.x - corner[i][0]) + (cur_pos.y - corner[i][1])*(cur_pos.y - corner[i][1]));
			if (d[i]<d[i_min])
			{
				i_min = i;
			}
	}

	for (i=0;i<=3;i++)
	{
		d[i] = sqrt((corner[i][0] - corner[i_min][0])*(corner[i][0] - corner[i_min][0]) + (corner[i][1] - corner[i_min][1])*(corner[i][1] - corner[i_min][1]));
	}

	// Sort the points such that: Point 0 is the nearest to MAV, i.e. starting point. Connection 0-3 is a diagonal, 0-2 is a longer side and 0-1 is a shorter side of the rectangle
for (i=3;i>0;i--)
{
	for (j=0;j<i;j++)
	{
		if (d[j]>d[j+1])
		{
			d_temp = d[j];
			d[j] = d[j+1];
			d[j+1] = d_temp;

			corner_temp[0] = corner[j][0];
			corner_temp[1] = corner[j][1];
			corner[j][0] = corner[j+1][0];
			corner[j][1] = corner[j+1][1];
			corner[j+1][0] = corner_temp[0];
			corner[j+1][1] = corner_temp[1];
		}
	}
}
	sw->x0 = corner[0][0];
	sw->y0 = corner[0][1];

	sw->long_side = sqrt((corner[2][0] - corner[0][0])*(corner[2][0] - corner[0][0]) + (corner[2][1] - corner[0][1])*(corner[2][1] - corner[0][1]));
	sw->short_side = sqrt((corner[1][0] - corner[0][0])*(corner[1][0] - corner[0][0]) + (corner[1][1] - corner[0][1])*(corner[1][1] - corner[0][1]));
	sw->d = sw->long_side - 2*sw->r;

	sw->u1 = (corner[2][0] - corner[0][0])/sw->long_side;
	sw->v1 = (corner[2][1] - corner[0][1])/sw->long_side;
	sw->u2 = (corner[1][0] - corner[0][0])/sw->short_side;
	sw->v2 = (corner[1][1] - corner[0][1])/sw->short_side;

	/*
	printf("Sweep parameters calculated:\n");
	printf("x0: %f  y0: %f\n", corner[0][0],corner[0][1]);
	printf("x1: %f  y1: %f\n", corner[1][0],corner[1][1]);
	printf("x2: %f  y2: %f\n", corner[2][0],corner[2][1]);
	printf("x3: %f  y3: %f\n", corner[3][0],corner[3][1]);

	printf("\nu1: %f  v1: %f \n", sw->u1,sw->v1);
	printf("u2: %f  v2: %f \n", sw->u2,sw->v2);
	*/
	printf("\nlong: %f  short: %f\n", sw->long_side, sw->short_side);
	return 0;
}





/*
void terminate_all_threads() //this fuction should be called every time when a new waypoint list is downloaded, to end threads from the old list.
{
	if (verbose) printf("Kill all threads!\n");

	terminate_threads = true;

	// FIX ME!! For some reason g_cond_broadcast is ignored and the thread does not wake up for termination.

	if (verbose) printf("Waking the search thread for termination...\n");
	g_cond_broadcast(cond_pattern_detected); //fake condition broadcast in order to wake the search thread for termination
	if (verbose) printf("Waking the sweep thread for termination...\n");
	g_cond_broadcast(cond_position_received); //fake condition broadcast in order to wake the sweep thread for termination
	terminate_threads = false;
	if (verbose) printf("All threads killed!\n");
}
*/

void* search_thread_func (gpointer n_det)
{
	//Manage the output of patternrec
	uint16_t npic = 0;	                         ///< number of times the picture has been detected
	float best_conf = 0;
	search_state = PX_WPP_SEARCH_RUNNING;
	int16_t* n_det_ = (int16_t*) n_det; //specifies the number of detections needed for success of the search
	int16_t detections_needed = *n_det_;
	if (verbose) printf("here %u.\n", detections_needed);
	while (1)
	{
		if (verbose) printf("here 2\n");
		g_cond_wait(cond_pattern_detected,main_mutex);
		if (verbose) printf("here 3\n");
		if (terminate_threads == true || search_state == PX_WPP_SEARCH_END)
		{
			search_state = PX_WPP_SEARCH_IDLE;
			g_mutex_unlock(main_mutex);
			return NULL;
		}
		else if (search_state == PX_WPP_SEARCH_RESET)
		{
			npic = 0;
			search_state = PX_WPP_SEARCH_RUNNING;
		}

		if (wpp_state == PX_WPP_RUNNING) //do not count pattern recognitions while on hold.
		{
			npic++;
			if (last_detected_pattern.confidence >= best_conf)
			{
				best_conf = last_detected_pattern.confidence;
				search_success_pos = last_known_pos;
				search_success_att = last_known_att;
			}

			if (debug) printf("npic = %u. det_need = %u\n",npic,detections_needed);

			if (npic >= detections_needed && search_state == PX_WPP_SEARCH_RUNNING)
			{
				if (verbose) printf("Search successful! Best detection so far happened at position (%.2f,%.2f) with confidence %f\n",search_success_pos.x,search_success_pos.y, best_conf);
				search_state = PX_WPP_SEARCH_SUCCESS;
			}
		}
	}
	return NULL;
}

void* sweep_thread_func (gpointer sweep_wp)
{
	sweep_state = PX_WPP_SWEEP_RUNNING;

	struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t now = ((uint64_t)tv.tv_sec)*1000000 + tv.tv_usec;

	mavlink_mission_item_t* sweep_wp_ = (mavlink_mission_item_t*) sweep_wp; // Sweep waypoint with all the necessary data.
	sweep_parameters sw;

	if(!calculate_sweep_parameters (sweep_wp_, last_known_pos, &sw))
	{
		bool yawReached=false;						// boolean for yaw attitude reached
		bool posReached=false;						// boolean for position reached
		uint16_t fake_next_wp_id = sweep_wp_->seq;  // defined for the sake of "check_if_reached_dest"-function
		next_sweep_wp = new mavlink_mission_item_t;
		next_sweep_wp->command = MAV_CMD_NAV_WAYPOINT; // Must be declared
		next_sweep_wp->frame = MAV_FRAME_LOCAL_NED;
		next_sweep_wp->param1 = 0.5; // MAV should stay 0.5s at each checkpoint within the sweep
		next_sweep_wp->param2 = 0.15; // acceptance radius may depend on sw.r, e.g. 0.2*sw.r
		next_sweep_wp->param4 = 0; // Should yaw stay constant all the time?
		next_sweep_wp->z = sw.z;

		uint16_t sweep_line = 0; // sweep line count, starting at zero.

		// consecutive checkpoints
		while (sw.short_side > (1+2*sweep_line)*sw.r)
		{
			if (verbose) printf("Sweep: proceeding to line %u\n", sweep_line);

			// (sweep_line*2)-th chechpoint
			if (verbose) printf("Sweep: next checkpoint: %u\n", sweep_line*2);
	    	next_sweep_wp->x = sw.x0 + (sw.r + (sweep_line % 2)*sw.d)*sw.u1 + (1+2*sweep_line)*sw.r*sw.u2;
	    	next_sweep_wp->y = sw.y0 + (sw.r + (sweep_line % 2)*sw.d)*sw.v1 + (1+2*sweep_line)*sw.r*sw.v2;
	    	now = ((uint64_t)tv.tv_sec)*1000000 + tv.tv_usec;
	    	handle_mission(current_active_wp_id,now);
	    	yawReached = false;						///< boolean for yaw attitude reached
	    	posReached = false;						///< boolean for position reached
	    	while (posReached==false || yawReached==false || wpp_state != PX_WPP_RUNNING)
	    	{
	    		//if (verbose) printf("...sweep thread going to wait (wp: %u)\n", sweep_line*2);
	    		g_cond_wait(cond_position_received,main_mutex);
	    		//if (verbose) printf("...sweep thread active\n");
				if (current_active_wp_id != sweep_wp_->seq || terminate_threads == true) //terminate thread if current waypoint changed
				{
					sweep_state = PX_WPP_SWEEP_IDLE;
					if (verbose && current_active_wp_id != sweep_wp_->seq) printf("Sweep: failed. Current waypoint changed. Thread terminated.\n");
					if (verbose && terminate_threads == true) printf("Sweep: Thread terminated.\n");
					g_mutex_unlock(main_mutex);
					return NULL;
				}

		    	yawReached = false;						///< boolean for yaw attitude reached
		    	posReached = false;						///< boolean for position reached
		    	check_if_reached_dest(&posReached, &yawReached, fake_next_wp_id);
	    	}

	    	// (sweep_line*2 + 1)-th chechpoint

	    	if (verbose) printf("Sweep: next checkpoint: %u\n", sweep_line*2+1);
	    	next_sweep_wp->x = sw.x0 + (sw.r + ((sweep_line+1) % 2)*sw.d)*sw.u1 + (1+2*sweep_line)*sw.r*sw.u2;
	    	next_sweep_wp->y = sw.y0 + (sw.r + ((sweep_line+1) % 2)*sw.d)*sw.v1 + (1+2*sweep_line)*sw.r*sw.v2;
	    	now = ((uint64_t)tv.tv_sec)*1000000 + tv.tv_usec;
	    	handle_mission(current_active_wp_id,now);
	    	yawReached = false;						///< boolean for yaw attitude reached
	    	posReached = false;						///< boolean for position reached
	    	while (posReached==false || yawReached==false || wpp_state != PX_WPP_RUNNING)
	    	{
	    		//if (verbose) printf("...sweep thread going to wait (wp: %u)\n", sweep_line*2+1);
	    		g_cond_wait(cond_position_received,main_mutex);
	    		//if (verbose) printf("...sweep thread active\n");
				if (current_active_wp_id != sweep_wp_->seq || terminate_threads == true)
				{
					sweep_state = PX_WPP_SWEEP_IDLE;
					if (verbose && current_active_wp_id != sweep_wp_->seq) printf("Sweep: failed. Current waypoint changed. Thread terminated.\n");
					if (verbose && terminate_threads == true) printf("Sweep: Thread terminated.\n");
					g_mutex_unlock(main_mutex);
					return NULL;
				}

		    	yawReached = false;						///< boolean for yaw attitude reached
		    	posReached = false;						///< boolean for position reached
		    	check_if_reached_dest(&posReached, &yawReached, fake_next_wp_id);
	    	}
	    	sweep_line++;
		} // end while

		if (sw.short_side > 2*sweep_line*sw.r) //The last sweep line was partially covered by the previous one, but should be added for complete cover of sweep area
		{

			if (verbose) printf("Sweep: proceeding to the last sweep line %u\n", sweep_line);

			// (sweep_line*2)-th chechpoint
			if (verbose) printf("Sweep: next checkpoint: %u\n", sweep_line*2);
	    	next_sweep_wp->x = sw.x0 + (sw.r + (sweep_line % 2)*sw.d)*sw.u1 + (sw.short_side - sw.r)*sw.u2;
	    	next_sweep_wp->y = sw.y0 + (sw.r + (sweep_line % 2)*sw.d)*sw.v1 + (sw.short_side - sw.r)*sw.r*sw.v2;
	    	now = ((uint64_t)tv.tv_sec)*1000000 + tv.tv_usec;
	    	handle_mission(current_active_wp_id,now);
	    	yawReached = false;						///< boolean for yaw attitude reached
	    	posReached = false;						///< boolean for position reached
	    	while (posReached==false || yawReached==false || wpp_state != PX_WPP_RUNNING)
	    	{
	    		//if (verbose) printf("...sweep thread going to wait (wp: %u)\n", sweep_line*2);
	    		g_cond_wait(cond_position_received,main_mutex);
	    		//if (verbose) printf("...sweep thread active\n");
				if (current_active_wp_id != sweep_wp_->seq || terminate_threads == true) //terminate thread if current waypoint changed
				{
					sweep_state = PX_WPP_SWEEP_IDLE;
					if (verbose && current_active_wp_id != sweep_wp_->seq) printf("Sweep: failed. Current waypoint changed. Thread terminated.\n");
					if (verbose && terminate_threads == true) printf("Sweep: Thread terminated.\n");
					g_mutex_unlock(main_mutex);
					return NULL;
				}

		    	yawReached = false;						///< boolean for yaw attitude reached
		    	posReached = false;						///< boolean for position reached
		    	check_if_reached_dest(&posReached, &yawReached, fake_next_wp_id);
	    	}

	    	// (sweep_line*2 + 1)-th chechpoint

	    	if (verbose) printf("Sweep: next checkpoint: %u\n", sweep_line*2+1);
	    	next_sweep_wp->x = sw.x0 + (sw.r + ((sweep_line+1) % 2)*sw.d)*sw.u1 + (sw.short_side - sw.r)*sw.u2;
	    	next_sweep_wp->y = sw.y0 + (sw.r + ((sweep_line+1) % 2)*sw.d)*sw.v1 + (sw.short_side - sw.r)*sw.v2;
	    	now = ((uint64_t)tv.tv_sec)*1000000 + tv.tv_usec;
	    	handle_mission(current_active_wp_id,now);
	    	yawReached = false;						///< boolean for yaw attitude reached
	    	posReached = false;						///< boolean for position reached
	    	while (posReached==false || yawReached==false || wpp_state != PX_WPP_RUNNING)
	    	{
	    		//if (verbose) printf("...sweep thread going to wait (wp: %u)\n", sweep_line*2+1);
	    		g_cond_wait(cond_position_received,main_mutex);
	    		//if (verbose) printf("...sweep thread active\n");
				if (current_active_wp_id != sweep_wp_->seq || terminate_threads == true)
				{
					sweep_state = PX_WPP_SWEEP_IDLE;
					if (verbose && current_active_wp_id != sweep_wp_->seq) printf("Sweep: failed. Current waypoint changed. Thread terminated.\n");
					if (verbose && terminate_threads == true) printf("Sweep: Thread terminated.\n");
					g_mutex_unlock(main_mutex);
					return NULL;
				}

		    	yawReached = false;						///< boolean for yaw attitude reached
		    	posReached = false;						///< boolean for position reached
		    	check_if_reached_dest(&posReached, &yawReached, fake_next_wp_id);
	    	}
		}


		sweep_state = PX_WPP_SWEEP_FINISHED;
		if (verbose) printf("Sweep: finished. Thread terminated.\n");
	}
	else
	{
		sweep_state = PX_WPP_SWEEP_IDLE;
		if (verbose) printf("Sweep: failed. Wrong parameters. Thread terminated.\n");
	}
	g_mutex_unlock(main_mutex);
	return NULL;
}

void handle_mission (uint16_t seq, uint64_t now)
{
	if (debug) printf("Started executing waypoint(%u)...\n",seq);

	if (seq < waypoints->size() && wpp_state == PX_WPP_RUNNING)
	{
		mavlink_mission_item_t *cur_wp = waypoints->at(seq);
		if (ready_to_continue == false){
	    switch(cur_wp->command)
	    {
	    case MAV_CMD_NAV_WAYPOINT:
	    {
	    	set_destination(cur_wp);

	    	bool yawReached = false;						///< boolean for yaw attitude reached
	    	bool posReached = false;						///< boolean for position reached

            next_wp_id = seq+1;
            // compare last known position with current destination
            check_if_reached_dest(&posReached, &yawReached, next_wp_id);

            //check if the current waypoint was reached
            if (posReached && yawReached)
            {
            	if (timestamp_firstinside_orbit == 0)
            	{
            	     // Announce that last waypoint was reached
            		 if (verbose) printf("*** Reached waypoint %u ***\n", cur_wp->seq);
    	             send_mission_reached(cur_wp->seq);
         	         timestamp_firstinside_orbit = now;
         	    }
            	// check if the MAV was long enough inside the waypoint orbit
	            if(now-timestamp_firstinside_orbit >= cur_wp->param1*1000000)
	            {
	            	ready_to_continue = true;
	            	timestamp_firstinside_orbit = 0;
	            }
    	    }
    	    else
    	    {
    	        timestamp_lastoutside_orbit = now;
    	    }
    	    break;
	    }
	    case MAV_CMD_NAV_LOITER_UNLIM:
	    	break;
	    case MAV_CMD_NAV_LOITER_TURNS:
	    	break;
	    case MAV_CMD_NAV_LOITER_TIME:
	    	break;
	    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
	    	break;
	    case MAV_CMD_NAV_LAND:
	    {
	    	// generate a waypoint above the landing zone
	    	mavlink_mission_item_t* land_wp = new mavlink_mission_item_t;
	    	*land_wp = *cur_wp;
	    	land_wp->command = MAV_CMD_NAV_WAYPOINT;
	    	land_wp->param1 = 5.0;		// Per definition, there is a 5.0 sec delay before landing
	    	land_wp->param2 = 0.15;		// Per definition, acceptance radius for land waypoint is 0.15m
	    	//land_wp->param4 = cur_dest.yaw; // Yaw is not specified, so taking the value from last waypoint;
	    	if (permission_to_land == true)
	    	{
	    		if(above_landing(land_wp)==true)
	    		{
	    			// Set z-value to 0 to initiate landing procedure.
	    			land_wp->z = 0;
	    			set_destination(land_wp);
	    			break;
	    		}
	    		else
	    		{
	    			if (verbose) printf("Had permission to land, but was not above the landing zone (must be within %.2f meters from (%.2f,%.2f) on x-y plane)\n", land_wp->param2, land_wp->x,land_wp->y);
	    			permission_to_land = false;
	    		}
	    	}

    		set_destination(land_wp);

	    	bool yawReached = false;						///< boolean for yaw attitude reached
	    	bool posReached = false;						///< boolean for position reached

            // compare last known position with current destination
            check_if_reached_dest(&posReached, &yawReached, seq);

            //check if the current waypoint was reached
            if (posReached && yawReached)
            {
            	if (timestamp_firstinside_orbit == 0)
            	{
            	     // Announce that last waypoint was reached
            		 if (verbose) printf("*** Reached a checkpoint above the landing zone. ***\n");
    	             send_mission_reached(cur_wp->seq);
         	         timestamp_firstinside_orbit = now;
         	    }
            	// check if the MAV was long enough inside the waypoint orbit
	            if(now-timestamp_firstinside_orbit >= 5.0*1000000)
	            {
	            	if (verbose) printf("*** Landing permission granted. ***\n");
            		permission_to_land = true;
	            	timestamp_firstinside_orbit = 0;
	            }
    	    }
    	    else
    	    {
    	        timestamp_lastoutside_orbit = now;
    	    }

	    	// WARNING: this waypoint is intended to finish the flight. WPP will not proceed to the next waypoint, unless "current wp" is changed manually from QGroundControl.
	    	// ready_to_continue = true;
	    	// next_wp_id = seq+1;

    	    break;

	    }
	    case MAV_CMD_NAV_TAKEOFF:
	    {
	    	// generate a waypoint above the takeoff zone
	    	mavlink_mission_item_t* takeoff_wp = new mavlink_mission_item_t;
	    	*takeoff_wp = *cur_wp;
	    	takeoff_wp->command = MAV_CMD_NAV_WAYPOINT;
	    	takeoff_wp->param1 = 5.0;	// Per definition, takeoff includes a 5.0 sec delay after reaching desired height.
	    	takeoff_wp->param2 = 0.15;
	    	//takeoff_wp->param4 = last_known_att.yaw; // Setting the desired yaw to current yaw for take-off -> care windup!!
	    	set_destination(takeoff_wp);

	    	bool yawReached = false;						///< boolean for yaw attitude reached
	    	bool posReached = false;						///< boolean for position reached

            next_wp_id = seq+1;
            // compare last known position with current destination
            check_if_reached_dest(&posReached, &yawReached, next_wp_id);

            //check if the current waypoint was reached
            if (posReached && yawReached)
            {
            	if (timestamp_firstinside_orbit == 0)
            	{
            	     // Announce that last waypoint was reached
    	             send_mission_reached(cur_wp->seq);
         	         timestamp_firstinside_orbit = now;
         	    }
            	// check if the MAV was long enough inside the waypoint orbit
	            if(now-timestamp_firstinside_orbit >= takeoff_wp->param1*1000000)
	            {
	            	if (verbose) printf("*** Takeoff complete ***\n");
	            	ready_to_continue = true;
	            	timestamp_firstinside_orbit = 0;
	            }
    	    }
    	    else
    	    {
    	        timestamp_lastoutside_orbit = now;
    	    }
    	    break;
	    }
	    case MAV_CMD_NAV_LAST:
	    	break;
	    case MAV_CMD_CONDITION_DELAY:
	    {
	    	if (timestamp_delay_started == 0)
	    	{
	    		timestamp_delay_started = now;
	    		if (verbose) printf("Delay initiated (%.2f sec)...\n", cur_wp->param1);
	    		if (verbose && paramClient->getParamValue("HANDLEWPDELAY")>cur_wp->param1)
	    			{
	    				printf("Warning: Delay shorter than HANDLEWPDELAY parameter (%.2f sec)!\n", paramClient->getParamValue("HANDLEWPDELAY"));
	    			}
	    	}
	    	if (now - timestamp_delay_started >= cur_wp->param1*1000000)
	    	{
	    		timestamp_delay_started = 0;
	    		next_wp_id = seq + 1;
				ready_to_continue = true;
				if (verbose) printf("... delay finished. Proceed to next waypoint.\n");
	    	}
	    	else
	    	{
	    		if (verbose) printf("... %.2f sec left...\n", cur_wp->param1-(float)(now - timestamp_delay_started)/1000000.0);
	    	}
	    	break;
	    }
	    case MAV_CMD_CONDITION_CHANGE_ALT:
	    	break;
	    case MAV_CMD_CONDITION_DISTANCE:
	    	break;
	    case MAV_CMD_CONDITION_LAST:
	    	break;
	    case MAV_CMD_DO_SET_MODE:
	    	break;
	    case MAV_CMD_DO_JUMP:
	    {
	    	if (cur_wp->param1 < waypoints->size())
	    	{
				if (cur_wp->param2 > 0)
				{
					cur_wp->param2 = cur_wp->param2 - 1;
					if (verbose) printf("Jump from waypoint %u to waypoint %u. %u jumps left\n", current_active_wp_id, (uint32_t) cur_wp->param1, (uint32_t) cur_wp->param2);
					next_wp_id = cur_wp->param1;
				}
				else
				{
					next_wp_id = seq + 1;
	                if (verbose) printf("Jump command not performed: jump limit reached. Proceed to next waypoint\n");
				}
	    	}
	    	else
	    	{
	    		next_wp_id = seq + 1;
	    		cur_wp->autocontinue = false;
	    		printf("Invalid parameters for MAV_CMD_DO_JUMP waypoint. Next waypoint is set to %u. Autocontinue turned off. \n",next_wp_id);
	    	}
			ready_to_continue = true;
			break;
	    }
	    case MAV_CMD_DO_CHANGE_SPEED:
	    	break;
	    case MAV_CMD_DO_SET_HOME:
	    	break;
	    case MAV_CMD_DO_SET_PARAMETER:
	    	break;
	    case MAV_CMD_DO_SET_RELAY:
	    	break;
	    case MAV_CMD_DO_REPEAT_RELAY:
	    	break;
	    case MAV_CMD_DO_SET_SERVO:
	    	break;
	    case MAV_CMD_DO_REPEAT_SERVO:
	    {
	    	break;
	    }
#ifdef MAVLINK_ENABLED_PIXHAWK
	    case MAV_CMD_DO_START_SEARCH:
	    {
	    	min_conf = cur_wp->param1; // setting minimal confidence
	    	if (search_state == PX_WPP_SEARCH_IDLE)
	    	{
	    		//Start pattern recognition in a fork
	    		if (patternrec_pid == -1) //means that no patternrec is running yet
	    		{
	    			patternrec_pid = fork();

	    			if (patternrec_pid == 0)                // child
	    			{
	    				if (verbose) printf("Executing px_patternrec...\n");
	    				std::string fcn_call = "px_patternrec";
	    				std::string before_path = " -i '";
	    				std::string after_path = "'";
	    				std::string fcn_param = "-v";
	    				for (uint16_t i = 0; i<image_list->size(); i++)
	    				{
	    					std::string pic_path;
	    					pic_path = image_list->at(i);
	    					fcn_param = fcn_param + before_path + pic_path + after_path;
	    				}
	    				printf("Trying to execute: %s %s\n", fcn_call.c_str(),fcn_param.c_str());

	    		        char* argv[fcn_param.size() + 1]; // +1 for the trailing 0
	    		        for (unsigned int i = 0; i < fcn_param.size(); ++i)
	    		            argv[i] = const_cast<char*>(&fcn_param.at(i));
	    		        argv[fcn_param.size()] = 0;

	    		        if (execvp("px_patternrec",argv))
	    				{
	    					if (verbose) std::cerr << "px_patternrec failed." << std::endl;
	    					return;
	    				}
	    	            // We shoulnd't ever reach this line. If we do, exit.
	    	            _exit(EXIT_FAILURE);
	    			}
	    			else if (patternrec_pid < 0)            // failed to fork
	    			{
	    				std::cerr << "Failed to fork" << std::endl;
	    		        exit(1);
	    			}
	    			else //parent
	    			{
	    				usleep(500000); // give some time to patternrec to initialize
	    			}

	    		}
	    		else
	    		{
	    			//Kill old patternrec and start a new one, if necessary
	    		}

		    	if( !g_thread_supported() )
		    	{
		    		g_thread_init(NULL); // Only initialize g thread if not already done
		    	}

		    	int16_t detections_needed = (int16_t) cur_wp->param2;
	    		int16_t default_value = 1;
	    		gpointer ptr = (gpointer) &default_value;
		    	if (detections_needed > 0)
		    	{
		    		ptr = (gpointer) &detections_needed;
		    	}

		    	if( (search_thread = g_thread_create(search_thread_func, ptr, TRUE, &error)) == NULL)
		    	{
		    		printf("Thread creation failed: %s!!\n", error->message );
		    		g_error_free ( error ) ;
		    	}
		    	if (verbose) printf("Search thread created!\n");
	    	}
	    	else
	    	{
	    		if (verbose) printf("Another search already running! No new thread created. Minimal needed confidence set to %f\n",min_conf);
	    	}
	    	next_wp_id = seq + 1;
	    	ready_to_continue = true;
	    	break;
	    }
	    case MAV_CMD_DO_FINISH_SEARCH:
	    {
	    	if (search_state != PX_WPP_SEARCH_IDLE)
	    	{
				if (cur_wp->param3 > 0)
				{
					cur_wp->param3 = cur_wp->param3 - 1;

			    	if (cur_wp->param1 < waypoints->size() && cur_wp->param2 < waypoints->size())
			    	{
				    	if (search_state == PX_WPP_SEARCH_SUCCESS)
				    	{
				    		next_wp_id = cur_wp->param1;
				    		if (verbose) printf("Search successful! Proceeding to waypoint %u\n",next_wp_id);
				    	}
				    	else
				    	{
				    		next_wp_id = cur_wp->param2;
				    		if (verbose) printf("Search failed. Proceeding to waypoint %u\n",next_wp_id);
				    	}
				    	search_state = PX_WPP_SEARCH_RESET;
			    	}
			    	else
			    	{
			    		next_wp_id = seq + 1;
			    		cur_wp->autocontinue = false;
			    		printf("Invalid parameters for MAV_CMD_DO_FINISH_SEARCH waypoint. Next waypoint is set to %u. Autocontinue turned off. \n",next_wp_id);
			    	}
				}
				else
				{
					search_state = PX_WPP_SEARCH_END;
					g_cond_broadcast(cond_pattern_detected); //fake condition broadcast in order to wake the search thread for termination
		    		next_wp_id = seq + 1;
				}
	    	}
	    	else
	    	{
	    		next_wp_id = seq+1;
	    		printf("No active search found! Please add MAV_CMD_DO_START_SEARCH waypoint before this one. Proceeding to the next waypoint %u.\n",next_wp_id);
	    	}

			ready_to_continue = true;
	    	break;
	    }
	    /*
	    case MAV_CMD_DO_SEND_MESSAGE:
	    {
	    	mavlink_message_t msg;
	    	mavlink_statustext_t stext;

	    	uint16_t some_nr = 13;

	    	stext.severity = 0;
	    	sprintf((char*)&stext.text,"Some very important message: %u",some_nr);
	    	mavlink_msg_statustext_encode(systemid, compid, &msg, &stext);
	    	sendMAVLinkMessage(lcm, &msg);
	    	next_wp_id = seq + 1;
	    	ready_to_continue = true;
	    	break;
	    }
	    */

	    case MAV_CMD_NAV_SWEEP:
	    {
	    	if (sweep_state == PX_WPP_SWEEP_IDLE)
	    	{
		    	if( !g_thread_supported() )
		    	{
		    		g_thread_init(NULL); // Only initialize g thread if not already done
		    	}

	    		gpointer ptr = (gpointer) cur_wp;

		    	if( (sweep_thread = g_thread_create(sweep_thread_func, ptr, TRUE, &error)) == NULL)
		    	{
		    		printf("Thread creation failed: %s!!\n", error->message );
		    		g_error_free ( error ) ;
		    	}
		    	if (verbose) printf("Sweep thread created!\n");
	    	}
	    	else if (sweep_state == PX_WPP_SWEEP_RUNNING)
	    	{
	    		set_destination(next_sweep_wp);
	    	}
	    	else if (sweep_state == PX_WPP_SWEEP_FINISHED)
	    	{
	    		sweep_state = PX_WPP_SWEEP_IDLE;
		    	next_wp_id = seq + 1;
		    	ready_to_continue = true;
	    	}
	    	break;
	    }
#endif
	    default:
	    {
	    	printf("Unsupported MAV_CMD_ID (#%u). Removing Autocontinue. Change CURRENT manually to proceed.\n",cur_wp->command);
	    	cur_wp->autocontinue = false;
	    	next_wp_id = seq + 1;
	    	ready_to_continue = true;
	    	break;
	    }
	    }} //end switch, end if

		if (ready_to_continue == true)
		{
			if (cur_wp->autocontinue)
		    {
				//if (verbose) printf("check current (%u) next (%u) size (%u) \n", current_active_wp_id, next_wp_id, waypoints->size());
		        if (next_wp_id < waypoints->size())
		        {
			        cur_wp->current = false;
			        ready_to_continue = false;
		        	current_active_wp_id = next_wp_id;
		        	next_wp_id = -1;
		           	// Proceed to next waypoint
		            send_mission_current(current_active_wp_id);
		            waypoints->at(current_active_wp_id)->current = true;
		            if (verbose) printf("Set new waypoint (%u)\n", current_active_wp_id);
		            //Waypoint changed, execute next waypoint at once. Warning: recursion!!!
		            handle_mission(current_active_wp_id,now);
		        }
		        else //the end of waypoint list is reached.
		        {
		        	if (verbose) printf("Reached the end of the list.\n");
		        }
		    }
			else
			{
				if (verbose) printf("New waypoint (%u) has been set. Waiting for autocontinue...\n", next_wp_id);
			}
		}

	}
	else if (seq >= waypoints->size())
	{
		if (verbose) printf("Waypoint %u is out of bounds. Currently have %u waypoints.\n", seq, (uint16_t)waypoints->size());
	}
	else // wpp_state != PX_WPP_RUNNING
	{
		if (debug) printf("Waypoint not executed, waypointplanner not in running state.\n");
	}
	if (debug) printf("Finished executing waypoint(%u)...\n",seq);
	timestamp_last_handle_mission = now;

}



static void handle_communication (const mavlink_message_t* msg, uint64_t now)
{
	switch(msg->msgid)
	{
		case MAVLINK_MSG_ID_MISSION_ACK:
	        {
	            mavlink_mission_ack_t wpa;
	            mavlink_msg_mission_ack_decode(msg, &wpa);

	            if((msg->sysid == protocol_current_partner_systemid && msg->compid == protocol_current_partner_compid) && (wpa.target_system == systemid && (wpa.target_component == compid || wpa.target_component == MAV_COMP_ID_ALL)))
	            {
	                protocol_timestamp_lastaction = now;

	                if (comm_state == PX_WPP_COMM_SENDLIST || comm_state == PX_WPP_COMM_SENDLIST_SENDWPS)
	                {
	                	//FIXME: Different reactions on different wpa->types
	                	switch (wpa.type)
	                	{
	                	case MAV_MISSION_ACCEPTED:
		                    if (protocol_current_wp_id == waypoints->size()-1)
		                    {
		                        if (verbose) printf("Received Ack after having sent last waypoint, going to state PX_WPP_COMM_IDLE\n");
		                        comm_state = PX_WPP_COMM_IDLE;
		                        protocol_current_wp_id = 0;
		                    }
		                    else
		                    {
		                        if (verbose) printf("Missionplanner: Received Ack too early!!! Going to state PX_WPP_COMM_IDLE\n");
		                        comm_state = PX_WPP_COMM_IDLE;
		                        protocol_current_wp_id = 0;
		                    }
		                    break;
	                	default:
	                        printf("Missionplanner: Received Ack with an error (%u).Going to state PX_WPP_COMM_IDLE\n",wpa.type);
	                        comm_state = PX_WPP_COMM_IDLE;
	                        protocol_current_wp_id = 0;
	                		break;
	                	}
	                }
	            }
	            break;
	        }

	    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
	        {
	            mavlink_mission_set_current_t wpc;
	            mavlink_msg_mission_set_current_decode(msg, &wpc);

	            if(wpc.target_system == systemid && (wpc.target_component == compid || wpc.target_component == MAV_COMP_ID_ALL))
	            {
	                protocol_timestamp_lastaction = now;

	                if (comm_state == PX_WPP_COMM_IDLE)
	                {
	                    if (wpc.seq < waypoints->size())
	                    {
	                        if (verbose) printf("Received MAVLINK_MSG_ID_MISSION_SET_CURRENT\n");
	                        current_active_wp_id = wpc.seq;
	                        uint32_t i;
	                        for(i = 0; i < waypoints->size(); i++)
	                        {
	                            if (i == current_active_wp_id)
	                            {
	                                waypoints->at(i)->current = true;
	                            }
	                            else
	                            {
	                                waypoints->at(i)->current = false;
	                            }
	                        }
	                        if (verbose) printf("New current waypoint %u\n", current_active_wp_id);
	                        send_mission_current(current_active_wp_id);
	                        ready_to_continue = false;
	                        handle_mission(current_active_wp_id,now);
	                        send_setpoint();
	                        timestamp_firstinside_orbit = 0;
	                        timestamp_delay_started = 0;
	                    }
	                    else
	                    {
	                        if (verbose) printf("Ignored MAVLINK_MSG_ID_MISSION_SET_CURRENT: Index out of bounds\n");
	                    }
	                }
	            }
	            break;
	        }

	    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
	        {
	            mavlink_mission_request_list_t wprl;
	            mavlink_msg_mission_request_list_decode(msg, &wprl);
	            if(wprl.target_system == systemid && (wprl.target_component == compid || wprl.target_component == MAV_COMP_ID_ALL))
	            {
	                protocol_timestamp_lastaction = now;

	                if (comm_state == PX_WPP_COMM_IDLE || comm_state == PX_WPP_COMM_SENDLIST)
	                {
	                    if (waypoints->size() > 0)
	                    {
	                        if (verbose && comm_state == PX_WPP_COMM_IDLE) printf("Got MAVLINK_MSG_ID_MISSION_REQUEST_LIST from %u changing state to PX_WPP_COMM_SENDLIST\n", msg->sysid);
	                        if (verbose && comm_state == PX_WPP_COMM_SENDLIST) printf("Got MAVLINK_MSG_ID_MISSION_REQUEST_LIST again from %u staying in state PX_WPP_COMM_SENDLIST\n", msg->sysid);
	                        comm_state = PX_WPP_COMM_SENDLIST;
	                        protocol_current_wp_id = 0;
	                        protocol_current_partner_systemid = msg->sysid;
	                        protocol_current_partner_compid = msg->compid;
	                    }
	                    else
	                    {
	                        if (verbose) printf("Got MAVLINK_MSG_ID_MISSION_REQUEST_LIST from %u but have no waypoints, staying in \n", msg->sysid);
	                    }
	                    protocol_current_count = waypoints->size();
	                    send_mission_count(msg->sysid,msg->compid, protocol_current_count);
	                }
	                else
	                {
	                    if (verbose) printf("Ignored MAVLINK_MSG_ID_MISSION_REQUEST_LIST because i'm doing something else already (state=%i).\n", comm_state);
	                }
	            }
	            break;
	        }

	    case MAVLINK_MSG_ID_MISSION_REQUEST:
	        {
	            mavlink_mission_request_t wpr;
	            mavlink_msg_mission_request_decode(msg, &wpr);
	            if(msg->sysid == protocol_current_partner_systemid && msg->compid == protocol_current_partner_compid && wpr.target_system == systemid && (wpr.target_component == compid || wpr.target_component == MAV_COMP_ID_ALL))
	            {
	                protocol_timestamp_lastaction = now;

	                //ensure that we are in the correct state and that the first request has id 0 and the following requests have either the last id (re-send last waypoint) or last_id+1 (next waypoint)
	                if ((comm_state == PX_WPP_COMM_SENDLIST && wpr.seq == 0) || (comm_state == PX_WPP_COMM_SENDLIST_SENDWPS && (wpr.seq == protocol_current_wp_id || wpr.seq == protocol_current_wp_id + 1) && wpr.seq < waypoints->size()))
	                {
	                    if (verbose && comm_state == PX_WPP_COMM_SENDLIST) printf("Got MAVLINK_MSG_ID_MISSION_REQUEST of waypoint %u from %u changing state to PX_WPP_COMM_SENDLIST_SENDWPS\n", wpr.seq, msg->sysid);
	                    if (verbose && comm_state == PX_WPP_COMM_SENDLIST_SENDWPS && wpr.seq == protocol_current_wp_id + 1) printf("Got MAVLINK_MSG_ID_MISSION_REQUEST of waypoint %u from %u staying in state PX_WPP_COMM_SENDLIST_SENDWPS\n", wpr.seq, msg->sysid);
	                    if (verbose && comm_state == PX_WPP_COMM_SENDLIST_SENDWPS && wpr.seq == protocol_current_wp_id) printf("Got MAVLINK_MSG_ID_MISSION_REQUEST of waypoint %u (again) from %u staying in state PX_WPP_COMM_SENDLIST_SENDWPS\n", wpr.seq, msg->sysid);

	                    comm_state = PX_WPP_COMM_SENDLIST_SENDWPS;
	                    protocol_current_wp_id = wpr.seq;
	                    send_mission(protocol_current_partner_systemid, protocol_current_partner_compid, wpr.seq);
	                }
	                else
	                {
	                    if (verbose)
	                    {
	                        if (!(comm_state == PX_WPP_COMM_SENDLIST || comm_state == PX_WPP_COMM_SENDLIST_SENDWPS)) { printf("Ignored MAVLINK_MSG_ID_MISSION_REQUEST because i'm doing something else already (state=%i).\n", comm_state); break; }
	                        else if (comm_state == PX_WPP_COMM_SENDLIST)
	                        {
	                            if (wpr.seq != 0) printf("Ignored MAVLINK_MSG_ID_MISSION_REQUEST because the first requested waypoint ID (%u) was not 0.\n", wpr.seq);
	                        }
	                        else if (comm_state == PX_WPP_COMM_SENDLIST_SENDWPS)
	                        {
	                            if (wpr.seq != protocol_current_wp_id && wpr.seq != protocol_current_wp_id + 1) printf("Ignored MAVLINK_MSG_ID_MISSION_REQUEST because the requested waypoint ID (%u) was not the expected (%u or %u).\n", wpr.seq, protocol_current_wp_id, protocol_current_wp_id+1);
	                            else if (wpr.seq >= waypoints->size()) printf("Ignored MAVLINK_MSG_ID_MISSION_REQUEST because the requested waypoint ID (%u) was out of bounds.\n", wpr.seq);
	                        }
	                        else printf("Ignored MAVLINK_MSG_ID_MISSION_REQUEST - FIXME: missed error description\n");
	                    }
	                }
	            }
	            else
	            {
	                //we we're target but already communicating with someone else
	                if((wpr.target_system == systemid && (wpr.target_component == compid || wpr.target_component == MAV_COMP_ID_ALL)) && !(msg->sysid == protocol_current_partner_systemid && msg->compid == protocol_current_partner_compid))
	                {
	                    if (verbose) printf("Ignored MAVLINK_MSG_ID_MISSION_REQUEST from ID %u because i'm already talking to ID %u.\n", msg->sysid, protocol_current_partner_systemid);
	                }
	            }
	            break;
	        }

	    case MAVLINK_MSG_ID_MISSION_COUNT:
	        {
	            mavlink_mission_count_t wpc;
	            mavlink_msg_mission_count_decode(msg, &wpc);
	            if(wpc.target_system == systemid && (wpc.target_component == compid || wpc.target_component == MAV_COMP_ID_ALL))
	            {
	                protocol_timestamp_lastaction = now;

	                if (comm_state == PX_WPP_COMM_IDLE || (comm_state == PX_WPP_COMM_GETLIST && protocol_current_wp_id == 0))
	                {
	                    if (wpc.count > 0)
	                    {
	                        if (verbose && comm_state == PX_WPP_COMM_IDLE) printf("Got MAVLINK_MSG_ID_MISSION_COUNT (%u) from %u changing state to PX_WPP_COMM_GETLIST\n", wpc.count, msg->sysid);
	                        if (verbose && comm_state == PX_WPP_COMM_GETLIST) printf("Got MAVLINK_MSG_ID_MISSION_COUNT (%u) again from %u\n", wpc.count, msg->sysid);

	                        comm_state = PX_WPP_COMM_GETLIST;
	                        protocol_current_wp_id = 0;
	                        protocol_current_partner_systemid = msg->sysid;
	                        protocol_current_partner_compid = msg->compid;
	                        protocol_current_count = wpc.count;

	                        printf("clearing receive buffer and readying for receiving waypoints\n");
	                        while(waypoints_receive_buffer->size() > 0)
	                        {
	                            delete waypoints_receive_buffer->back();
	                            waypoints_receive_buffer->pop_back();
	                        }
	                        //terminate_all_threads();
	                        send_mission_request(protocol_current_partner_systemid, protocol_current_partner_compid, protocol_current_wp_id);
	                    }
	                    else if (wpc.count == 0)
	                    {
	                        printf("got waypoint count of 0, clearing waypoint list and staying in state PX_WPP_COMM_IDLE\n");
	                        while(waypoints_receive_buffer->size() > 0)
	                        {
	                            delete waypoints->back();
	                            waypoints->pop_back();
	                        }
	                        //terminate_all_threads();
	                        valid_destination_available = false;
	                        current_active_wp_id = -1;
	                        break;

	                    }
	                    else
	                    {
	                        if (verbose) printf("Ignoring MAVLINK_MSG_ID_MISSION_COUNT from %u with count of %u\n", msg->sysid, wpc.count);
	                    }
	                }
	                else
	                {
	                    if (verbose && !(comm_state == PX_WPP_COMM_IDLE || comm_state == PX_WPP_COMM_GETLIST)) printf("Ignored MAVLINK_MSG_ID_MISSION_COUNT because i'm doing something else already (state=%i).\n", comm_state);
	                    else if (verbose && comm_state == PX_WPP_COMM_GETLIST && protocol_current_wp_id != 0) printf("Ignored MAVLINK_MSG_ID_MISSION_COUNT because i'm already receiving waypoint %u.\n", protocol_current_wp_id);
	                    else printf("Ignored MAVLINK_MSG_ID_MISSION_COUNT - FIXME: missed error description\n");
	                }
	            }
	            break;
	        }

	    case MAVLINK_MSG_ID_MISSION_ITEM:
	        {
	            mavlink_mission_item_t wp;
	            mavlink_msg_mission_item_decode(msg, &wp);

	            if((msg->sysid == protocol_current_partner_systemid && msg->compid == protocol_current_partner_compid) && (wp.target_system == systemid && (wp.target_component == compid || wp.target_component == MAV_COMP_ID_ALL)))
	            {
	                protocol_timestamp_lastaction = now;
	                printf("Received WP %3u%s: Frame: %u\tCommand: %3u\tparam1: %6.2f\tparam2: %7.2f\tparam3: %6.2f\tparam4: %7.2f\tX: %7.2f\tY: %7.2f\tZ: %7.2f\tAuto-Cont: %u\t\n", wp.seq, (wp.current?"*":" "), wp.frame, wp.command, wp.param1, wp.param2, wp.param3, wp.param4, wp.x, wp.y, wp.z, wp.autocontinue);

	                //ensure that we are in the correct state and that the first waypoint has id 0 and the following waypoints have the correct ids
	                if ((comm_state == PX_WPP_COMM_GETLIST && wp.seq == 0) || (comm_state == PX_WPP_COMM_GETLIST_GETWPS && wp.seq == protocol_current_wp_id && wp.seq < protocol_current_count))
	                {
	                    if (verbose && comm_state == PX_WPP_COMM_GETLIST) printf("Got MAVLINK_MSG_ID_MISSION %u from %u changing state to PX_WPP_COMM_GETLIST_GETWPS\n", wp.seq, msg->sysid);
	                    if (verbose && comm_state == PX_WPP_COMM_GETLIST_GETWPS && wp.seq == protocol_current_wp_id) printf("Got MAVLINK_MSG_ID_MISSION %u from %u\n", wp.seq, msg->sysid);
	                    if (verbose && comm_state == PX_WPP_COMM_GETLIST_GETWPS && wp.seq-1 == protocol_current_wp_id) printf("Got MAVLINK_MSG_ID_MISSION %u (again) from %u\n", wp.seq, msg->sysid);

	                    comm_state = PX_WPP_COMM_GETLIST_GETWPS;
	                    protocol_current_wp_id = wp.seq + 1;
	                    mavlink_mission_item_t* newwp = new mavlink_mission_item_t;
	                    memcpy(newwp, &wp, sizeof(mavlink_mission_item_t));
	                    waypoints_receive_buffer->push_back(newwp);

	                    if(protocol_current_wp_id == protocol_current_count && comm_state == PX_WPP_COMM_GETLIST_GETWPS)
	                    {
	                        if (verbose) printf("Got all %u waypoints, changing state to PX_WPP_COMM_IDLE\n", protocol_current_count);

	                        send_mission_ack(protocol_current_partner_systemid, protocol_current_partner_compid, MAV_MISSION_ACCEPTED);

	                        if (current_active_wp_id > waypoints_receive_buffer->size()-1)
	                        {
	                            current_active_wp_id = waypoints_receive_buffer->size() - 1;
	                        }

	                        // switch the waypoints list
	                        std::vector<mavlink_mission_item_t*>* waypoints_temp = waypoints;
	                        waypoints = waypoints_receive_buffer;
	                        waypoints_receive_buffer = waypoints_temp;

	                        //get the new current waypoint
	                        uint32_t i;
	                        for(i = 0; i < waypoints->size(); i++)
	                        {
	                            if (waypoints->at(i)->current == 1)
	                            {
	                                current_active_wp_id = i;
	                                //if (verbose) printf("New current waypoint %u\n", current_active_wp_id);
	                                send_mission_current(current_active_wp_id);
	                                ready_to_continue = false;
	                                handle_mission(current_active_wp_id,now);
	                                send_setpoint();
	                                timestamp_firstinside_orbit = 0;
	                                timestamp_delay_started = 0;
	                                break;
	                            }
	                        }

	                        if (i == waypoints->size())
	                        {
	                            current_active_wp_id = -1;
	                            timestamp_firstinside_orbit = 0;
	                            timestamp_delay_started = 0;
	                        }

	                        comm_state = PX_WPP_COMM_IDLE;
	                    }
	                    else
	                    {
	                        send_mission_request(protocol_current_partner_systemid, protocol_current_partner_compid, protocol_current_wp_id);
	                    }
	                }
	                else
	                {
	                    if (comm_state == PX_WPP_COMM_IDLE)
	                    {
	                        //we're done receiving waypoints, answer with ack.
	                        send_mission_ack(protocol_current_partner_systemid, protocol_current_partner_compid, MAV_MISSION_ACCEPTED);
	                        printf("Received MAVLINK_MSG_ID_MISSION while state=PX_WPP_COMM_IDLE, answered with WAYPOINT_ACK.\n");
	                    }
	                    if (verbose)
	                    {
	                        if (!(comm_state == PX_WPP_COMM_GETLIST || comm_state == PX_WPP_COMM_GETLIST_GETWPS))
	                        {
	                        	printf("Ignored MAVLINK_MSG_ID_MISSION %u because i'm doing something else already (state=%i).\n", wp.seq, comm_state);
	                        	send_mission_ack(protocol_current_partner_systemid, protocol_current_partner_compid, MAV_MISSION_ERROR);
	                        	break;
	                        }
	                        else if (comm_state == PX_WPP_COMM_GETLIST)
	                        {
	                            if(!(wp.seq == 0))
	                            	{
	                            		printf("Ignored MAVLINK_MSG_ID_MISSION because the first waypoint ID (%u) was not 0.\n", wp.seq);
	                            		send_mission_ack(protocol_current_partner_systemid, protocol_current_partner_compid, MAV_MISSION_INVALID_SEQUENCE);
	                            	}
	                            else printf("Ignored MAVLINK_MSG_ID_MISSION %u - FIXME: missed error description\n", wp.seq);
	                        }
	                        else if (comm_state == PX_WPP_COMM_GETLIST_GETWPS)
	                        {
	                            if (!(wp.seq == protocol_current_wp_id))
	                            	{
	                            		printf("Ignored MAVLINK_MSG_ID_MISSION because the waypoint ID (%u) was not the expected %u.\n", wp.seq, protocol_current_wp_id);
	                            		send_mission_ack(protocol_current_partner_systemid, protocol_current_partner_compid, MAV_MISSION_INVALID_SEQUENCE);
	                            	}
	                            else if (!(wp.seq < protocol_current_count)) printf("Ignored MAVLINK_MSG_ID_MISSION because the waypoint ID (%u) was out of bounds.\n", wp.seq);
	                            else printf("Ignored MAVLINK_MSG_ID_MISSION %u - FIXME: missed error description\n", wp.seq);
	                        }
	                        else printf("Ignored MAVLINK_MSG_ID_MISSION %u - FIXME: missed error description\n", wp.seq);
	                    }
	                }
	            }
	            else
	            {
	                //we we're target but already communicating with someone else
	                if((wp.target_system == systemid && (wp.target_component == compid || wp.target_component == MAV_COMP_ID_ALL)) && !(msg->sysid == protocol_current_partner_systemid && msg->compid == protocol_current_partner_compid) && comm_state != PX_WPP_COMM_IDLE)
	                {
	                    if (verbose) printf("Ignored MAVLINK_MSG_ID_MISSION %u from ID %u because i'm already talking to ID %u.\n", wp.seq, msg->sysid, protocol_current_partner_systemid);
	                }
	                else if(wp.target_system == systemid && (wp.target_component == compid || wp.target_component == MAV_COMP_ID_ALL))
	                {
	                    if (verbose) printf("Ignored MAVLINK_MSG_ID_MISSION %u from ID %u because i have no idea what to do with it\n", wp.seq, msg->sysid);
	                }
	            }
	            break;
	        }

		case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
	        {
	            mavlink_mission_clear_all_t wpca;
	            mavlink_msg_mission_clear_all_decode(msg, &wpca);

	            if(wpca.target_system == systemid && (wpca.target_component == compid || wpca.target_component == MAV_COMP_ID_ALL) && comm_state == PX_WPP_COMM_IDLE)
	            {
	                protocol_timestamp_lastaction = now;

	                if (verbose) printf("Got MAVLINK_MSG_ID_MISSION_CLEAR_LIST from %u deleting all waypoints\n", msg->sysid);
	                while(waypoints->size() > 0)
	                {
	                    delete waypoints->back();
	                    waypoints->pop_back();
	                }
	                //terminate_all_threads();
	                valid_destination_available = false;
	                current_active_wp_id = -1;
	            }
	            else if (wpca.target_system == systemid && (wpca.target_component == compid || wpca.target_component == MAV_COMP_ID_ALL) && comm_state != PX_WPP_COMM_IDLE)
	            {
	                if (verbose) printf("Ignored MAVLINK_MSG_ID_MISSION_CLEAR_LIST from %u because i'm doing something else already (state=%i).\n", msg->sysid, comm_state);
	            }
	            break;
	        }

	    case MAVLINK_MSG_ID_ATTITUDE:
	        {
	            if(msg->sysid == systemid)
	            {
	                if(cur_dest.frame ==  MAV_FRAME_LOCAL_NED)
	                {
	                    mavlink_msg_attitude_decode(msg, &last_known_att);
	                    struct timeval tv;
                        gettimeofday(&tv, NULL);
                        uint64_t now = ((uint64_t)tv.tv_sec)*1000000 + tv.tv_usec;
                        if(now-timestamp_last_handle_mission > paramClient->getParamValue("HANDLEWPDELAY")*1000000 && current_active_wp_id != (uint16_t)-1)
                        {
                        	handle_mission(current_active_wp_id,now);
                        }
	                }
	            }
	            break;
	        }

	    case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
	        {
	            if(msg->sysid == systemid)
	            {
	                if(cur_dest.frame == MAV_FRAME_LOCAL_NED)
	                {
	                    mavlink_msg_local_position_ned_decode(msg, &last_known_pos);

	                    g_cond_broadcast (cond_position_received);

	                    if (debug) printf("Received new position: x: %f | y: %f | z: %f\n", last_known_pos.x, last_known_pos.y, last_known_pos.z);
	                    struct timeval tv;
                        gettimeofday(&tv, NULL);
                        uint64_t now = ((uint64_t)tv.tv_sec)*1000000 + tv.tv_usec;
                        if(now-timestamp_last_handle_mission > paramClient->getParamValue("HANDLEWPDELAY")*1000000 && current_active_wp_id != (uint16_t)-1)
                        {
                        	handle_mission(current_active_wp_id,now);
                        }
	                }
	            }
	            break;
	        }
	    case MAVLINK_MSG_ID_PATTERN_DETECTED:
			{
				mavlink_pattern_detected_t pd;
				mavlink_msg_pattern_detected_decode(msg, &pd);
				if (debug) printf("Pattern - conf: %f, detect: %i, file: %s, type: %i\n",pd.confidence,pd.detected,pd.file,pd.type);
				std::string SEARCH_PIC;
				SEARCH_PIC = image_list->at(0);
				if (debug) printf("Searching for file: %s. Result: %i\n",SEARCH_PIC.c_str(), strcmp((char*)pd.file,(char*)SEARCH_PIC.c_str()));
				if (search_state != PX_WPP_SEARCH_IDLE)
				{
					if(pd.detected==1 && strcmp((char*)pd.file,SEARCH_PIC.c_str()) == 0 && pd.confidence >= min_conf)
					{
						last_detected_pattern = pd;
						if(verbose) printf("Found it! - confidence: %f, detect: %i, file: %s, type: %i\n",pd.confidence,pd.detected,pd.file,pd.type);
						g_cond_broadcast (cond_pattern_detected);
					}
				}
				break;
			}
		case MAVLINK_MSG_ID_COMMAND_LONG:
			{
				mavlink_command_long_t command;
				mavlink_msg_command_long_decode(msg, &command);


	            if(command.target_system == systemid && (command.target_component == compid || command.target_component == MAV_COMP_ID_ALL))
	            {
	                protocol_timestamp_lastaction = now;

	                if (comm_state == PX_WPP_COMM_IDLE)
	                {
	    				switch (command.command)
	    				{
	    				/*
	    				case CMD_SET_AUTOCONTINUE:
	    					{
	    						uint8_t wp_id = (uint8_t) command.param1;
	    						if (wp_id < waypoints->size())
	    						{
		    						uint8_t new_autocontinue_value = (uint8_t) command.param2;
		    						if (new_autocontinue_value == 0)
		    						{
		    							waypoints->at(wp_id)->autocontinue = false;
		    							send_command_ack(CMD_SET_AUTOCONTINUE,0);
		    						}
		    						else if (new_autocontinue_value == 1)
		    						{
		    							waypoints->at(wp_id)->autocontinue = true;
		    							send_command_ack(CMD_SET_AUTOCONTINUE,0);
		    						}
		    						else
		    						{
		    							if (debug) std::cerr << "Waypointplanner: CMD_SET_AUTOCONTINUE command must have param2 value of 0 or 1" << std::endl;
		    							send_command_ack(CMD_SET_AUTOCONTINUE,2);
		    						}
	    						}
	    						else
	    						{
	    							if (verbose) printf("Ignored MAVLINK_MSG_ID_COMMAND (CMD_SET_AUTOCONTINUE): Waypoint index out of bounds\n");
	    							send_command_ack(CMD_SET_AUTOCONTINUE,2);
	    						}

	    						break;
	    					}
						*/
	    				case MAV_CMD_OVERRIDE_GOTO:
	    					{
	    						switch ((MAV_GOTO) command.param1)
		    					{
		    					case MAV_GOTO_DO_HOLD:
		    					{
		    						if (verbose) printf("Received HOLD command.\n");
			    					if (command.param2 == MAV_GOTO_HOLD_AT_CURRENT_POSITION)
			    					{
		    						set_destination(get_wp_of_current_position());
		    						wpp_state = PX_WPP_ON_HOLD;
		    						send_command_ack(MAV_CMD_OVERRIDE_GOTO,MAV_RESULT_ACCEPTED);
			    					}
			    					else
			    					{
			    						if (verbose) printf("Warning! param2 of MAV_CMD_OVERRIDE_GOTO command is ignored. Halting at current position.\n");
			    						send_command_ack(MAV_CMD_OVERRIDE_GOTO,MAV_RESULT_UNSUPPORTED);
			    					}

		    						break;
		    					}

		    					case MAV_GOTO_DO_CONTINUE:
		    					{
		    						if (verbose) printf("Received CONTINUE command.\n");
		    						send_command_ack(MAV_CMD_OVERRIDE_GOTO,MAV_RESULT_ACCEPTED);
		    						if (wpp_state != PX_WPP_RUNNING)
		    						{
	    								wpp_state = PX_WPP_RUNNING;
	    								handle_mission(current_active_wp_id,now);
		    						}
		    						else //Received CONTINUE order while not on hold. Interpret as overruling "autocontinue"
		    						{
		    							if(waypoints->at(current_active_wp_id)->autocontinue == false && ready_to_continue == true)
		    							{
		    								uint16_t prev_id = current_active_wp_id;
		    								waypoints->at(current_active_wp_id)->autocontinue = true;
		    								handle_mission(current_active_wp_id,now);
		    								waypoints->at(prev_id)->autocontinue = false; //changing autocontinue back to false
		    							}
		    						}


		    						break;
		    					}


		    					default:
		    					{
		    						printf("Received MAV_CMD_OVERRIDE_GOTO command with invalid/unsupported parameters.\n");
		    						send_command_ack(MAV_CMD_OVERRIDE_GOTO,MAV_RESULT_UNSUPPORTED);
		    					}
		    					}
	    						break;
	    					}

    					default:
    					{
    						std::cerr << "Missionplanner: received Command message of unknown type " << command.command << std::endl;
    		        		send_command_ack(command.command,MAV_RESULT_UNSUPPORTED);
    		            	break;
    		        	}
	    				}
	                }
	            }

				break;
			}

		default:
        {
            if (debug) std::cerr << "Missionplanner: received message of unknown type:" << msg->msgid << std::endl;
            break;
        }

	}
}

static void mavlink_handler (const lcm_recv_buf_t *rbuf, const char * channel, const mavconn_mavlink_msg_container_t* container, void * user)
{
	const mavlink_message_t* msg = getMAVLinkMsgPtr(container);

	g_mutex_lock(main_mutex);

    // Handle param messages
    paramClient->handleMAVLinkPacket(msg);

    //check for timed-out operations
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t now = ((uint64_t)tv.tv_sec)*1000000 + tv.tv_usec;
    if (now-protocol_timestamp_lastaction > paramClient->getParamValue("PROTTIMEOUT")*1000000 && comm_state != PX_WPP_COMM_IDLE)
    {
        if (verbose) printf("Last operation (state=%u) timed out, changing state to PX_WPP_COMM_IDLE\n", comm_state);
        comm_state = PX_WPP_COMM_IDLE;
        protocol_current_count = 0;
        protocol_current_partner_systemid = 0;
        protocol_current_partner_compid = 0;
        protocol_current_wp_id = -1;

        if(waypoints->size() == 0)
        {
        	valid_destination_available = false;
            current_active_wp_id = -1;
        }
    }

    handle_communication(msg, now);

    g_mutex_unlock(main_mutex);
}

void* lcm_thread_func (gpointer lcm_ptr)
{
	lcm_t* lcm = (lcm_t*) lcm_ptr;
	while (1)
	{
		lcm_handle(lcm);
	}
	return NULL;
}

int main(int argc, char* argv[])
/**
*  @brief main function of process (start here)
*
*  The function parses for program options, sets up some example waypoints and connects to IPC
*/
{
	std::string waypointfile;
	std::string imagelistfile;
    config::options_description desc("Allowed options");
    desc.add_options()
            ("help", "produce help message")
            ("debug,d", config::bool_switch(&debug)->default_value(false), "Emit debug information")
            ("verbose,v", config::bool_switch(&verbose)->default_value(false), "verbose output")
            ("config", config::value<std::string>(&configFile)->default_value("config/parameters_missionplanner.cfg"), "Config file for system parameters")
            ("waypointfile", config::value<std::string>(&waypointfile)->default_value(""), "Config file for waypoint")
            ("imagelistfile", config::value<std::string>(&imagelistfile)->default_value(""), "List of paths of images, which are used for search")
            ("nosetpointonhold", config::bool_switch(&nosetpointonhold)->default_value(false), "If true, MP will stop sending last known position as a setpoint while in HOLD state.")
            ;
    config::variables_map vm;
    config::store(config::parse_command_line(argc, argv, desc), vm);
    config::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 1;
    }


    //initialize current destination as (0,0,0) local coordinates
    cur_dest.frame = MAV_FRAME_LOCAL_NED; ///< The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
	cur_dest.x = 0; //local: x position, global: longitude
	cur_dest.y = 0; //local: y position, global: latitude
	cur_dest.z = 0; //local: z position, global: altitude
	cur_dest.yaw = 0; //Yaw orientation in degrees, [0..360] 0 = NORTH
	cur_dest.rad = 0; //Acceptance radius, in meter

    /**********************************
    * Run LCM and subscribe to MAVLINK
    **********************************/
    lcm = lcm_create ("udpm://");
    if (!lcm)
    {
    	printf("LCM failed.\n");
    	return NULL;
    }
    comm_sub = mavconn_mavlink_msg_container_t_subscribe (lcm, "MAVLINK", &mavlink_handler, NULL);


    /**********************************
    * Set default parameters and read parameters from file, if available.
    * Warning: Parameter name may not be longer than 14 characters!!!
    **********************************/
    paramClient = new MAVConnParamClient(systemid, compid, lcm, configFile, verbose);
    paramClient->setParamValue("POSFILTER", 1.f);
    paramClient->setParamValue("SETPOINTDELAY", 1.0);
    paramClient->setParamValue("HANDLEWPDELAY",0.2);
    paramClient->setParamValue("PROTDELAY",40);	 //Attention: microseconds!!
    paramClient->setParamValue("PROTTIMEOUT", 2.0);
    paramClient->setParamValue("YAWTOLERANCE", 0.1745f);
    paramClient->readParamsFromFile(configFile);

    /**********************************
    * Read list of images
    **********************************/

    if (imagelistfile.length())
    {
    	if(load_imagelist_from_file(imagelistfile))
    		{
    			printf("Error reading image list from file\n");
    		}
	}

    /**********************************
    * Run the LCM thread
    **********************************/
	if( !g_thread_supported() )
	{
		g_thread_init(NULL);
		// Only initialize g thread if not already done
	}

	gpointer lcm_ptr = (gpointer) lcm;

	if( (waypoint_lcm_thread = g_thread_create(lcm_thread_func, lcm_ptr, TRUE, &error)) == NULL)
	{
		printf("Thread creation failed: %s!!\n", error->message );
		g_error_free ( error ) ;
	}

    /**********************************
    * Initialize mutex(es) and Condition(s)
    **********************************/
	if (!main_mutex)
	{
		main_mutex = g_mutex_new();
		printf("Mutex created\n");
	}

	if (!cond_position_received)
	{
		cond_position_received = g_cond_new();
		cond_pattern_detected = g_cond_new();
		printf("Conditions created\n");
	}

    /**********************************
    * Read waypoints from file and
    * set the new current waypoint
    **********************************/
	g_mutex_lock(main_mutex);
	wpp_state = PX_WPP_RUNNING;
    if (waypointfile.length())
    {
        if(load_mission_from_file(waypointfile))
        {
            printf("Error reading mission from file\n");
            exit(1); // terminate with error
        }

        struct timeval tv;
        gettimeofday(&tv, NULL);
        uint64_t now = ((uint64_t)tv.tv_sec)*1000000 + tv.tv_usec;

        uint32_t i;
        for(i = 0; i < waypoints->size(); i++)
        {
            if (waypoints->at(i)->current == 1)
            {
        		current_active_wp_id = i;
        		if (verbose) printf("New current waypoint %u\n", current_active_wp_id);
        		handle_mission(current_active_wp_id,now);
        		send_setpoint();
        		break;
            }
        }
        if (i == waypoints->size())
        {
            current_active_wp_id = -1;
            timestamp_firstinside_orbit = 0;
        }

    }
    g_mutex_unlock(main_mutex);

    printf("WAYPOINTPLANNER INITIALIZATION DONE, RUNNING...\n");

    /**********************************
    * Main loop
    **********************************/
    while (1)//need some break condition, e.g. if LCM fails
    {
        if(current_active_wp_id != (uint16_t)-1 && (nosetpointonhold==false || wpp_state != PX_WPP_ON_HOLD))
        {
        	g_mutex_lock(main_mutex);
            send_setpoint();
            g_mutex_unlock(main_mutex);
        }
        usleep(paramClient->getParamValue("SETPOINTDELAY")*1000000);
    }

    /**********************************
    * Terminate the LCM
    **********************************/
	mavconn_mavlink_msg_container_t_unsubscribe (lcm, comm_sub);
	lcm_destroy (lcm);
	printf("WAYPOINTPLANNER TERMINATED\n");

}
