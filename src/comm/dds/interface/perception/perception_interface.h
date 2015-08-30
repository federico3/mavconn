#ifndef PERCEPTION_INTERFACE_H
#define PERCEPTION_INTERFACE_H

#include "../../Topic.h"
#include "dds_obstacle_map_message_tPlugin.h"
#include "dds_obstacle_map_message_tSupport.h"

namespace px
{

/**
 * Defines the type specific interface for dds_obstacle_map_message_t.
 */
class ObstacleMapTopic: public Topic< dds_obstacle_map_message_t,
									  dds_obstacle_map_message_tTypeSupport,
									  dds_obstacle_map_message_tDataReader,
									  dds_obstacle_map_message_tDataWriter >
{
public:
	/**
	* Returns a handle to the interface instance.
	* @return The instance handle
	*/
	static ObstacleMapTopic* instance(void);

protected:
	/**
	* A constructor. Does nothing.
	*/
	ObstacleMapTopic();

private:
	/**
	* The handle to the singleton instance.
	*/
	static ObstacleMapTopic* _instance;
};

}

#endif
