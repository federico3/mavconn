#include "perception_interface.h"

namespace px
{

const char* OBSTACLE_MAP_NAME = "obstacle_map";

ObstacleMapTopic* ObstacleMapTopic::_instance = 0;

ObstacleMapTopic* ObstacleMapTopic::instance()
{
	if (_instance == 0)
	{
		_instance = new ObstacleMapTopic;
		_instance->topicName.assign(OBSTACLE_MAP_NAME);
		_instance->topicType = TOPIC_PUBLISH_SUBSCRIBE;
		_instance->plugin = dds_obstacle_map_message_tPlugin_new();
	}
	return _instance;
}

ObstacleMapTopic::ObstacleMapTopic() { };

}
