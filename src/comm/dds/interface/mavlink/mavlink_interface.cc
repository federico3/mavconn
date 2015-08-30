#include "mavlink_interface.h"

namespace px
{

const char* MAVLINK_NAME = "mavlink";

MavlinkTopic* MavlinkTopic::_instance = 0;

MavlinkTopic* MavlinkTopic::instance()
{
	if (_instance == 0)
	{
		_instance = new MavlinkTopic;
		_instance->topicName.assign(MAVLINK_NAME);
		_instance->topicType = TOPIC_PUBLISH_SUBSCRIBE;
		_instance->plugin = dds_mavlink_message_tPlugin_new();
	}
	return _instance;
}

MavlinkTopic::MavlinkTopic() { };

}
