#include "image_interface.h"

namespace px
{

const char* IMAGE_NAME = "image";

ImageTopic* ImageTopic::_instance = 0;

ImageTopic* ImageTopic::instance()
{
	if (_instance == 0)
	{
		_instance = new ImageTopic;
		_instance->topicName.assign(IMAGE_NAME);
		_instance->topicType = TOPIC_PUBLISH_SUBSCRIBE;
		_instance->plugin = dds_image_message_tPlugin_new();
	}
	return _instance;
}

ImageTopic::ImageTopic() { };

}
