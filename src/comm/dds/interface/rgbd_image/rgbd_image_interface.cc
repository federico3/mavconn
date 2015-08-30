#include "rgbd_image_interface.h"

namespace px
{

const char* RGBD_IMAGE_NAME = "rgbd_image";

RGBDImageTopic* RGBDImageTopic::_instance = 0;

RGBDImageTopic* RGBDImageTopic::instance()
{
	if (_instance == 0)
	{
		_instance = new RGBDImageTopic;
		_instance->topicName.assign(RGBD_IMAGE_NAME);
		_instance->topicType = TOPIC_PUBLISH_SUBSCRIBE;
		_instance->plugin = dds_rgbd_image_message_tPlugin_new();
	}
	return _instance;
}

RGBDImageTopic::RGBDImageTopic() { };

}
