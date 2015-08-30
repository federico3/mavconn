#ifndef RGBD_IMAGE_INTERFACE_H
#define RGBD_IMAGE_INTERFACE_H

#include "../../Topic.h"
#include "dds_rgbd_image_message_tPlugin.h"
#include "dds_rgbd_image_message_tSupport.h"

namespace px
{

/**
 * Defines the type specific interface for dds_rgbd_image_message_t.
 */
class RGBDImageTopic: public Topic< dds_rgbd_image_message_t,
									dds_rgbd_image_message_tTypeSupport,
									dds_rgbd_image_message_tDataReader,
									dds_rgbd_image_message_tDataWriter >
{
public:
	/**
	* Returns a handle to the interface instance.
	* @return The instance handle
	*/
	static RGBDImageTopic* instance(void);

protected:
	/**
	* A constructor. Does nothing.
	*/
	RGBDImageTopic();

private:
	/**
	* The handle to the singleton instance.
	*/
	static RGBDImageTopic* _instance;
};

}

#endif
