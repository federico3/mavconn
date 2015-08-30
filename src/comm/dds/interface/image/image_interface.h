#ifndef IMAGE_INTERFACE_H
#define IMAGE_INTERFACE_H

#include "../../Topic.h"
#include "dds_image_message_tPlugin.h"
#include "dds_image_message_tSupport.h"

namespace px
{

/**
 * Defines the type specific interface for dds_image_message_t.
 */
class ImageTopic: public Topic< dds_image_message_t,
								dds_image_message_tTypeSupport,
								dds_image_message_tDataReader,
								dds_image_message_tDataWriter >
{
public:
	/**
	* Returns a handle to the interface instance.
	* @return The instance handle
	*/
	static ImageTopic* instance(void);

protected:
	/**
	* A constructor. Does nothing.
	*/
	ImageTopic();

private:
	/**
	* The handle to the singleton instance.
	*/
	static ImageTopic* _instance;
};

}

#endif
