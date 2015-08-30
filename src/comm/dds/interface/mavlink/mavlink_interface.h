#ifndef MAVLINK_INTERFACE_H
#define MAVLINK_INTERFACE_H

#include "../../Topic.h"
#include "dds_mavlink_message_tPlugin.h"
#include "dds_mavlink_message_tSupport.h"

namespace px
{

/**
 * Defines the type specific interface for dds_mavlink_message_t.
 */
class MavlinkTopic: public Topic< dds_mavlink_message_t,
								  dds_mavlink_message_tTypeSupport,
								  dds_mavlink_message_tDataReader,
								  dds_mavlink_message_tDataWriter >
{
public:
	/**
	* Returns a handle to the interface instance.
	* @return The instance handle
	*/
	static MavlinkTopic* instance(void);

protected:
	/**
	* A constructor. Does nothing.
	*/
	MavlinkTopic();

private:
	/**
	* The handle to the singleton instance.
	*/
	static MavlinkTopic* _instance;
};

}

#endif
