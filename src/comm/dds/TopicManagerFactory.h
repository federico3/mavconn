#ifndef TOPICMANAGERFACTORY_H
#define TOPICMANAGERFACTORY_H

#include "TopicManager.h"
#include "DDSTopicManager.h"

namespace px
{
// Topic Manager Factory

// The choice of default middleware should be done here
typedef DDSTopicManager TopicManager;
const MiddlewareTypeMask DEFAULT_MIDDLEWARE_MASK = MIDDLEWARE_RTI_DDS;

class TopicManagerFactory
{
public:
	/**
	* Get the default TopicManager
	*/
	static TopicManager* getTopicManager(void);
	static DDSTopicManager* getDDSTopicManager(void);

private:
	TopicManagerFactory();
	TopicManagerFactory(const TopicManagerFactory&);
	TopicManagerFactory& operator=(const TopicManagerFactory);
};

}

#endif
