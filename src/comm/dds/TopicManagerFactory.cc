#include "TopicManagerFactory.h"
#include "DDSTopicManager.h"

namespace px
{

TopicManager* TopicManagerFactory::getTopicManager(void)
{
	return TopicManager::getInstance();
}

DDSTopicManager* TopicManagerFactory::getDDSTopicManager(void)
{
	return DDSTopicManager::getInstance();
}

}
