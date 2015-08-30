#ifndef MIDDLEWARE_H
#define MIDDLEWARE_H

#include "TopicManagerFactory.h"

namespace px
{

/**
 * Thread priority level.
 */
enum PriorityKind
{
	PRIORITY_MAX,        /**< Shouldn't be necessary */
	PRIORITY_LOWLATENCY, /**< For CPU-light sensor apps */
	PRIORITY_REALTIME,   /**< Use real-time scheduling */
	PRIORITY_DEFAULT,    /**< Above average priority (nice-level -5) but no
							   real-time */
	PRIORITY_NORMAL,     /**< Unchanged priority (nice-level 0) */
	PRIORITY_LOW         /**< For batch processes */
};

class Middleware
{
public:
	Middleware();

	/**
	 * Start up middleware. This is required at the beginning of
	 * any program utilizing the middleware.
	 * @param argc Number of arguments.
	 * @param argv List of arguments.
	 */
	void init(int argc, char** argv);

	/**
	 * Kill message listening thread and delete all middleware entities.
	 */
	void shutdown(void);

	/**
	 * Logs messages associated with a topic to a file.
	 * @param topic Name of topic.
	 * @param logHandler Callback function to be called when a new message
	 *                    is available.
	 * @param startTime Time at which logging starts.
	 * @param logfile File pointer to log file.
	 * @param subscribeKind parameter controlling how new messages are
	 *                      handled.
	 * @return A boolean value indicating whether the operation is successful.
	 */
	bool log(const std::string &topicString,
	         LogHandler& logHandler, double startTime,
	         FILE* logfile, SubscriptionKind subscribeKind);

	/**
	 * Sends a request and waits for an associated response.
	 * @param request Pointer to request message.
	 * @param response Pointer to response message.
	 * @param timeout Timeout in milliseconds.
	 * @return A boolean value indicating whether the operation is successful.
	 */
	template< typename TQueryTopic,
	          typename TResponseTopic >
	bool queryResponse(typename TQueryTopic::data_type& query,
	                   typename TResponseTopic::data_type& response,
	                   unsigned int timeout_ms)
	{
		TopicManager *topicManager = TopicManagerFactory::getTopicManager();
		return topicManager->queryResponse<TQueryTopic,TResponseTopic>(query, response, timeout_ms);
	}

private:
	Glib::Thread* listenThread;
	bool quit;
};

}

#endif
