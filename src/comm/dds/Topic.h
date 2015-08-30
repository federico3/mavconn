#ifndef TOPIC_H
#define TOPIC_H

#include <cassert>

#include "TopicManagerFactory.h"

// forward declaration
struct PRESTypePlugin;

namespace px
{

template< typename TData,
          class    TTypeSupport,
          class    TDataReader,
          class    TDataWriter >
class Topic
{
public:
	/**
	 * Public typedefs for emulating traits.
	 */
	typedef TData data_type;
	typedef TTypeSupport support_type;
	typedef TDataReader data_reader_type;
	typedef TDataWriter data_writer_type;

	/**
	 * Destructor.
	 */
	virtual ~Topic();

	/**
	 * Advertises a topic.
	 * @return A boolean value indicating whether the operation is successful.
	 */
	bool advertise(void);

	/**
	 * Publish TData messages.
	 * @param sample Pointer to memory location for unmarshalled message.
	 * @return A boolean value indicating whether the operation is successful.
	 */
	bool publish(TData* sample);

	/**
	 * Register as a listener for a single TData message.
	 *
	 * @param handler Callback function to be called when a new message is
	 *                available.
	 *
	 * @return A boolean value indicating whether the operation is successful.
	 */
	bool listenSingle(Handler& handler);

	/**
	 * Subscribe to TData messages. Memory will be allocated for the message.
	 *
	 * @param handler Callback function to be called when a new message is
	 *                available.
	 * @param subscribeKind parameter controlling how new messages are
	 *                      handled.
	 *
	 * @return A boolean value indicating whether the operation is successful.
	 */
	bool subscribe(Handler& handler, SubscriptionKind subscribeKind);

	/**
	 * Unsubscribe to TData messages.
	 * @param handler No longer subscribe to messages addressed to this handler.
	 *                If handler == NULL, unsubscribe to all registered handlers.
	 * @return A boolean value indicating whether the operation is successful.
	 */
	bool unsubscribe(Handler& handler);

	/**
	 * Logs TData messages.
	 * @param logHandler Callback function to be called when a new message
	 *                   is available.
	 * @param startTime Time at which logging starts.
	 * @param logfile File pointer to logfile.
	 * @param subscribeKind parameter controlling how new messages are
	 *                      handled.
	 * @return A boolean value indicating whether the operation is successful.
	 */
	bool log(LogHandler& logHandler, double startTime,
			 FILE* logfile, SubscriptionKind subscribeKind);

	const std::string& getName(void) const { return topicName; }
	const std::string& getReverseName(void) const;
	TopicType getType(void) const { return topicType; }
	TransportBuiltinPolicy getTransportBuiltinPolicy() const { return transportBuiltin; };

protected:
	/**
	 * Constructor. The constructor is protected for implementation of the Singleton pattern.
	 */
	Topic();

	/**
	 * Name of topic.
	 */
	std::string topicName;

	/**
	 * Type of topic: query/reply, publish/subscribe.
	 */
	TopicType topicType;

	/**
	 * List of enabled built-in transports.
	 */
	TransportBuiltinPolicy transportBuiltin;

	/**
	 * Plug-in containing message manipulation functions.
	 */
	PRESTypePlugin* plugin;

	/**
	 * Name of the other corresponding topic, only for a query/reply topic. All query/reply topic
	 * relationships must be strictly one-to-one and unique.
	 */
	std::string reverseTopicName;

private:
	/**
	 * Pointer to a topic callback set
	 */
	TopicCallbackSet* topicCallbackSet;

	/**
	 * Copy constructor and copy assignment operator. These methods are kept private to prevent copying of topics.
	 */
	//@{
	Topic(const Topic&);
	Topic& operator=(const Topic);
	//@}

	/**
	 * Topic registration helper method.
	 */
	TopicCallbackSet* registerTopicHelper(void);
};

template< typename TData,
          class    TTypeSupport,
          class    TDataReader,
          class    TDataWriter >
Topic< TData, TTypeSupport, TDataReader, TDataWriter >::
	Topic()
{
	transportBuiltin.mask = TRANSPORTBUILTIN_UDP;

	topicName.assign("");
	topicType = TOPIC_PUBLISH_SUBSCRIBE;
	topicCallbackSet = 0;
	reverseTopicName.assign("");
	plugin = 0;
}

template< typename TData,
          class    TTypeSupport,
          class    TDataReader,
          class    TDataWriter >
Topic< TData, TTypeSupport, TDataReader, TDataWriter >::
    ~Topic() { }

template< typename TData,
          class    TTypeSupport,
          class    TDataReader,
          class    TDataWriter >
TopicCallbackSet* Topic< TData, TTypeSupport, TDataReader, TDataWriter >::
    registerTopicHelper(void)
{
  TopicManager* topicManager = TopicManagerFactory::getTopicManager();

  return topicManager->registerTopic(*this, plugin);
}

template< typename TData,
          class    TTypeSupport,
          class    TDataReader,
          class    TDataWriter >
bool Topic< TData, TTypeSupport, TDataReader, TDataWriter >::
    advertise(void)
{
	TopicManager* topicManager = TopicManagerFactory::getTopicManager();

	topicCallbackSet = registerTopicHelper();
	if (topicCallbackSet == 0)
	{
		fprintf(stderr, "# WARNING (TOPIC): Attempt to register topic %s failed to return a callback set.\n",
				topicName.c_str());
		return false;
	}

	if (!topicManager->registerPublisher(*this))
	{
		fprintf(stderr, "# WARNING (TOPIC): Attempt to register publisher for topic %s failed.\n",
				topicName.c_str());
		return false;
	}

	return true;
}

template< typename TData,
          class    TTypeSupport,
          class    TDataReader,
          class    TDataWriter >
bool Topic< TData, TTypeSupport, TDataReader, TDataWriter >::
    publish(TData* sample)
{
	assert(sample != 0);

	TopicManager* topicManager = TopicManagerFactory::getTopicManager();

	if (topicCallbackSet == 0)
	{
		fprintf(stderr, "# WARNING (TOPIC): Topic %s is not registered.\n", topicName.c_str());
		return false;
	}

	bool publishSuccess = topicManager->publish(topicCallbackSet, sample);
	if (publishSuccess == false)
	{
		fprintf(stderr, "# WARNING (TOPIC): Attempt to publish %s sample failed.\n",
				topicName.c_str());
		return false;
	}

	return true;
}

template< typename TData,
          class    TTypeSupport,
          class    TDataReader,
          class    TDataWriter >
bool Topic< TData, TTypeSupport, TDataReader, TDataWriter >::
	listenSingle(Handler& handler)
{
	TopicManager* topicManager = TopicManagerFactory::getTopicManager();

	topicCallbackSet = registerTopicHelper();
	if (topicCallbackSet == 0)
	{
		fprintf(stderr, "# WARNING (TOPIC): Attempt to register topic %s failed during a listen single operation.\n",
				topicName.c_str());
		return false;
	}

	bool listenSingleSuccess = topicManager->listenSingle(topicName, handler);
	if (listenSingleSuccess == false)
	{
		fprintf(stderr, "# WARNING (TOPIC): Attempt to listen for a single sample failed for topic %s.\n",
				topicName.c_str());
		return false;
	}

	bool registerSubscriberSuccess = topicManager->registerSubscriber(*this, false);
	if (registerSubscriberSuccess == false)
	{
		fprintf(stderr, "# WARNING (TOPIC): Attempt to register subscriber for topic %s failed during a listen single operation.\n",
				topicName.c_str());
		return false;
	}

	return true;
}


template< typename TData,
          class    TTypeSupport,
          class    TDataReader,
          class    TDataWriter >
bool Topic< TData, TTypeSupport, TDataReader, TDataWriter >::
	subscribe(Handler& handler, SubscriptionKind subscribeKind)
{
	TopicManager* topicManager = TopicManagerFactory::getTopicManager();

	topicCallbackSet = registerTopicHelper();
	if (topicCallbackSet == 0)
	{
		fprintf(stderr, "# WARNING (TOPIC): Attempt to register topic %s failed to return a callback set.\n",
				topicName.c_str());
		return false;
	}

	bool subscribeSuccess = topicManager->subscribe(topicName, handler,
												    subscribeKind);
	if (subscribeSuccess == false)
	{
		fprintf(stderr, "# WARNING (TOPIC): Attempt to subscribe failed for topic %s.\n",
				topicName.c_str());
		return false;
	}

	bool registerSubscriberSuccess = topicManager->registerSubscriber(*this, true);
	if (registerSubscriberSuccess == false)
	{
		fprintf(stderr, "# WARNING (TOPIC): Attempt to register subscriber for topic %s failed during a subscribe operation.\n",
				topicName.c_str());
		return false;
	}

	return true;
}

template< typename TData,
          class    TTypeSupport,
          class    TDataReader,
          class    TDataWriter >
bool Topic< TData, TTypeSupport, TDataReader, TDataWriter >::
    unsubscribe(Handler& handler)
{
	return TopicManagerFactory::getTopicManager()->unsubscribe(topicName, handler);
}

template< typename TData,
          class    TTypeSupport,
          class    TDataReader,
          class    TDataWriter >
bool Topic< TData, TTypeSupport, TDataReader, TDataWriter >::
	log(LogHandler& logHandler, double startTime,
	    FILE* logfile, SubscriptionKind subscribeKind)
{
	TopicManager* topicManager = TopicManagerFactory::getTopicManager();

	topicCallbackSet = registerTopicHelper();
	if (topicCallbackSet == 0)
	{
		fprintf(stderr, "# WARNING (TOPIC): Attempt to register topic %s failed to return a callback set.\n",
				topicName.c_str());
		return false;
	}

	bool logSuccess = topicManager->log(topicName, logHandler, startTime, logfile, subscribeKind);
	if (logSuccess == false)
	{
		fprintf(stderr, "# WARNING (TOPIC): Attempt to log failed for topic %s.\n",
				topicName.c_str());
		return false;
	}

	bool registerSubscriberSuccess = topicManager->registerSubscriber(*this, true);
	if (registerSubscriberSuccess == false)
	{
		fprintf(stderr, "# WARNING (TOPIC): Attempt to register subscriber for topic %s failed during a subscribe to local operation.\n",
				topicName.c_str());
		return false;
	}

	return true;
}

template< typename TData,
          class    TTypeSupport,
          class    TDataReader,
          class    TDataWriter >
const std::string& Topic< TData, TTypeSupport, TDataReader, TDataWriter >::
	getReverseName(void) const
{
	assert(topicType == TOPIC_QUERY_REPLY);
	assert(reverseTopicName.empty() == false);
	assert(reverseTopicName != topicName);

	return reverseTopicName;
}

}

#endif
