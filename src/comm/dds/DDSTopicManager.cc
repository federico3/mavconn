#include "DDSTopicManager.h"

#include <cassert>

namespace px
{

DDSTopicManager* DDSTopicManager::instance = NULL;

DDSTopicManager* DDSTopicManager::getInstance(void)
{
	if (instance == 0)
	{
		instance = new DDSTopicManager();
	}

	return instance;
}

DDSTopicManager::DDSTopicManager()
  : mParticipant(NULL)
  , mSubscriber(NULL)
  , mWaitset(NULL)
{
}

bool DDSTopicManager::start(int argc, char** argv,
                            MiddlewarePolicy &middlewarePolicy)
{
	// Find option for filename of QoS profile
	Glib::OptionGroup optGroupDDS("dds", "DDS options", "DDS configuration options");

	Glib::OptionEntry optProfile;
	optProfile.set_short_name('p');
	optProfile.set_long_name("profile");
	optProfile.set_description("Path to DDS QoS profile file");

	std::string profilePath;
	optGroupDDS.add_entry_filename(optProfile, profilePath);

	Glib::OptionContext optContext("");
	optContext.set_help_enabled(true);
	optContext.set_ignore_unknown_options(true);
	optContext.set_main_group(optGroupDDS);

	try
	{
		if (!optContext.parse(argc, argv))
		{
			fprintf(stderr, "# ERROR: Cannot parse options.\n");
			return false;
		}
	}
	catch (Glib::OptionError& error)
	{
		fprintf(stderr, "# ERROR: Cannot parse options.\n");
		return false;
	}

	std::ostringstream oss;
	oss << "[file://" << profilePath << "]";

	if (setenv("NDDS_QOS_PROFILES", oss.str().c_str(), 0) != 0)
	{
		fprintf(stderr, "# ERROR: Cannot set NDDS_QOS_PROFILES environment variable.\n");
		return false;
	}

	if (!(middlewarePolicy.mask & MIDDLEWARE_RTI_DDS))
	{
		return false;
	}

	if (middlewarePolicy.ddsDomainIds.empty())
	{
		return false;
	}

	DDSDomainParticipantFactory* factory = DDSDomainParticipantFactory::get_instance();
	if (factory == NULL)
	{
		fprintf(stderr, "# ERROR: Unable to get domain mParticipant factory instance.\n");
		exit(EXIT_FAILURE);
	}

	mParticipant = factory->create_participant_with_profile(middlewarePolicy.ddsDomainIds[0],
															"PIXHAWK",
															"Reliable",
															NULL,
															DDS_STATUS_MASK_NONE);
	if (mParticipant == NULL)
	{
		fprintf(stderr, "# ERROR: Unable to create DDS domain mParticipant.\n");
		exit(EXIT_FAILURE);
	}

	mSubscriber = mParticipant->create_subscriber(DDS_SUBSCRIBER_QOS_DEFAULT, NULL,	DDS_STATUS_MASK_NONE);
	if (mSubscriber == NULL)
	{
		fprintf(stderr, "# ERROR: Unable to create mSubscriber.\n");
		exit(EXIT_FAILURE);
	}

	mWaitsetMutex.lock();
	mWaitset = new DDSWaitSet();
	mWaitsetMutex.unlock();

	return true;
}

bool DDSTopicManager::shutdown(void)
{
	DDS_ReturnCode_t retcode;

	for (StringMetadataMap::iterator it = mStringMetadataMap.begin();
		 it != mStringMetadataMap.end(); ++it)
	{
		unregisterPublisher(it->first);
		unregisterSubscriber(it->first);

		retcode = mParticipant->delete_topic(it->second.topic);
		if (retcode != DDS_RETCODE_OK)
		{
			fprintf(stderr, "# WARNING: delete_topic error %d\n", retcode);
			return false;
		}
	}
	mStringMetadataMap.clear();

	for (StringTopicMap::iterator it = mStringTopicMap.begin();
		 it != mStringTopicMap.end(); ++it)
	{
		delete it->second;
	}

	mStringTopicMap.clear();

	if (mParticipant != NULL)
	{
		retcode = mParticipant->delete_contained_entities();
		if (retcode != DDS_RETCODE_OK)
		{
			fprintf(stderr, "# WARNING: delete_contained_entities error %d\n", retcode);
			return false;
		}

		retcode = DDSTheParticipantFactory->delete_participant(mParticipant);
		if (retcode != DDS_RETCODE_OK)
		{
			fprintf(stderr, "# WARNING: delete_participant error %d\n", retcode);
			return false;
		}
	}

	retcode = DDSDomainParticipantFactory::finalize_instance();
	if (retcode != DDS_RETCODE_OK)
	{
		fprintf(stderr, "# WARNING: finalize_instance error %d\n", retcode);
		return false;
	}

	return true;
}

bool DDSTopicManager::listenSingle(const std::string& topicName __attribute__((unused)),
                                   Handler& handler __attribute__((unused)))
{
	return true;
}

bool DDSTopicManager::subscribe(const std::string& topicName, Handler& handler,
                                SubscriptionKind subscribeKind)
{
	DDS_ReturnCode_t retcode;

	TopicCallbackSet* topicCallbackSet = lookupTopicCallbackSet(topicName);
	if (topicCallbackSet == 0)
	{
		fprintf(stderr, "# WARNING (DDSTopicManager): Topic not registered.\n");
		return false;
	}

	DDS_DataReaderQos reader_qos;
	mSubscriber->get_default_datareader_qos(reader_qos);

	switch (subscribeKind)
	{
	case UNSUBSCRIBE:
		fprintf(stderr, "# ERROR (DDSTopicManager): invalid parameter: UNSUBSCRIBE for dds_subscribe_topic");
		exit(EXIT_FAILURE);
		break;
	case SUBSCRIBE_LATEST:
		setPeriodicSubscriberQos(reader_qos, 1);
		break;
	case SUBSCRIBE_ALL:
		setPeriodicSubscriberQos(reader_qos, 100);
		break;
	case SUBSCRIBE_ALL_LONGQUEUELIMIT:
		setPeriodicSubscriberQos(reader_qos, 1000);
		fprintf(stderr, "# WARNING (DDSTopicManager): Long DDS sample queue selected.\n"
				"This piece of software should not run live on the robot.\n");
		break;
	}

	retcode = mSubscriber->set_default_datareader_qos(reader_qos);
	if (retcode != DDS_RETCODE_OK)
	{
		fprintf(stderr, "# WARNING (DDSTopicManager): set_default_datareader_qos error %d\n", retcode);
		return false;
	}

	// make sure that the callback isn't already in there
	bool foundCallback = false;
	for (size_t i = 0; i < topicCallbackSet->callback.size(); ++i)
	{
		if (topicCallbackSet->callback[i].getHandler() == handler)
		{
			foundCallback = true;
			break;
		}
	}

	if (!foundCallback)
	{
		Callback callback(topicCallbackSet->createFn(), handler);

		topicCallbackSet->callback.push_back(callback);
	}

	return true;
}

bool DDSTopicManager::unsubscribe(const std::string& topicName,
                                  Handler& handler)
{
	DDSMetadata* metadata = lookupMetadata(topicName);
	if (metadata == 0)
	{
		fprintf(stderr, "# WARNING: Metadata missing for unsubscribe operation.\n");
		return false;
	}

	TopicCallbackSet* topicCallbackSet = metadata->topicCallbackSet;
	if (topicCallbackSet == 0)
	{
		fprintf(stderr, "# WARNING (DDSTopicManager): Topic not registered.\n");
		return false;
	}

	bool done = false;
	for (size_t i = 0; i < topicCallbackSet->callback.size(); ++i)
	{
		if (!handler.empty() || handler == topicCallbackSet->callback[i].getHandler())
		{
			topicCallbackSet->deleteFn(topicCallbackSet->callback[i].getData());

			topicCallbackSet->callback.erase(topicCallbackSet->callback.begin() + i);
			i--;
			done = true;
		}
	}

	if (topicCallbackSet->callback.empty())
	{
		if (metadata->receiver != NULL)
		{
			if (metadata->receiver->reader != NULL)
			{
				if (metadata->cond != NULL)
				{
					mConditionMetadataMap.erase(metadata->cond);
					mWaitsetMutex.lock();
					if (mWaitset != NULL)
					{
						mWaitset->detach_condition(metadata->cond);
					}
					mWaitsetMutex.unlock();
					metadata->cond = NULL;
				}

				DDS_ReturnCode_t retcode =
					mSubscriber->delete_datareader(metadata->receiver->reader);
				if (retcode != DDS_RETCODE_OK)
				{
					fprintf(stderr, "# WARNING: delete_datareader error %d\n", retcode);
					return false;
				}
				metadata->receiver->reader = NULL;
			}

			delete metadata->receiver;
			metadata->receiver = NULL;
		}
	}

	if (!done)
	{
		fprintf(stderr, "# WARNING: Could not find"
				" matching callback for %s\n", topicName.c_str());
	}

	return true;
}

bool DDSTopicManager::unadvertise(const std::string& topicName)
{
	return unregisterPublisher(topicName);
}

bool DDSTopicManager::publish(const std::string& topicName,
                              void* sampleMem)
{
	DDSMetadata* metadata = lookupMetadata(topicName);
	if (metadata != 0)
	{
		metadata->sender->handlerSignal.emit(sampleMem);

		return true;
	}
	else
	{
		fprintf(stderr, "# WARNING: Topic %s is not advertised.\n",
				topicName.c_str());
		return false;
	}
}

bool DDSTopicManager::publish(TopicCallbackSet* topic, void* sampleMem)
{
	return publish(topic->topicName, sampleMem);
}

bool DDSTopicManager::unregisterPublisher(const std::string& topicName)
{
	DDSMetadata* metadata = lookupMetadata(topicName);
	if (metadata == 0)
	{
		return false;
	}

	if (metadata->sender != NULL)
	{
		DDS_ReturnCode_t retcode;
		if (metadata->sender->writer != NULL)
		{
			retcode =
				metadata->sender->publisher->delete_datawriter(metadata->sender->writer);
			if (retcode != DDS_RETCODE_OK)
			{
				fprintf(stderr, "# WARNING: delete_datawriter error %d\n", retcode);
				return false;
			}
			metadata->sender->writer = NULL;
		}

		if (metadata->sender->publisher != NULL)
		{
			retcode = mParticipant->delete_publisher(metadata->sender->publisher);
			if (retcode != DDS_RETCODE_OK)
			{
				fprintf(stderr, "# WARNING: delete_publisher error %d\n", retcode);
				return false;
			}
			metadata->sender->publisher = NULL;
		}

		delete metadata->sender;
		metadata->sender = NULL;
	}

	return true;
}

bool DDSTopicManager::unregisterSubscriber(const std::string& topicName)
{
	DDSMetadata* metadata = lookupMetadata(topicName);
	if (metadata == 0)
	{
		fprintf(stderr, "# WARNING: Metadata missing for unregister subscriber operation.\n");
		return false;
	}

	TopicCallbackSet* topicCallbackSet = metadata->topicCallbackSet;
	if (topicCallbackSet == 0)
	{
		return true;
	}

	for (size_t i = 0; i < topicCallbackSet->callback.size(); ++i)
	{
		topicCallbackSet->deleteFn(topicCallbackSet->callback[i].getData());
	}
	topicCallbackSet->callback.clear();

	if (metadata->receiver != NULL)
	{
		if (metadata->receiver->reader != NULL)
		{
			if (metadata->cond != NULL)
			{
				mConditionMetadataMap.erase(metadata->cond);
				mWaitsetMutex.lock();
				if (mWaitset != NULL)
				{
					mWaitset->detach_condition(metadata->cond);
				}
				mWaitsetMutex.unlock();
				metadata->cond = NULL;
			}

			DDS_ReturnCode_t retcode =
				mSubscriber->delete_datareader(metadata->receiver->reader);
			if (retcode != DDS_RETCODE_OK)
			{
				fprintf(stderr, "# WARNING: delete_datareader error %d\n", retcode);
				return false;
			}
			metadata->receiver->reader = NULL;
		}

		delete metadata->receiver;
		metadata->receiver = NULL;
	}

	return true;
}

DDSTopicManager::DDSMetadata* DDSTopicManager::lookupMetadata(DDSCondition *condition)
{
	ConditionMetadataMap::iterator it;

	it = mConditionMetadataMap.find(condition);
	if (it != mConditionMetadataMap.end())
	{
		return it->second;
	}
	else
	{
		return 0;
	}
}


DDSTopicManager::DDSMetadata* DDSTopicManager::lookupMetadata(const std::string& topicName)
{
	StringMetadataMap::iterator it = mStringMetadataMap.find(topicName);
	if (it != mStringMetadataMap.end())
	{
		return &it->second;
	}
	else
	{
		return 0;
	}
}

TopicCallbackSet* DDSTopicManager::lookupTopicCallbackSet(const std::string& topicName)
{
	StringTopicMap::iterator it = mStringTopicMap.find(topicName);
	if (it != mStringTopicMap.end())
	{
		return it->second;
	}
	else
	{
		return 0;
	}
}

bool DDSTopicManager::log(const std::string& topicName,
                          LogHandler& logHandler, double startTime,
                          FILE *logfile, SubscriptionKind subscribeKind)
{
	TopicCallbackSet* topic = lookupTopicCallbackSet(topicName);
	if (topic == 0)
	{
		fprintf(stderr, "# WARNING: Topic not registered.\n");
		return false;
	}

	DDS_DataReaderQos reader_qos;
	mSubscriber->get_default_datareader_qos(reader_qos);

	if (subscribeKind == SUBSCRIBE_LATEST)
	{
		setPeriodicSubscriberQos(reader_qos, 1);
	}
	else
	{
		setPeriodicSubscriberQos(reader_qos, 100);
	}

	DDS_ReturnCode_t retcode = mSubscriber->set_default_datareader_qos(reader_qos);
	if (retcode != DDS_RETCODE_OK)
	{
		fprintf(stderr, "# WARNING: set_default_datareader_qos error %d\n",
				retcode);
		return false;
	}

	// make sure that the callback isn't already in there
	bool foundCallback = false;
	for (size_t i = 0; i < topic->callback.size(); ++i)
	{
		if (topic->callback[i].getLogHandler() == logHandler)
		{
			foundCallback = true;
			break;
		}
	}

	if (!foundCallback)
	{
		Callback callback(topic->createFn(), logHandler, startTime, logfile);

		topic->callback.push_back(callback);
	}

	return true;
}

void DDSTopicManager::listenThread(bool* quitFlag)
{
	const DDS_Duration_t timeout = { 0, 1000000 }; // 1 msec
	DDSConditionSeq activeConditions; // holder for active conditions

	DDS_ReturnCode_t retcode;

	while (*quitFlag == false)
	{
		mWaitsetMutex.lock();
		retcode = mWaitset->wait(activeConditions, timeout);
		mWaitsetMutex.unlock();

		// if message is received
		if (retcode == DDS_RETCODE_OK)
		{
			for (int i = 0; i < activeConditions.length(); ++i)
			{
				// find metadata associated to received message
				DDSMetadata* metadata = lookupMetadata(activeConditions[i]);

				if (metadata != 0)
				{
					TopicCallbackSet* topic = metadata->topicCallbackSet;

					if (topic != 0)
					{
						// invoke all callbacks associated with topic
						for (size_t j = 0; j < topic->callback.size(); ++j)
						{
							Callback* callback = &(topic->callback[j]);

							if (callback->getData())
							{
								bool validData =
									metadata->receiver->handlerSignal.emit(callback->getData());

								if (validData)
								{
									callback->activate();
								}
							}
						} // end callback for loop
					} // end if topic != 0
				} // end if metadata != 0
				else
				{
					fprintf(stderr, "# WARNING: Received message is not registered.\n");
				}
			}
		}
	}

	DDSConditionSeq attachedConditions;
	mWaitsetMutex.lock();
	retcode = mWaitset->get_conditions(attachedConditions);
	if (retcode != DDS_RETCODE_OK)
	{
		fprintf(stderr, "# WARNING: get_conditions error %d\n", retcode);
		return;
	}

	for (int i = 0; i < attachedConditions.length(); ++i)
	{
		mWaitset->detach_condition(attachedConditions[i]);
	}

	delete mWaitset;
	mWaitset = NULL;
	mWaitsetMutex.unlock();
}

/**
* Sets the DataWriter QoS for an aperiodic, one-at-a-time, strict
* reliable model.
* @param qos the pointer to the DataWriter QoS.
*/
void DDSTopicManager::setAperiodicPublisherQos(DDS_DataWriterQos& qos)
{
	qos.history.depth = 1;

	qos.resource_limits.initial_samples =
		qos.resource_limits.max_samples = 1;
	qos.resource_limits.max_samples_per_instance =
		qos.resource_limits.max_samples;

	// want to piggyback HB w/ every sample
	qos.protocol.rtps_reliable_writer.heartbeats_per_max_samples = 1;

	qos.protocol.rtps_reliable_writer.high_watermark = 1;
	qos.protocol.rtps_reliable_writer.low_watermark = 0;

	// essentially turn off slow HB period
	qos.protocol.rtps_reliable_writer.heartbeat_period.sec = 3600 * 24 * 7;
}

/**
* Sets the DataWriter QoS for a periodic, non-strict reliable model.
* @param qos the pointer to the DataWriter QoS.
*/
void DDSTopicManager::setPeriodicPublisherQos(DDS_DataWriterQos& qos)
{
	qos.history.depth = 20;

	qos.resource_limits.max_samples =
		qos.resource_limits.initial_samples = 20;
	qos.resource_limits.max_samples_per_instance =
		qos.resource_limits.max_samples;

	qos.protocol.rtps_reliable_writer.heartbeats_per_max_samples = 2;

	qos.protocol.rtps_reliable_writer.high_watermark = 16;
	qos.protocol.rtps_reliable_writer.low_watermark = 4;
}

/**
* Sets the DataWriter QoS for a frequent periodic, non-strict reliable model.
* @param qos the pointer to the DataWriter QoS.
*/
void DDSTopicManager::setFrequentPeriodicPublisherQos(DDS_DataWriterQos& qos)
{
	qos.history.depth = 300;

	qos.resource_limits.max_samples =
		qos.resource_limits.initial_samples = 300;
	qos.resource_limits.max_samples_per_instance =
		qos.resource_limits.max_samples;

	qos.protocol.rtps_reliable_writer.heartbeats_per_max_samples = 30;

	qos.protocol.rtps_reliable_writer.high_watermark = 240;
	qos.protocol.rtps_reliable_writer.low_watermark = 60;
}

/**
* Sets the DataReader QoS for an aperiodic, one-at-a-time, strict
* reliable model.
* @param qos the pointer to the DataReader QoS.
*/
void DDSTopicManager::setAperiodicSubscriberQos(DDS_DataReaderQos& qos)
{
	qos.history.depth = 1;

	qos.resource_limits.initial_samples =
		qos.resource_limits.max_samples =
			qos.reader_resource_limits.max_samples_per_remote_writer = 1;
	qos.resource_limits.max_samples_per_instance =
		qos.resource_limits.max_samples;
}

/**
* Sets the DataReader QoS for a periodic, non-strict reliable model.
* @param qos the pointer to the DataReader QoS.
* @param queueSize the size of the queue.
*/
void DDSTopicManager::setPeriodicSubscriberQos(DDS_DataReaderQos& qos,
                                               int queueSize)
{
	qos.history.depth = queueSize;

	qos.resource_limits.initial_samples =
		qos.resource_limits.max_samples =
			qos.reader_resource_limits.max_samples_per_remote_writer =
				queueSize;

	qos.resource_limits.max_samples_per_instance =
		qos.resource_limits.max_samples;
}


bool DDSTopicManager::findTopicServer(TopicCallbackSet* requestTopic,
                                      TopicCallbackSet* responseTopic,
                                      unsigned int timeout_ms)
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	double ts = tv.tv_sec + static_cast<double>(tv.tv_usec) / 1000000.0;
	double scheduled_ts = ts + (timeout_ms / 1000.0);
	DDS_Duration_t sleepPeriod = {0, 1000000};

	if (requestTopic == 0 || responseTopic == 0)
	{
		return false;
	}

	bool requestMatched = false;
	bool responseMatched = false;

	DDSMetadata* requestMetadata = lookupMetadata(requestTopic->topicName);
	DDS_PublicationMatchedStatus pubStatus;

	DDSMetadata* responseMetadata = lookupMetadata(responseTopic->topicName);
	DDS_SubscriptionMatchedStatus subStatus;

	while (ts < scheduled_ts && !(requestMatched && responseMatched))
	{
		requestMetadata->sender->writer->
			get_publication_matched_status(pubStatus);
		if (pubStatus.current_count > 0)
		{
			requestMatched = true;
		}

		responseMetadata->receiver->reader->
			get_subscription_matched_status(subStatus);
		if (subStatus.current_count > 0)
		{
			responseMatched = true;
		}

		NDDSUtility::sleep(sleepPeriod);

		gettimeofday(&tv, NULL);
		ts = tv.tv_sec + static_cast<double>(tv.tv_usec) / 1000000.0;
	}

	return (requestMatched && responseMatched);
}

}
