#ifndef DDSTOPICMANAGER_H
#define DDSTOPICMANAGER_H

#include <ndds/ndds_cpp.h>

#include "TopicManager.h"

namespace px
{

typedef DDS_ReturnCode_t (*DDSRegisterFunction)(DDSDomainParticipant *, const char *);

/**
 * DDS TopicManager. Uses CRTP (Curiously Recurring Template Pattern).
 */
class DDSTopicManager : public ITopicManager<DDSTopicManager>
{
public:
	static DDSTopicManager* getInstance(void);

	bool start(int argc, char** argv, MiddlewarePolicy& middlewarePolicy);
	bool shutdown(void);

	bool listenSingle(const std::string& topicName, Handler& handler);
	bool subscribe(const std::string& topicName,
				   Handler& handler, SubscriptionKind subscribeKind);
	bool unsubscribe(const std::string& topicName, Handler& handler);

	bool advertise(const std::string& topicName);
	bool unadvertise(const std::string& topicName);

	bool publish(const std::string& topicName, void* sampleMem);
	bool publish(TopicCallbackSet* topic, void* sampleMem);

	template<typename TTopic>
	TopicCallbackSet* registerTopic(const TTopic& topicObject,
									PRESTypePlugin* plugin);

	template<typename TTopic>
	bool registerPublisher(const TTopic& topicObject);
	bool unregisterPublisher(const std::string& topicName);

	template<typename TTopic>
	bool registerSubscriber(const TTopic& topicObject,
						  bool processIncomingMessages);
	bool unregisterSubscriber(const std::string& topicName);

	bool log(const std::string& topicName,
			 LogHandler& logHandler, double startTime,
			 FILE* logfile, SubscriptionKind subscribeKind);

	void listenThread(bool* quitFlag);

	template<typename TQueryTopic,
		   typename TResponseTopic>
	bool queryResponse(typename TQueryTopic::data_type& query,
					  typename TResponseTopic::data_type& response,
					  unsigned int timeout_ms);

	TopicCallbackSet* lookupTopicCallbackSet(const std::string& topicName);

private:
	/**
	* Entity structure for sender.
	*/
	typedef struct
	{
		DDSPublisher* publisher;  /**< Publisher */
		DDSDataWriter* writer;    /**< DataWriter */
		sigc::slot<bool, void*> handler; /**< write handler */
		sigc::signal<bool, void*> handlerSignal; /**< write handler signal */
	} DDSSender;

	/**
	* Entity structure for receiver.
	*/
	typedef struct
	{
		DDSDataReader* reader;   /**< DataReader */
		sigc::slot<bool, void*> handler; /**< take handler */
		sigc::signal<bool, void*> handlerSignal; /**< take handler signal */
	} DDSReceiver;

	struct DDSMetadata
	{
		DDSTopic* topic;
		DDSCondition* cond;
		DDSSender* sender;
		DDSReceiver* receiver;
		TopicCallbackSet* topicCallbackSet;
	};

	DDSTopicManager();
	DDSTopicManager(const DDSTopicManager&);
	DDSTopicManager& operator=(const DDSTopicManager&);

	// DDS Specific methods
	static void setAperiodicPublisherQos(DDS_DataWriterQos& qos);
	static void setPeriodicPublisherQos(DDS_DataWriterQos& qos);
	static void setFrequentPeriodicPublisherQos(DDS_DataWriterQos& qos);
	static void setAperiodicSubscriberQos(DDS_DataReaderQos& qos);
	static void setPeriodicSubscriberQos(DDS_DataReaderQos& qos, int queueSize);

	bool findTopicServer(TopicCallbackSet* queryTopic, TopicCallbackSet* responseTopic,
						 unsigned int timeout_ms);

	DDSMetadata* lookupMetadata(DDSCondition* condition);
	DDSMetadata* lookupMetadata(const std::string& topicName);

	template< typename TData,
			  typename TTypeSupport,
			  typename TDataReader >
	static bool takeSample(void* sample, DDSDataReader* reader);
	template< typename TData,
			  typename TTypeSupport,
			  typename TDataWriter >
	static bool writeSample(void* sample, DDSDataWriter* writer);

	// Data members
	static DDSTopicManager* instance;

	DDSDomainParticipant* mParticipant;
	DDSSubscriber* mSubscriber;
	DDSWaitSet* mWaitset;

	Glib::Mutex mWaitsetMutex;

	typedef std::map<std::string, DDSMetadata> StringMetadataMap;
	typedef std::map<std::string, TopicCallbackSet*> StringTopicMap;
	typedef std::map<DDSCondition*, DDSMetadata*> ConditionMetadataMap;

	StringMetadataMap mStringMetadataMap;
	StringTopicMap mStringTopicMap;
	ConditionMetadataMap mConditionMetadataMap;
}; // end DDSTopicManager class definition

template< typename TTopic >
TopicCallbackSet* DDSTopicManager::registerTopic(const TTopic& topicObject,
                                                 PRESTypePlugin* plugin __attribute__ ((unused)))
{
	typedef typename TTopic::support_type TTypeSupport;

	std::string topicName = topicObject.getName();

	TopicCallbackSet* topicCallbackSet = lookupTopicCallbackSet(topicName);
	if (topicCallbackSet != 0)
	{
		return topicCallbackSet;
	}

	DDS_ReturnCode_t retcode = TTypeSupport::register_type(mParticipant,
														   TTypeSupport::get_type_name());
	if (retcode != DDS_RETCODE_OK)
	{
		fprintf(stderr, "# ERROR: register_type error %d\n", retcode);
		exit(EXIT_FAILURE);
	}

	topicCallbackSet = new TopicCallbackSet;
	topicCallbackSet->topicName = topicName;
	topicCallbackSet->typeName = TTypeSupport::get_type_name();
	topicCallbackSet->topicType = topicObject.getType();
	topicCallbackSet->createFn = (TypeCreateFunction)TTypeSupport::create_data;
	topicCallbackSet->copyFn = (TypeCopyFunction)TTypeSupport::copy_data;
	topicCallbackSet->deleteFn = (TypeDeleteFunction)TTypeSupport::delete_data;

	topicCallbackSet->callback.clear();

	mStringTopicMap.insert(std::pair<std::string,TopicCallbackSet*>(topicName, topicCallbackSet));

	// Create DDS-specific metadata for the topic
	DDSMetadata metadata;
	metadata.cond = NULL;
	metadata.receiver = NULL;
	metadata.sender = NULL;
	metadata.topic = mParticipant->create_topic(topicName.c_str(),
												TTypeSupport::get_type_name(),
												DDS_TOPIC_QOS_DEFAULT, NULL,
												DDS_STATUS_MASK_NONE);
	if (metadata.topic == NULL)
	{
		fprintf(stderr, "# WARNING: Unable to create topic.\n");
		return 0;
	}
	metadata.topicCallbackSet = topicCallbackSet;
	mStringMetadataMap.insert(std::pair<std::string,DDSMetadata>(topicName, metadata));

	return topicCallbackSet;
}

template< typename TData,
          typename TTypeSupport,
          typename TDataReader >
bool DDSTopicManager::takeSample(void* sample, DDSDataReader* reader)
{
	TDataReader* sampleReader = TDataReader::narrow(reader);
	if (sampleReader == NULL)
	{
		fprintf(stderr, "# WARNING: DataReader narrow error\n");
		return false;
	}

	DDS_SampleInfo info;
	DDS_ReturnCode_t retcode = sampleReader->take_next_sample(*(reinterpret_cast<TData*>(sample)), info);

	if (retcode != DDS_RETCODE_OK)
	{
		if (retcode != DDS_RETCODE_NO_DATA)
		{
			fprintf(stderr, "# WARNING: Attempt to read %s sample failed. (error code %d)\n",
					TTypeSupport::get_type_name(), retcode);
		}
		return false;
	}

	return info.valid_data;
}

template< typename TData,
          typename TTypeSupport,
          typename TDataWriter >
bool DDSTopicManager::writeSample(void* sample, DDSDataWriter* writer)
{
	TDataWriter* sampleWriter = TDataWriter::narrow(writer);
	if (sampleWriter == NULL)
	{
		fprintf(stderr, "# WARNING: DataWriter narrow error\n");
		return false;
	}

	DDS_ReturnCode_t retcode = sampleWriter->write(*(reinterpret_cast<TData*>(sample)), DDS_HANDLE_NIL);
	if (retcode != DDS_RETCODE_OK)
	{
		if (retcode != DDS_RETCODE_NO_DATA)
		{
			fprintf(stderr, "# WARNING: Attempt to write %s sample failed. (error code %d)\n",
					TTypeSupport::get_type_name(), retcode);
		}
		return false;
	}

	return true;
}

template<typename TTopic>
bool DDSTopicManager::registerPublisher(const TTopic& topicObject)
{
	std::string topicName = topicObject.getName();

	// Look for DDS metadata information
	DDSMetadata* metadata = lookupMetadata(topicName);
	if (metadata == NULL)
	{
		fprintf(stderr, "# WARNING: Metadata missing for register publisher operation.\n");
		return false;
	}

	// Get callback set corresponding to this topic
	TopicCallbackSet* topicCallbackSet = metadata->topicCallbackSet;
	if (topicCallbackSet == 0)
	{
		fprintf(stderr, "# WARNING: Topic not registered.\n");
		return false;
	}

	if (metadata->sender == NULL)
	{
		DDSPublisher *publisher =
			mParticipant->create_publisher(DDS_PUBLISHER_QOS_DEFAULT,
										   NULL,
										   DDS_STATUS_MASK_NONE);
		if (publisher == NULL)
		{
			fprintf(stderr, "# WARNING: Unable to create publisher.\n");
		}

		DDS_DataWriterQos writer_qos;
		publisher->get_default_datawriter_qos(writer_qos);

		writer_qos.publish_mode.kind = DDS_ASYNCHRONOUS_PUBLISH_MODE_QOS;
		writer_qos.liveliness.lease_duration.sec = 1;
		writer_qos.liveliness.lease_duration.nanosec = 0;

		// enable specified transports
		int numTransports = 0;
		writer_qos.transport_selection.enabled_transports.maximum(0);
		if ((topicObject.getTransportBuiltinPolicy().mask & TRANSPORTBUILTIN_UDP) ==
		   TRANSPORTBUILTIN_UDP)
		{
			numTransports++;
			writer_qos.transport_selection.enabled_transports.maximum(numTransports);
			writer_qos.transport_selection.enabled_transports.length(numTransports);
			writer_qos.transport_selection.enabled_transports[numTransports - 1] =
				DDS_String_dup(DDS_TRANSPORTBUILTIN_UDPv4_ALIAS);
		}

		numTransports++;
		writer_qos.transport_selection.enabled_transports.maximum(numTransports);
		writer_qos.transport_selection.enabled_transports.length(numTransports);
		writer_qos.transport_selection.enabled_transports[numTransports - 1] =
				DDS_String_dup(DDS_TRANSPORTBUILTIN_SHMEM_ALIAS);

		// set publisher QOS
		if (topicCallbackSet->topicType == TOPIC_QUERY_REPLY)
		{
			setAperiodicPublisherQos(writer_qos);
		}
		else if (topicCallbackSet->topicType == TOPIC_PUBLISH_SUBSCRIBE)
		{
			setPeriodicPublisherQos(writer_qos);
		}
		else if (topicCallbackSet->topicType == TOPIC_PUBLISH_SUBSCRIBE_FREQUENT)
		{
			setFrequentPeriodicPublisherQos(writer_qos);
		}
		else
		{
			fprintf(stderr, "# WARNING: Invalid topic type chosen.\n");
			return false;
		}

		// create a datawriter
		DDSDataWriter* writer = publisher->create_datawriter(metadata->topic,
															 writer_qos,
															 NULL,
															 DDS_STATUS_MASK_NONE);
		if (writer == NULL)
		{
			fprintf(stderr, "# WARNING: Unable to create data writer.\n");
			return false;
		}

		typedef typename TTopic::data_type TData;
		typedef typename TTopic::support_type TTypeSupport;
		typedef typename TTopic::data_writer_type TDataWriter;

		metadata->sender = new DDSSender;
		metadata->sender->publisher = publisher;
		metadata->sender->writer = writer;

		metadata->sender->handler = sigc::slot<bool, void*>(sigc::bind(sigc::ptr_fun(&(DDSTopicManager::writeSample<TData, TTypeSupport, TDataWriter>)), writer));
		metadata->sender->handlerSignal.connect(metadata->sender->handler);
	}

	return true;
}

template<typename TTopic>
bool DDSTopicManager::registerSubscriber(const TTopic &topicObject,
                                         bool processIncomingMessages)
{
	std::string topicName = topicObject.getName();

	// Look for DDS metadata information
	DDSMetadata* metadata = lookupMetadata(topicName);
	if (metadata == NULL)
	{
		fprintf(stderr, "# WARNING: Metadata missing for register subscriber operation.\n");
		return false;
	}

	// Get callback set corresponding to this topic
	TopicCallbackSet* topicCallbackSet = metadata->topicCallbackSet;
	if (topicCallbackSet == 0)
	{
		fprintf(stderr, "# WARNING: Topic not registered.\n");
		return false;
	}

	if (metadata->receiver == NULL)
	{
		// Create a datareader
		DDSDataReader* reader =
			mSubscriber->create_datareader(metadata->topic,
										   DDS_DATAREADER_QOS_DEFAULT,
										   NULL,
										   DDS_STATUS_MASK_NONE);
		if (reader == NULL)
		{
			fprintf(stderr, "# WARNING: Unable to create data reader.\n");
			return false;
		}

		typedef typename TTopic::data_type TData;
		typedef typename TTopic::support_type TTypeSupport;
		typedef typename TTopic::data_reader_type TDataReader;

		metadata->receiver = new DDSReceiver;
		metadata->receiver->reader = reader;
		metadata->receiver->handler = sigc::slot<bool, void*>(sigc::bind(sigc::ptr_fun(&(DDSTopicManager::takeSample<TData, TTypeSupport, TDataReader>)), reader));
		metadata->receiver->handlerSignal.connect(metadata->receiver->handler);

		DDSStatusCondition* cond = reader->get_statuscondition();
		cond->set_enabled_statuses(DDS_DATA_AVAILABLE_STATUS);
		metadata->cond = cond;

		if (processIncomingMessages)
		{
			DDS_ReturnCode_t retcode = mWaitset->attach_condition(metadata->cond);
			if (retcode != DDS_RETCODE_OK)
			{
				fprintf(stderr, "# WARNING: Unable to attach condition to waitset.\n");
				return false;
			}
		}

		mConditionMetadataMap.insert(
				std::pair<DDSCondition*, DDSMetadata*>(metadata->cond, metadata));
	}

  return true;
}

template< typename TQueryTopic,
          typename TResponseTopic >
bool DDSTopicManager::queryResponse(typename TQueryTopic::data_type& query,
                                    typename TResponseTopic::data_type& response,
                                    unsigned int timeout_ms)
{
	typedef typename TQueryTopic::data_type TQueryData;
	typedef typename TResponseTopic::data_type TResponseData;

	TopicCallbackSet* queryTopic =
			lookupTopicCallbackSet(TQueryTopic::instance()->getName());
	if (queryTopic == 0)
	{
		if (!TQueryTopic::instance()->advertise())
		{
			return false;
		}
		queryTopic =
			lookupTopicCallbackSet(TQueryTopic::instance()->getName());
	}

	TopicCallbackSet* responseTopic =
			lookupTopicCallbackSet(TResponseTopic::instance()->getName());
	if (responseTopic == 0)
	{
		if (!TResponseTopic::instance()->listenSingle(Handler(NULL), NULL))
		{
			return false;
		}
		responseTopic =
			lookupTopicCallbackSet(TResponseTopic::instance()->getName());
	}

	struct timeval tv;
	gettimeofday(&tv, NULL);
	double ts = tv.tv_sec + static_cast<double>(tv.tv_usec) / 1000000.0;
	double scheduled_ts = ts + static_cast<double>(timeout_ms / 1000.0);

	if (!findTopicServer(queryTopic, responseTopic, timeout_ms))
	{
		return false;
	}

	gettimeofday(&tv, NULL);
	ts = tv.tv_sec + static_cast<double>(tv.tv_usec) / 1000000.0;
	if (ts > scheduled_ts)
	{
		fprintf(stderr, "# WARNING: Out of time for request/response!\n");
		return false;
	}

	// Look for DDS metadata information
	DDSMetadata* responseMetadata = lookupMetadata(responseTopic->topicName);
	if (responseMetadata == NULL)
	{
		fprintf(stderr, "# WARNING: Response metadata missing for request/response.\n");
		return false;
	}

	publish(queryTopic, (void *)&query);

	DDS_Duration_t ddsTimeout = {0,1000000};
	bool receivedResponse = false;
	while (ts < scheduled_ts && !receivedResponse)
	{
		receivedResponse =
			responseMetadata->receiver->handlerSignal.emit(&response);
		NDDSUtility::sleep(ddsTimeout);

		gettimeofday(&tv, NULL);
		ts = tv.tv_sec + static_cast<double>(tv.tv_usec) / 1000000.0;
	}

	return receivedResponse;
}

}

#endif
