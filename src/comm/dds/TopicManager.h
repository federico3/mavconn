#ifndef TOPICMANAGER_H
#define TOPICMANAGER_H

#include <cassert>
#include <glibmm.h>
#include <string>
#include <sys/time.h>
#include <vector>
#include <tr1/memory>
#include <ndds/pres/pres_typePlugin.h>

namespace px
{

typedef sigc::slot<void, void*> Handler;
typedef sigc::slot<void, void*, double, FILE*> LogHandler;

typedef void* (*TypeCreateFunction)();

typedef bool (*TypeCopyFunction)(void *, void *);

typedef void (*TypeDeleteFunction)(void *);

/**
 * Topic kind.
 */
enum TopicType
{
	TOPIC_PUBLISH_SUBSCRIBE,  /**< Publish-subscribe kind. */
	TOPIC_PUBLISH_SUBSCRIBE_FREQUENT,  /**< Publish-subscribe kind, with high data rate. */
	TOPIC_QUERY_REPLY         /**< Query-reply kind. */
};

/**
 * Subscription kind.
 */
enum SubscriptionKind
{
	UNSUBSCRIBE,       /**< Unsubscribe. */
	SUBSCRIBE_LATEST,  /**< Keep latest message in the queue. */
	SUBSCRIBE_ALL,     /**< Keep latest 100 messages in the queue. */
	SUBSCRIBE_ALL_LONGQUEUELIMIT  /**< Keep latest 1000 messages in the queue. */
};

/**
 * Built-in transport kind.
 */
enum TransportBuiltinKind
{
	TRANSPORTBUILTIN_UDP    = 0x01, /**< Built-in UDP transport. */
	TRANSPORTBUILTIN_SHMEM  = 0x02  /**< Built-in shared memory transport. */
};

typedef long TransportBuiltinKindMask;

/**
 * Specifies which built-in transports are used.
 */
typedef struct
{
	TransportBuiltinKindMask mask; /**< Specifies the built-in transports */
} TransportBuiltinPolicy;

/**
 * Callback structure for receiver.
 */
class Callback
{
public:
	Callback(void* _data, Handler& handler)
	  : data(_data)
	  , handlerType(STANDARD_HANDLER)
	  , logHandler(LogHandler())
	  , startTime(0.0)
	  , logfile(0)
	{
		this->handler = handler;
		handlerSignal.connect(this->handler);
	}

	Callback(void* _data, LogHandler& logHandler, double _startTime,
			 FILE* _logfile)
	  : data(_data)
	  , handlerType(LOG_HANDLER)
	  , handler(Handler())
	  , startTime(_startTime)
	  , logfile(_logfile)
	{
		this->logHandler = logHandler;
		logHandlerSignal.connect(this->logHandler);
	}

	Handler& getHandler(void)
	{
		assert(handlerType == STANDARD_HANDLER);

		return handler;
	}

	LogHandler& getLogHandler(void)
	{
		assert(handlerType == LOG_HANDLER);

		return logHandler;
	}

	void activate(void)
	{
		if (handlerType == STANDARD_HANDLER)
		{
			if (!handler.empty())
			{
				handlerSignal.emit(data);
			}
		}
		else
		{
			if (!logHandler.empty())
			{
				struct timeval tv;
				gettimeofday(&tv, NULL);
				double ts = tv.tv_sec + static_cast<double>(tv.tv_usec) / 1000000.0;

				logHandlerSignal.emit(data, ts - startTime, logfile);
			}
		}
	}

	void* getData(void)
	{
		return data;
	}

private:
	enum HandlerType
	{
		STANDARD_HANDLER,
		LOG_HANDLER
	};

	void* data;

	HandlerType handlerType;
	Handler handler;
	sigc::signal<void, void*> handlerSignal;

	LogHandler logHandler;
	sigc::signal<void, void*, double, FILE*> logHandlerSignal;
	double startTime;
	FILE* logfile;
};

/**
 * Struct encapsulating topic callback information.
 */
struct TopicCallbackSet{
	std::string topicName;
	std::string typeName;
	TopicType topicType;
	TypeCreateFunction createFn;
	TypeCopyFunction copyFn;
	TypeDeleteFunction deleteFn;
	std::vector<Callback> callback;
};

/**
 * Middleware type.
 */
enum MiddlewareType {
	MIDDLEWARE_RTI_DDS  = 0x01  /**< RTI DDS middleware. */
};

typedef long MiddlewareTypeMask;

/**
 * Specifies which middleware are used, and their parameters.
 */
struct MiddlewarePolicy {
	MiddlewareTypeMask mask; /**< Specifies the built-in middleware
								  that are registered automatically
								  when BaseComponent is
								  created. */
	std::vector<unsigned short> ddsDomainIds;    /**< List of DDS domain IDs. */
};

/**
 * ITopicManager class. Functions as an interface class for a topic manager implementation class.
 * All its methods will be called on the implementation class.
 */

template <typename TopicManagerImpl>
class ITopicManager
{
public:
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
			 LogHandler logHandler, double startTime,
			 FILE* logfile, SubscriptionKind subscribeKind);

	TopicCallbackSet* lookupTopicCallbackSet(const std::string& topicName);

	void listenThread(bool* quitFlag);

	/**
	* Sends a request and waits for an associated response.
	* @param request Pointer to request message.
	* @param response Pointer to response message.
	* @param timeout_ms Timeout in milliseconds.
	* @return A boolean value indicating whether the operation is successful.
	*/
	template< typename TRequestTopic,
			typename TResponseTopic >
	bool queryResponse(typename TRequestTopic::data_type& request,
					   typename TResponseTopic::data_type& response,
					   unsigned int timeout_ms);

protected:
	ITopicManager();
	virtual ~ITopicManager();

private:
	bool mHasStarted;

	ITopicManager(const ITopicManager<TopicManagerImpl>&);
	ITopicManager<TopicManagerImpl>& operator=(const ITopicManager<TopicManagerImpl>);
};

// end Class declaration

// Implementation

template<typename TopicManagerImpl>
ITopicManager<TopicManagerImpl>::ITopicManager()
  :mHasStarted(false)
{
}

template<typename TopicManagerImpl>
ITopicManager<TopicManagerImpl>::~ITopicManager()
{
}

template<typename TopicManagerImpl>
bool ITopicManager<TopicManagerImpl>::start(int argc, char** argv,
                                            MiddlewarePolicy& middlewarePolicy)
{
	mHasStarted = static_cast<TopicManagerImpl*>(this)->start(argc, argv, middlewarePolicy);

	return mHasStarted;
}

template<typename TopicManagerImpl>
bool ITopicManager<TopicManagerImpl>::shutdown(void)
{
	if (!mHasStarted)
	{
		fprintf(stderr, "# ERROR: Topic Manager has not yet been started!\n");
		exit(EXIT_FAILURE);
	}

	return static_cast<TopicManagerImpl*>(this)->shutdown();
}

template<typename TopicManagerImpl>
bool ITopicManager<TopicManagerImpl>::listenSingle(const std::string& topicName,
                                                   Handler& handler)
{
	if (!mHasStarted)
	{
		fprintf(stderr, "# ERROR: Topic Manager has not yet been started!\n");
		exit(EXIT_FAILURE);
	}

	return static_cast<TopicManagerImpl*>(this)->listenSingle(topicName, handler);
}

template<typename TopicManagerImpl>
bool ITopicManager<TopicManagerImpl>::subscribe(const std::string& topicName,
                                                Handler& handler,
                                                SubscriptionKind subscribeKind)
{
	if (!mHasStarted)
	{
		fprintf(stderr, "# ERROR: Topic Manager has not yet been started!\n");
		exit(EXIT_FAILURE);
	}

	return static_cast<TopicManagerImpl*>(this)->subscribe(topicName, handler, subscribeKind);
}

template<typename TopicManagerImpl>
bool ITopicManager<TopicManagerImpl>::unsubscribe(const std::string& topicName,
                                                  Handler& handler)
{
	if (!mHasStarted)
	{
		fprintf(stderr, "# ERROR: Topic Manager has not yet been started!\n");
		exit(EXIT_FAILURE);
	}

	return static_cast<TopicManagerImpl*>(this)->unsubscribe(topicName, handler);
}

template<typename TopicManagerImpl>
bool ITopicManager<TopicManagerImpl>::unadvertise(const std::string& topicName)
{
	if (!mHasStarted)
	{
		fprintf(stderr, "# ERROR: Topic Manager has not yet been started!\n");
		exit(EXIT_FAILURE);
	}

	TopicManagerImpl* impl = TopicManagerImpl::getInstance();
	return impl->unadvertise(topicName);
}

template<typename TopicManagerImpl>
bool ITopicManager<TopicManagerImpl>::publish(const std::string& topicName, void* sampleMem)
{
	if (!mHasStarted)
	{
		fprintf(stderr, "# ERROR: Topic Manager has not yet been started!\n");
		exit(EXIT_FAILURE);
	}

	return static_cast<TopicManagerImpl*>(this)->publish(topicName, sampleMem);
}

template<typename TopicManagerImpl>
bool ITopicManager<TopicManagerImpl>::publish(TopicCallbackSet* topic, void* sampleMem)
{
	if (!mHasStarted)
	{
		fprintf(stderr, "# ERROR: Topic Manager has not yet been started!\n");
		exit(EXIT_FAILURE);
	}

	return static_cast<TopicManagerImpl*>(this)->publish(topic, sampleMem);
}

template<typename TopicManagerImpl>
bool ITopicManager<TopicManagerImpl>::unregisterPublisher(const std::string& topicName)
{
	if (!mHasStarted)
	{
		fprintf(stderr, "# ERROR: Topic Manager has not yet been started!\n");
		exit(EXIT_FAILURE);
	}

  return static_cast<TopicManagerImpl*>(this)->unregisterPublisher(topicName);
}

template<typename TopicManagerImpl>
bool ITopicManager<TopicManagerImpl>::unregisterSubscriber(const std::string& topicName)
{
	if (!mHasStarted)
	{
		fprintf(stderr, "# ERROR: Topic Manager has not yet been started!\n");
		exit(EXIT_FAILURE);
	}

	return static_cast<TopicManagerImpl*>(this)->unregisterSubscriber(topicName);
}

template<typename TopicManagerImpl>
bool ITopicManager<TopicManagerImpl>::
	log(const std::string& topicName,
		LogHandler logHandler, double startTime,
		FILE* logfile, SubscriptionKind subscribeKind)
{
	if (!mHasStarted)
	{
		fprintf(stderr, "# ERROR: Topic Manager has not yet been started!\n");
		exit(EXIT_FAILURE);
	}

	return static_cast<TopicManagerImpl*>(this)->log(topicName, logHandler, startTime, logfile, subscribeKind);
}

template<typename TopicManagerImpl>
TopicCallbackSet* ITopicManager<TopicManagerImpl>::lookupTopicCallbackSet(const std::string& topicName)
{
	if (!mHasStarted)
	{
		fprintf(stderr, "# ERROR: Topic Manager has not yet been started!\n");
		exit(EXIT_FAILURE);
	}

	return static_cast<TopicManagerImpl*>(this)->lookupTopicCallbackSet(topicName);
}

template<typename TopicManagerImpl>
void ITopicManager<TopicManagerImpl>::listenThread(bool* quitFlag)
{
	if (!mHasStarted)
	{
		fprintf(stderr, "# ERROR: Topic Manager has not yet been started!\n");
		exit(EXIT_FAILURE);
	}

	static_cast<TopicManagerImpl*>(this)->listenThread(quitFlag);
}

template<typename TopicManagerImpl>
template<typename TTopic>
TopicCallbackSet* ITopicManager<TopicManagerImpl>::
	registerTopic(const TTopic& topicObject, PRESTypePlugin* plugin)
{
	if (!mHasStarted)
	{
		fprintf(stderr, "# ERROR: Topic Manager has not yet been started!\n");
		exit(EXIT_FAILURE);
	}

	return static_cast<TopicManagerImpl*>(this)->registerTopic(topicObject, plugin);
}

template<typename TopicManagerImpl>
template<typename TTopic>
bool ITopicManager<TopicManagerImpl>::
	registerPublisher(const TTopic& topicObject)
{
	if (!mHasStarted)
	{
		fprintf(stderr, "# ERROR: Topic Manager has not yet been started!\n");
		exit(EXIT_FAILURE);
	}

	return static_cast<TopicManagerImpl*>(this)->registerPublisher(topicObject);
}

template<typename TopicManagerImpl>
template<typename TTopic>
bool ITopicManager<TopicManagerImpl>::
	registerSubscriber(const TTopic& topicObject, bool processIncomingMessages)
{
	if (!mHasStarted)
	{
		fprintf(stderr, "# ERROR: Topic Manager has not yet been started!\n");
		exit(EXIT_FAILURE);
	}

	return static_cast<TopicManagerImpl*>(this)->registerSubscriber(topicObject, processIncomingMessages);
}

template<typename TopicManagerImpl>
template<typename TRequestTopic,
         typename TResponseTopic>
bool ITopicManager<TopicManagerImpl>::
	queryResponse(typename TRequestTopic::data_type& request,
                  typename TResponseTopic::data_type& response,
                  unsigned int timeout_ms)
{
	if (!mHasStarted)
	{
		fprintf(stderr, "# ERROR: Topic Manager has not yet been started!\n");
		exit(EXIT_FAILURE);
	}

	return static_cast<TopicManagerImpl*>(this)->queryResponse<TRequestTopic,TResponseTopic>(request, response, timeout_ms);
}

// end Implementation

}

#endif
