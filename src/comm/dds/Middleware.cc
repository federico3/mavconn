#include "Middleware.h"

namespace px
{

Middleware::Middleware()
 : listenThread(0)
{

}

void Middleware::init(int argc, char **argv)
{
	MiddlewarePolicy middlewarePolicy;
	middlewarePolicy.mask = DEFAULT_MIDDLEWARE_MASK;
	middlewarePolicy.ddsDomainIds.push_back(0);

	if (middlewarePolicy.mask & MIDDLEWARE_RTI_DDS)
	{
		// DDS
		if (middlewarePolicy.ddsDomainIds.size() == 0)
		{
			fprintf(stderr, "# ERROR: No domain IDs specified.\n");
			exit(EXIT_FAILURE);
		}

		int verbosityLevel = 0;
		if (argc > 1)
		{
			for (int i = 1; i < argc; ++i)
			{
				if (strcmp(argv[i], "--verbosity") == 0)
				{
					if (i == argc - 1 || *argv[++i] == '-')
					{
						fprintf(stderr, "# ERROR: Missing <level> after --verbosity\n");
						exit(EXIT_FAILURE);
					}
					verbosityLevel = atoi(argv[i]);
				}
			}
		}

		if (verbosityLevel > 0)
		{
			NDDSConfigLogger::get_instance()->set_verbosity(NDDS_CONFIG_LOG_VERBOSITY_WARNING);
		}
	}

	if (!Glib::thread_supported())
	{
		Glib::thread_init();
	}

	TopicManager* topicManager = TopicManagerFactory::getTopicManager();
	if (!topicManager->start(argc, argv, middlewarePolicy))
	{
		shutdown();
	}

	listenThread = Glib::Thread::create(sigc::bind(sigc::mem_fun(topicManager, &TopicManager::listenThread), &quit), true);

	quit = false;
}

void Middleware::shutdown(void)
{
	// kill message processing thread
	quit = true;
	if (listenThread)
	{
		listenThread->join();
	}

	TopicManager* topicManager = TopicManagerFactory::getTopicManager();
	topicManager->shutdown();

	exit(0);
}

bool Middleware::log(const std::string &topicString,
					 LogHandler& logHandler, double startTime,
					 FILE* logfile, SubscriptionKind subscribeKind)
{
	TopicManager*topicManager = TopicManagerFactory::getTopicManager();

	return topicManager->log(topicString, logHandler, startTime, logfile, subscribeKind);
}

}
