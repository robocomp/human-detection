/*
 *    Copyright (C) 2019 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */


/** \mainpage RoboComp::cameraclient
 *
 * \section intro_sec Introduction
 *
 * The cameraclient component...
 *
 * \section interface_sec Interface
 *
 * interface...
 *
 * \section install_sec Installation
 *
 * \subsection install1_ssec Software depencences
 * ...
 *
 * \subsection install2_ssec Compile and install
 * cd cameraclient
 * <br>
 * cmake . && make
 * <br>
 * To install:
 * <br>
 * sudo make install
 *
 * \section guide_sec User guide
 *
 * \subsection config_ssec Configuration file
 *
 * <p>
 * The configuration file etc/config...
 * </p>
 *
 * \subsection execution_ssec Execution
 *
 * Just: "${PATH_TO_BINARY}/cameraclient --Ice.Config=${PATH_TO_CONFIG_FILE}"
 *
 * \subsection running_ssec Once running
 *
 * ...
 *
 */
#include <signal.h>

// QT includes
#include <QtCore>
#include <QtGui>

// ICE includes
#include <Ice/Ice.h>
#include <IceStorm/IceStorm.h>
#include <Ice/Application.h>

#include <rapplication/rapplication.h>
#include <sigwatch/sigwatch.h>
#include <qlog/qlog.h>

#include "config.h"
#include "genericmonitor.h"
#include "genericworker.h"
#include "specificworker.h"
#include "specificmonitor.h"
#include "commonbehaviorI.h"




// User includes here

// Namespaces
using namespace std;
using namespace RoboCompCommonBehavior;

class cameraclient : public RoboComp::Application
{
public:
	cameraclient (QString prfx) { prefix = prfx.toStdString(); }
private:
	void initialize();
	std::string prefix;
	TuplePrx tprx;

public:
	virtual int run(int, char*[]);
};

void ::cameraclient::initialize()
{
	// Config file properties read example
	// configGetString( PROPERTY_NAME_1, property1_holder, PROPERTY_1_DEFAULT_VALUE );
	// configGetInt( PROPERTY_NAME_2, property1_holder, PROPERTY_2_DEFAULT_VALUE );
}

int ::cameraclient::run(int argc, char* argv[])
{
#ifdef USE_QTGUI
	QApplication a(argc, argv);  // GUI application
#else
	QCoreApplication a(argc, argv);  // NON-GUI application
#endif


	sigset_t sigs;
	sigemptyset(&sigs);
	sigaddset(&sigs, SIGHUP);
	sigaddset(&sigs, SIGINT);
	sigaddset(&sigs, SIGTERM);
	sigprocmask(SIG_UNBLOCK, &sigs, 0);

	UnixSignalWatcher sigwatch;
	sigwatch.watchForSignal(SIGINT);
	sigwatch.watchForSignal(SIGTERM);
	QObject::connect(&sigwatch, SIGNAL(unixSignal(int)), &a, SLOT(quit()));

	int status=EXIT_SUCCESS;

	HumanPosePrxPtr humanpose_pubproxy;
	AprilTagsServerPrxPtr apriltagsserver_proxy;
	PeopleServerPrxPtr peopleserver_proxy;

	string proxy, tmp;
	initialize();


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "AprilTagsServerProxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy AprilTagsServerProxy\n";
		}
		apriltagsserver_proxy = Ice::uncheckedCast<AprilTagsServerPrx>( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy AprilTagsServer: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("AprilTagsServerProxy initialized Ok!");


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "PeopleServerProxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy PeopleServerProxy\n";
		}
		peopleserver_proxy = Ice::uncheckedCast<PeopleServerPrx>( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy PeopleServer: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("PeopleServerProxy initialized Ok!");

	IceStorm::TopicManagerPrxPtr topicManager;
	try
	{
		topicManager = Ice::checkedCast<IceStorm::TopicManagerPrx>(communicator()->propertyToProxy("TopicManager.Proxy"));
	}
	catch (const Ice::Exception &ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception: STORM not running: " << ex << endl;
		return EXIT_FAILURE;
	}
	std::shared_ptr<IceStorm::TopicPrx> humanpose_topic;

	while (!humanpose_topic)
	{
		try
		{
			humanpose_topic = topicManager->retrieve("HumanPose");
		}
		catch (const IceStorm::NoSuchTopic&)
		{
			cout << "[" << PROGRAM_NAME << "]: ERROR retrieving HumanPose topic. \n";
			try
			{
				humanpose_topic = topicManager->create("HumanPose");
			}
			catch (const IceStorm::TopicExists&){
				// Another client created the topic.
				cout << "[" << PROGRAM_NAME << "]: ERROR publishing the HumanPose topic. It's possible that other component have created\n";
			}
		}
	}

	auto humanpose_pub = humanpose_topic->getPublisher()->ice_oneway();
	humanpose_pubproxy = Ice::uncheckedCast<HumanPosePrx>(humanpose_pub);

	tprx = std::make_tuple(apriltagsserver_proxy,peopleserver_proxy,humanpose_pubproxy);
	SpecificWorker *worker = new SpecificWorker(tprx);
	//Monitor thread
	SpecificMonitor *monitor = new SpecificMonitor(worker,communicator());
	QObject::connect(monitor, SIGNAL(kill()), &a, SLOT(quit()));
	QObject::connect(worker, SIGNAL(kill()), &a, SLOT(quit()));
	monitor->start();

	if ( !monitor->isRunning() )
		return status;

	while (!monitor->ready)
	{
		usleep(10000);
	}

	try
	{
		try {
			// Server adapter creation and publication
			if (not GenericMonitor::configGetString(communicator(), prefix, "CommonBehavior.Endpoints", tmp, "")) {
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy CommonBehavior\n";
			}
			Ice::ObjectAdapterPtr adapterCommonBehavior = communicator()->createObjectAdapterWithEndpoints("commonbehavior", tmp);
			auto commonbehaviorI = std::make_shared<CommonBehaviorI>(monitor);
			adapterCommonBehavior->add(commonbehaviorI, Ice::stringToIdentity("commonbehavior"));
			adapterCommonBehavior->activate();
		}
		catch(const Ice::Exception& ex)
		{
			status = EXIT_FAILURE;

			cout << "[" << PROGRAM_NAME << "]: Exception raised while creating CommonBehavior adapter: " << endl;
			cout << ex;

		}




		// Server adapter creation and publication
		cout << SERVER_FULL_NAME " started" << endl;

		// User defined QtGui elements ( main window, dialogs, etc )

		#ifdef USE_QTGUI
			//ignoreInterrupt(); // Uncomment if you want the component to ignore console SIGINT signal (ctrl+c).
			a.setQuitOnLastWindowClosed( true );
		#endif
		// Run QT Application Event Loop
		a.exec();


		status = EXIT_SUCCESS;
	}
	catch(const Ice::Exception& ex)
	{
		status = EXIT_FAILURE;

		cout << "[" << PROGRAM_NAME << "]: Exception raised on main thread: " << endl;
		cout << ex;

	}
	#ifdef USE_QTGUI
		a.quit();
	#endif

	status = EXIT_SUCCESS;
	monitor->terminate();
	monitor->wait();
	delete worker;
	delete monitor;
	return status;
}

int main(int argc, char* argv[])
{
	string arg;

	// Set config file
	std::string configFile = "config";
	if (argc > 1)
	{
		std::string initIC("--Ice.Config=");
		size_t pos = std::string(argv[1]).find(initIC);
		if (pos == 0)
		{
			configFile = std::string(argv[1]+initIC.size());
		}
		else
		{
			configFile = std::string(argv[1]);
		}
	}

	// Search in argument list for --prefix= argument (if exist)
	QString prefix("");
	QString prfx = QString("--prefix=");
	for (int i = 2; i < argc; ++i)
	{
		arg = argv[i];
		if (arg.find(prfx.toStdString(), 0) == 0)
		{
			prefix = QString::fromStdString(arg).remove(0, prfx.size());
			if (prefix.size()>0)
				prefix += QString(".");
			printf("Configuration prefix: <%s>\n", prefix.toStdString().c_str());
		}
	}
	::cameraclient app(prefix);

	return app.main(argc, argv, configFile.c_str());
}
