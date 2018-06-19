

#include <ros/ros.h>
#include "audio_msg/AudioBuffer.h"
#include "audio_core/create.h"
#include "audio_core/remove.h"
#include "audio_core/getBufferTime.h"
#include <audio/river/river.h>
#include <audio/river/Interface.h>
#include <audio/river/Manager.h>
#include <boost/thread.hpp>
#include <sstream>
#include "debug.h"
#include <etk/stdTools.h>
#include <etk/tool.h>

static std11::mutex mutex;


#include "InterfaceInput.h"
#include "InterfaceOutput.h"


std11::shared_ptr<audio::river::Manager> g_manager;

bool f_create(audio_core::create::Request& _req,
              audio_core::create::Response& _res) {
	/*
	std11::shared_ptr<appl::InterfaceOutput> newInterface;
	newInterface = std11::make_shared<appl::InterfaceOutput>(g_manager,
	                                                   _req.flowName,
	                                                   audio::convertFormat(_req.channelFormat),
	                                                   _req.frequency,
	                                                   audio::convertChannel(_req.channelMap));
	if (newInterface == null) {
		_res.handle = -1;
		return false;
	}
	_res.handle = newInterface->getId();
	mutex.lock();
	// TODO : g_listInterafceOut.pushBack(newInterface);
	mutex.unlock();
	APPL_INFO("create : [" << _res.handle << "] type: '" << _req.flowName << "'");
	*/
	return true;
}

bool f_remove(audio_core::remove::Request& _req,
              audio_core::remove::Response& _res) {
	/*
	std11::shared_ptr<appl::InterfaceOutput> interface;
	mutex.lock();
	for(size_t iii=0; iii<g_listInterafceOut.size(); ++iii) {
		if (g_listInterafceOut[iii] == null) {
			continue;
		}
		if (g_listInterafceOut[iii]->getId() == _req.handle) {
			interface = g_listInterafceOut[iii];
			// remove handle in the list
			g_listInterafceOut[iii].reset();
			g_listInterafceOut.erase(g_listInterafceOut.begin() + iii);
			break;
		}
	}
	mutex.unlock();
	if (interface == null) {
		APPL_ERROR("remove : [" << _req.handle << "] Can not remove this ==> already removed.");
		return false;
	}
	// Remove interface :
	APPL_INFO("remove : [" << _req.handle << "] (start)");
	interface.reset();
	APPL_INFO("remove : [" << _req.handle << "] (end)");
	*/
	return true;
}

bool f_getBufferTime(audio_core::getBufferTime::Request& _req,
                     audio_core::getBufferTime::Response& _res) {
	/*
	std11::shared_ptr<appl::InterfaceOutput> interface;
	// reset ouput
	_res.microseconds = 0;
	mutex.lock();
	// Find the handle:
	for(size_t iii=0; iii<g_listInterafceOut.size(); ++iii) {
		if (g_listInterafceOut[iii] == null) {
			continue;
		}
		if (g_listInterafceOut[iii]->getId() == _req.handle) {
			interface = g_listInterafceOut[iii];
			break;
		}
	}
	mutex.unlock();
	if (interface == null) {
		APPL_ERROR("getBufferTime : [" << _req.handle << "] Can not get time ==> handle does not exist...");
		return false;
	}
	_res.microseconds = interface->getTimeBuffer();
	*/
	return true;
}



/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int _argc, char **_argv) {
	ros::init(_argc, _argv, "audio_interface");
	etk::log::setLevel(etk::log::logLevelInfo);
	for (int32_t iii=0; iii<_argc ; ++iii) {
		etk::String data = _argv[iii];
		if (data == "-l0") {
			etk::log::setLevel(etk::log::logLevelNone);
		} else if (data == "-l1") {
			etk::log::setLevel(etk::log::logLevelCritical);
		} else if (data == "-l2") {
			etk::log::setLevel(etk::log::logLevelError);
		} else if (data == "-l3") {
			etk::log::setLevel(etk::log::logLevelWarning);
		} else if (data == "-l4") {
			etk::log::setLevel(etk::log::logLevelInfo);
		} else if (data == "-l5") {
			etk::log::setLevel(etk::log::logLevelDebug);
		} else if (data == "-l6") {
			etk::log::setLevel(etk::log::logLevelVerbose);
		} else if (    data == "-h"
		            || data == "--help") {
			APPL_INFO("Help : ");
			APPL_INFO("    ./xxx [options]");
			APPL_INFO("        -l0: debug None");
			APPL_INFO("        -l1: debug Critical");
			APPL_INFO("        -l2: debug Error");
			APPL_INFO("        -l3: debug Warning");
			APPL_INFO("        -l4: debug Info");
			APPL_INFO("        -l5: debug Debug");
			APPL_INFO("        -l6: debug Verbose");
			APPL_INFO("        -h/--help: this help");
			exit(0);
		}
	}

	etk::initDefaultFolder(_argv[0]);
	etk::setArgZero(_argv[0]);
	for (int32_t iii=0; iii<_argc; ++iii) {
		APPL_ERROR("Argument : " << iii << " '" << _argv[iii] << "'");
	}
	etk::String hardwareInterface="DATA:hardware.json";
	for (int32_t iii=0; iii<_argc; ++iii) {
		etk::String arg = _argv[iii];
		if (etk::start_with(_argv[iii], "--hardware=") == true) {
			hardwareInterface = etk::String(&_argv[iii][11]);
			APPL_INFO("Find hardware configuration ... : '" << hardwareInterface << "'");
		}
	}
	audio::river::init(hardwareInterface);
	
	ros::NodeHandle n;
	
	ros::ServiceServer serviceCreate = n.advertiseService("create", f_create);
	ros::ServiceServer serviceRemove = n.advertiseService("remove", f_remove);
	ros::ServiceServer serviceGetBufferTime = n.advertiseService("getBufferTime", f_getBufferTime);
	
	g_manager = audio::river::Manager::create(n.getNamespace());
	// start publishing of Microphone
	std11::shared_ptr<appl::InterfaceInput> m_input = std11::make_shared<appl::InterfaceInput>(g_manager, "microphone", "microphone", false);
	// start publishing of Speaker feedback
	std11::shared_ptr<appl::InterfaceInput> m_feedback = std11::make_shared<appl::InterfaceInput>(g_manager, "speaker", "feedback/speaker", true);
	// create the Stream for output
	std11::shared_ptr<appl::InterfaceOutput> m_speaker = std11::make_shared<appl::InterfaceOutput>("speaker", "speaker");
	
	/*
	 * A count of how many messages we have sent. This is used to create
	 * a unique string for each message.
	 */
	ros::spin();
	m_input.reset();
	m_feedback.reset();
	g_manager.reset();
	audio::river::unInit();
	
	return 0;
}
