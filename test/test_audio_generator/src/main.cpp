#include "ros/ros.h"
#include "audio_msg/AudioBuffer.h"
#include "audio_core/create.h"
#include "audio_core/remove.h"
#include "audio_core/getBufferTime.h"

#include <sstream>
#include <boost/thread/mutex.hpp>

int64_t audioInterface_create(ros::NodeHandle& _n, ros::Time _timeUs, int32_t _freq, const etk::Vector<uint8_t> _channelMap) {
	ros::ServiceClient client = _n.serviceClient<audio_core::create>("create");
	audio_core::create srv;
	srv.request.stamp = _timeUs;
	srv.request.frequency = _freq;
	srv.request.channelMap = _channelMap;
	srv.request.channelFormat = audio_msg::AudioBuffer::FORMAT_INT16;
	if (client.call(srv)) {
		ROS_INFO("uid: %ld", srv.response.handle);
		return srv.response.handle;
	} else {
		ROS_ERROR("Failed to call service create");
		return -1;
	}
}

bool audioInterface_remove(ros::NodeHandle& _n, int64_t _uid) {
	ros::ServiceClient client = _n.serviceClient<audio_core::remove>("remove");
	audio_core::remove srv;
	srv.request.handle = _uid;
	if (client.call(srv)) {
		ROS_INFO("remove uid: %ld", _uid);
		return true;
	} else {
		ROS_ERROR("Failed to call service remove");
		return false;
	}
}

uint32_t audioInterface_getBufferTime(ros::NodeHandle& _n, int64_t _uid) {
	ros::ServiceClient client = _n.serviceClient<audio_core::getBufferTime>("getBufferTime");
	audio_core::getBufferTime srv;
	srv.request.handle = _uid;
	if (client.call(srv)) {
		//ROS_INFO("write need wait time : %d", srv.response.time);
		return srv.response.microseconds;
	}
	ROS_ERROR("Failed to call service getBufferTime");
	return 0;
}

static etk::String p_channelToPlay = "/audio/speaker";
static int32_t p_sampleRate = 48000;
static int32_t p_nbChannels = 2;
static int32_t p_frequency = 440;

static int32_t baseDataSize = 0;
static double phase = 0.0;
etk::Vector<uint8_t> channelMap;
static ros::Publisher stream;

ros::Time nextFrame;

void onTimer(const ros::TimerEvent& _timer) {
	int32_t frameSizeTime = (double(baseDataSize) / (double)p_sampleRate) * 1000000000.0;
	static boost::mutex mutex;
	boost::unique_lock<boost::mutex> lock(mutex);
	if ((nextFrame - ros::Time::now()) > ros::Duration(0, frameSizeTime)) {
		return;
	}
	
	audio_msg::AudioBuffer msg;
	// Basic source name is the curant node handle (it is unique)
	msg.sourceName = ros::NodeHandle("~").getNamespace();
	msg.sourceId = 0;
	// create the Ros timestamp
	msg.header.stamp = nextFrame;
	// set message frequency
	msg.frequency = p_sampleRate;
	// set channel map properties
	msg.channelMap = channelMap;
	// Set the format of flow
	msg.channelFormat = audio_msg::AudioBuffer::FORMAT_INT16;
	
	etk::Vector<int16_t> data;
	data.resize(baseDataSize*channelMap.size());
	double baseCycle = 2.0*M_PI/(double)p_sampleRate * (double)p_frequency;
	for (int32_t iii=0; iii<data.size()/channelMap.size(); iii++) {
		for (int32_t jjj=0; jjj<channelMap.size(); jjj++) {
			data[channelMap.size()*iii+jjj] = cos(phase) * 15000;
		}
		phase += baseCycle;
		if (phase >= 2*M_PI) {
			phase -= 2*M_PI;
		}
	}
	// copy data:
	msg.data.resize(data.size()*sizeof(int16_t));
	memcpy(&msg.data[0], &data[0], data.size()*sizeof(int16_t));
	// publish message
	stream.publish(msg);
	
	nextFrame += ros::Duration(0, frameSizeTime);
	ROS_INFO_STREAM("next frame  " << nextFrame );
}


void usage() {
	ROS_INFO("test_audio_generator usage -c=[channel] -s=[sampleRate] -n=[nbChannels] -f=[frequency] -t=[timeToPlay]:");
	ROS_INFO("	[channel] output to write data (default /audio/speaker)");
	ROS_INFO("	[sampleRate] number of sample per second (default 48000)");
	ROS_INFO("	[nbChannels] nb channel (default 2)");
	ROS_INFO("	[frequency] frequency to generate (default 440)");
	ROS_INFO("ex");
	ROS_INFO("	rosrun test_audio_generator test_audio_generator_node /audio/ 48000 2 440 10");
	exit (-1);
}

int main(int _argc, char **_argv) {
	ros::init(_argc, _argv, "test_audio_generator");
	
	for (int32_t iii=0; iii<_argc ; ++iii) {
		if (strncmp(_argv[iii],"-c=", 3) == 0) {
			p_channelToPlay = &_argv[iii][3];
		} else if (strncmp(_argv[iii],"-s=", 3) == 0) {
			sscanf(&_argv[iii][3], "%d", &p_sampleRate);
		} else if (strncmp(_argv[iii],"-n=", 3) == 0) {
			sscanf(&_argv[iii][3], "%d", &p_nbChannels);
		} else if (strncmp(_argv[iii],"-f=", 3) == 0) {
			sscanf(&_argv[iii][3], "%d", &p_frequency);
		} else if (    strcmp(_argv[iii],"-h") == 0
		            || strcmp(_argv[iii],"--help") == 0) {
			usage();
		}
	}
	ROS_INFO_STREAM("Run with:");
	ROS_INFO_STREAM("    channel='" << p_channelToPlay << "'");
	ROS_INFO_STREAM("    sampleRate=" << p_sampleRate << " Hz");
	ROS_INFO_STREAM("    nbChannels=" << p_nbChannels);
	ROS_INFO_STREAM("    frequency=" << p_frequency << " Hz");
	
	ros::NodeHandle nodeHandlePrivate("~");
	// send data:
	ros::Time timee = ros::Time();
	if (p_sampleRate <= 32000) {
		baseDataSize = 1024*2;
	} else {
		baseDataSize = 1024*8;
	}
	
	
	if (p_nbChannels == 1) {
		channelMap.pushBack(audio_msg::AudioBuffer::CHANNEL_FRONT_CENTER);
	} else if (p_nbChannels == 2) {
		channelMap.pushBack(audio_msg::AudioBuffer::CHANNEL_FRONT_LEFT);
		channelMap.pushBack(audio_msg::AudioBuffer::CHANNEL_FRONT_RIGHT);
	} else if (p_nbChannels == 3) {
		channelMap.pushBack(audio_msg::AudioBuffer::CHANNEL_FRONT_LEFT);
		channelMap.pushBack(audio_msg::AudioBuffer::CHANNEL_FRONT_CENTER);
		channelMap.pushBack(audio_msg::AudioBuffer::CHANNEL_FRONT_RIGHT);
	} else if (p_nbChannels == 4) {
		channelMap.pushBack(audio_msg::AudioBuffer::CHANNEL_FRONT_LEFT);
		channelMap.pushBack(audio_msg::AudioBuffer::CHANNEL_FRONT_RIGHT);
		channelMap.pushBack(audio_msg::AudioBuffer::CHANNEL_REAR_LEFT);
		channelMap.pushBack(audio_msg::AudioBuffer::CHANNEL_REAR_RIGHT);
	} else {
		ROS_ERROR("nb chnnale supported error : %d not in [1,2,3,4]", p_nbChannels);
		exit(-1);
	}
	// new interface: just published data:
	
	nextFrame = ros::Time::now();
	
	ros::NodeHandle nodeHandle;
	// create the output stream:
	stream = nodeHandle.advertise<audio_msg::AudioBuffer>(p_channelToPlay, 10);
	
	
	ros::Timer timer = nodeHandlePrivate.createTimer(ros::Duration(ros::Rate(100)), boost::bind(&onTimer, _1));
	
	ros::spin();
	
	return 0;
}

