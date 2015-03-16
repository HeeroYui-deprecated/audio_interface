#include "ros/ros.h"
#include "audio_msg/AudioBuffer.h"
#include "audio_core/create.h"
#include "audio_core/remove.h"
#include "audio_core/write.h"
#include "audio_core/getBufferTime.h"

#include <sstream>

FILE* filee = NULL;
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void audioCallback(const audio_msg::AudioBuffer::ConstPtr& msg) {
	fwrite(&msg->data[0] , sizeof(int16_t), msg->data.size(), filee);
	ROS_INFO("get message: freq=%d nbChannel=%u nbSample=%ld", (int32_t)msg->frequency, (int32_t)msg->channelMap.size(), msg->data.size());
}

int64_t audioInterface_create(ros::NodeHandle& _n, ros::Time _timeUs, int32_t _freq, const std::vector<uint8_t> _channelMap) {
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

/**
 * @brief return wait time
 */
uint32_t audioInterface_write(ros::NodeHandle& _n, int64_t _uid, const std::vector<int16_t>& _value) {
	ros::ServiceClient client = _n.serviceClient<audio_core::write>("write");
	audio_core::write srv;
	srv.request.handle = _uid;
	srv.request.data.resize(_value.size()*2);
	memcpy(&srv.request.data[0], &_value[0], srv.request.data.size());
	if (client.call(srv)) {
		ROS_INFO("write need wait time : %d", srv.response.waitTime);
		return srv.response.waitTime;
	} else {
		ROS_ERROR("Failed to call service write");
		assert(0);
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

void usage() {
	ROS_INFO("test_audio_generator usage [channel] [sampleRate] [nbChannels] [frequency] [timeToPlay]:");
	ROS_INFO("	[channel] output to write data");
	ROS_INFO("	[sampleRate] number of sample per second");
	ROS_INFO("	[nbChannels] nb channel");
	ROS_INFO("	[frequency] frequency to generate");
	ROS_INFO("	[timeToPlay] time to generate signal");
	ROS_INFO("ex");
	ROS_INFO("	rosrun test_audio_generator test_audio_generator_node /audio/ 48000 2 440 10");
	exit (-1);
}

int main(int argc, char **argv) {
	if (argc != 6 ) {
		ROS_ERROR ("not enought argument : %d", argc);
		usage();
	}
	std::string p_channelToPlay = argv[1];
	int32_t p_sampleRate = atoi(argv[2]);
	int32_t p_nbChannels = atoi(argv[3]);
	int32_t p_frequency = atoi(argv[4]);
	int32_t p_timeToPlay = atoi(argv[5]);
	
	sscanf(argv[2], "%d", &p_sampleRate);
	sscanf(argv[3], "%d", &p_nbChannels);
	sscanf(argv[4], "%d", &p_frequency);
	sscanf(argv[5], "%d", &p_timeToPlay);
	
	ROS_INFO("	rosrun test_audio_generator test_audio_generator_node %s %d %d %d %d", p_channelToPlay.c_str(), p_sampleRate, p_nbChannels, p_frequency, p_timeToPlay);
	
	ros::init(argc, argv, "test_audio_generator");
	ros::NodeHandle n;
	double phase = 0;
	// send data:
	ros::Time timee = ros::Time();
	int32_t baseDataSize = 0;
	if (p_sampleRate <= 32000) {
		baseDataSize = 1024*2;
	} else {
		baseDataSize = 1024*8;
	}
	
	std::vector<uint8_t> channelMap;
	if (p_nbChannels == 1) {
		channelMap.push_back(audio_msg::AudioBuffer::CHANNEL_FRONT_CENTER);
	} else if (p_nbChannels == 2) {
		channelMap.push_back(audio_msg::AudioBuffer::CHANNEL_FRONT_LEFT);
		channelMap.push_back(audio_msg::AudioBuffer::CHANNEL_FRONT_RIGHT);
	} else if (p_nbChannels == 3) {
		channelMap.push_back(audio_msg::AudioBuffer::CHANNEL_FRONT_LEFT);
		channelMap.push_back(audio_msg::AudioBuffer::CHANNEL_FRONT_CENTER);
		channelMap.push_back(audio_msg::AudioBuffer::CHANNEL_FRONT_RIGHT);
	} else if (p_nbChannels == 4) {
		channelMap.push_back(audio_msg::AudioBuffer::CHANNEL_FRONT_LEFT);
		channelMap.push_back(audio_msg::AudioBuffer::CHANNEL_FRONT_RIGHT);
		channelMap.push_back(audio_msg::AudioBuffer::CHANNEL_REAR_LEFT);
		channelMap.push_back(audio_msg::AudioBuffer::CHANNEL_REAR_RIGHT);
	} else {
		ROS_ERROR("nb chnnale supported error : %d not in [1,2,3,4]", p_nbChannels);
		exit(-1);
	}
	// connect:
	int64_t uid = audioInterface_create(n, timee, p_sampleRate, channelMap);
	std::vector<int16_t> data;
	data.resize(baseDataSize*channelMap.size());
	
	double baseCycle = 2.0*M_PI/(double)p_sampleRate * (double)p_frequency;
	
	int32_t generateTime = (p_timeToPlay * p_sampleRate) / baseDataSize;
	
	
	for (int32_t kkk=0; kkk<generateTime; ++kkk) {
		for (int32_t iii=0; iii<data.size()/channelMap.size(); iii++) {
			for (int32_t jjj=0; jjj<channelMap.size(); jjj++) {
				data[channelMap.size()*iii+jjj] = cos(phase) * 30000;
			}
			phase += baseCycle;
			if (phase >= 2*M_PI) {
				phase -= 2*M_PI;
			}
		}
		//ROS_INFO("send data");
		int32_t needSleep = audioInterface_write(n, uid, data);
		if (needSleep > 0) {
			ROS_INFO("need sleep %d", needSleep);
			usleep(needSleep);
		} else {
			ROS_INFO("not sleep");
			usleep(needSleep);
		}
	}
	// wait end if playing :
	usleep(200000);
	uint32_t waitTime = audioInterface_getBufferTime(n, uid);
	while (waitTime>0) {
		ROS_INFO("wait end of playing ... %u us", waitTime);
		usleep(waitTime/2);
		waitTime = audioInterface_getBufferTime(n, uid);
	}
	// close:
	audioInterface_remove(n, uid);
	
	return 0;
}
