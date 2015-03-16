audio_interface
===============

`audio_interface` is a FREE software with Free dependency (BSD or APPACHE-2 or MIT ...).

Instructions
============

download the software :

	mkdir -p catkin_ws/src
	cd catkin_ws/src
	git clone git://github.com/HeeroYui/audio_interface.git
	# download dependency
	git clone git://github.com/HeeroYui/etk.git
	git clone git://github.com/HeeroYui/audio.git
	git clone git://github.com/HeeroYui/ejson.git
	git clone git://github.com/HeeroYui/airtaudio.git
	git clone git://github.com/HeeroYui/drain.git
	git clone git://github.com/HeeroYui/river.git
	cd ../..

Compile software and install :

	cd catkin_ws
	catkin_make install

Run With ROS:
=============

	cd catkin_ws
	source install/setup.bash
	roslaunch audio_bringup bringup_pc.launch

Run the player or the recorder:
===============================

	cd catkin_ws
	source install/setup.bash
	./install/lib/test_audio_generator/test_audio_generator_node /audio/ 48000 2 440 10
	./install/lib/test_audio_recorder/test_audio_recorder_node /audio/microphone recorded.raw

License (APACHE v2.0)
=====================

Copyright ewol Edouard DUPIN

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

