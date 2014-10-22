A simple interface for the [MARY TTS system](http://mary.dfki.de/). Offers a service `/ros_mary` and an actionlib interface `/speak` that accepts a simple text argument and plays the audio using PulseAudio.

* launch it: `roslaunch ros_mary_tts ros_mary.launch`
* make the robot speak: `rosservice call /ros_mary 'Welcome to the school of computer science. Please follow me!'`
* in order to use the actionlib cleint you can run `rosrun actionlib axclient.py /speak`
* switching voices:
`rosservice call /ros_mary/set_voice "dfki-obadiah-hsmm"`
* switching languages:
`rosservice call /ros_mary/set_locale "en_US"`

Available languages and voices:
* it
 * None
* te
 * None
* en_US
 * cmu-slt-hsmm (female)
 * dfki-obadiah-hsmm (male)
 * dfki-prudence-hsmm (female)
 * dfki-poppy-hsmm (female)
 * dfki-spike-hsmm (male)
* tr
 * None
* ru
 * None
* de
 * bits1-hsmm (female)
 * bits3-hsmm (male)
 * dfki-pavoque-neutral-hsmm (male)
* sv
 * None

Installing new voices: Use `strands_hri/ros_mary_tts/marytts-5.0/bin/marytts-component-installer.sh`
