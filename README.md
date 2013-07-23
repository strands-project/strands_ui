A simple interface for the [MARY TTS system](http://mary.dfki.de/). Offers a service `/ros_mary` and an actionlib interface `/speak` that accepts a simple text argument and plays the audio using PulseAudio.

* launch it: `roslaunch ros_mary_tts ros_mary.launch`
* make the robot speak: `rosservice call /ros_mary 'Welcome to the school of computer science. Please follow me!'`
* in order to use the actionlib cleint you can run `rosrun actionlib axclient.py /speak` 



