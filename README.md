# ROS_Speech_Control
Use NLP to extract the keyword of a sentence and publish the command to a turtlebot. This project is done in Ubuntu 18.04 and ROS melodic.

## Requirement
First, you need install the turtlebot3 package in your ROS.

Additionally, these packages need to be installed 
> pip install nltk=x.x 

> pip install SpeechRecognition

> sudo apt-get python-pyaudio

> pip install --upgrade google-cloud-speech


## How to run the code
Download the source code to your own catkin_ws/src and 
> catkin_make

Then launch the turtlebot3_house.launch:
> roslaunch turtlebot3_gazebo turtlebot3_house.launch 

Launch the navigation node of turtlebot:
> roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/YOUR_PATH/maphomeag.yaml

Launch the node of speech_control:
> roslaunch speechcontrol speechrobotcontrol.launch 

## The command sentences:
You can say these sentences in your normal tones.
> Get me a coffee.

> Bring a hamburger to Miller

