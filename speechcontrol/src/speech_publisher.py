#!/usr/bin/env python
__author__ = 'Chenghao Wang, Matthew Chang'
import roslib
import rospy
import speech_recognition as sr
from nltk.ccg import chart, lexicon
from nltk.ccg.logic import *
from nltk.sem.logic import *
import re
from speechcontrol.msg import array_msg

def speechreco():
    pub = rospy.Publisher("speech", array_msg, queue_size = 10)
    rospy.init_node("speechrobot", anonymous=True)
    rate = rospy.Rate(10)
    hello_str = speech_reco_core()
    rospy.loginfo(hello_str)
    pub.publish(hello_str)
    rate.sleep()




def speech_reco_core():
    with open('/home/crazykoe/turtlebotws/lexicon.txt', 'r') as file:
        myLexicon = file.read()
    lex = lexicon.fromstring(myLexicon, True)
    parser = chart.CCGChartParser(lex, chart.DefaultRuleSet)
    r = sr.Recognizer()
    with sr.Microphone() as source:
        print("What do you need?")
        audio = r.listen(source)
    try:
        print("I think you said " + r.recognize_google(audio) + ". Got it!")
    except sr.UnknownValueError:
        print("Please say it again.")
    except sr.RequestError as e:
        print("The service is down".format(e))

    requestuni = r.recognize_google(audio)
    request = requestuni.encode("utf-8")
    cmd = request
    parses = list(parser.parse(cmd.lower().split()))
    if len(parses) != 0:
        (token, op) = parses[0][()].label()
        if token.semantics() is not None:
            output = str(token.semantics())
            match = re.findall("(?:action\((\w+)\) & target\((\w+)(?:\((\w+)\))?\)(?: &)?)+", output)
            if len(match)==1:
                robotmove = array_msg()
                robotmove.action = match[0][0]
                robotmove.target = match[0][1]
                robotmove.name = match[0][2]
                robotmove.cmdaction = ''
                robotmove.targetroom = ''
                robotmove.names = ''
            else:
                robotmove = array_msg()
                robotmove.action = match[0][0]
                robotmove.target = match[0][1]
                robotmove.name = match[0][2]
                robotmove.cmdaction = match[1][0]
                robotmove.targetroom = match[1][1]
                robotmove.names = match[1][2]
    else:
        print('Unable to parse')
    return(robotmove)
    

if __name__ == '__main__':
    try:
        speechreco()
    except rospy.ROSInterruptException:
        pass
