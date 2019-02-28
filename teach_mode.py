import numpy as np
import time
import serial
import scipy.optimize
import socket
import math
import json
import _thread
from math import pi

thread_flag = False

def wait_for_enter():
    """"""
    global thread_flag
    thread_flag = True
    input()
    print("enter")
    thread_flag = False
    return

class teach():
    """teach mode allows the user to record a sequence by physically moving the robot, playback is also featured"""
    def __init__(self):
        self.teach_name = "test.json"

    def record(self):
        """record a series of moves when manually moving the robot, series recorded as a json with a user input name"""
        input("press enter to start and stop recording")
        global thread_flag
        _thread.start_new_thread(wait_for_enter,())
        sequence = []
        n = 0
        time.sleep(1)
        #print("here0")
        self.socket_send(self.format_prog(30))
        #print("here")
        toc = time.time()
        while(thread_flag == True and n < 3000):
            sequence.append(self.getl())
            n+=1
            time.sleep(0.05)

        self.socket_send(self.format_prog(31))
        tic = time.time()
        sequence.append(tic-toc)
        print("recorded ",tic-toc,"secs")
        self.teach_name = input('Sequence captured\r\nenter name: ')
        self.teach_name+=".json"
        open(self.teach_name, "w").write(json.dumps(sequence))

    def play(self,name = "",t=0.9):
        """play a recording, e.g. name = "test.json", if no name specified, will play the last recording, 
        t for filtering the velocity, high t for fast reponse, low t for smoother playback"""
        if name == "":
            name = self.teach_name
        sequence = json.load(open(name))
        print('Sequence length: ',len(sequence))
        self.movel(sequence[0])
        toc = time.time()
        av_vel = 0
        for i in range(1,len(sequence)-1):
            #av_vel = 0.1*av_vel + t*math.sqrt(math.pow(sequence[i][0]-sequence[i-1][0],2)+math.pow(sequence[i][1]-sequence[i-1][1],2)+math.pow(sequence[i][2]-sequence[i-1][2],2))/0.08
            dist = math.sqrt(math.pow(sequence[i][0]-sequence[i-1][0],2)+math.pow(sequence[i][1]-sequence[i-1][1],2)+math.pow(sequence[i][2]-sequence[i-1][2],2))
            #print(dist)
            if i == len(sequence)-2:
                #print("end")
                av_vel = 0*av_vel + t*math.sqrt(math.pow(sequence[i][0]-sequence[i-1][0],2)+math.pow(sequence[i][1]-sequence[i-1][1],2)+math.pow(sequence[i][2]-sequence[i-1][2],2))/0.1
                self.movep(sequence[i],vel=av_vel,radius=0,wait=True)
            else:
                av_vel = 0*av_vel + t*math.sqrt(math.pow(sequence[i][0]-sequence[i-1][0],2)+math.pow(sequence[i][1]-sequence[i-1][1],2)+math.pow(sequence[i][2]-sequence[i-1][2],2))/0.1
                #print("norm")
                self.movep(sequence[i],vel=av_vel,radius=0.001)
            #else:
                #self.movep(sequence[i-1],vel=av_vel,radius=0.001)
            #    print("too small")
            time.sleep(0.045)
        tic = time.time()
        print("Recorded ",sequence[len(sequence)-1],"secs")
        print("Executed in ",tic-toc,"secs")
        #print(sequence)
