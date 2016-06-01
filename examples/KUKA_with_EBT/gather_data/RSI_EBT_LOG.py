#! /usr/bin/env python

import socket
import sys
import time
import collections
import json
import datetime

data_point = collections.namedtuple('data_point', ['data','time'])


class Listener(object):

    def __init__(self, squawk_string, port, name=''):
        self.bufferSize = 1024
        self.initialized = False
        self.buffer = ''
        self.delim = '\n'
        self.squawk_string = squawk_string
        self.port = port
        if name=='':
            self.name = port
        else:
            self.name = name

    def __del__(self):
        print("[{}] closing shop".format(self.name))
        if self.initialized:
            self.sock.shutdown(socket.SHUT_RDWR)
            self.sock.close()
            print("[{}] port closed".format(self.name))
        else:
            print("[{}] no ports open, closing now".format(self.name))

    def connect(self):
        if not self.initialized:
            try:
                self.sock = socket.socket()
                self.sock.connect(('localhost', self.port))
                self.initialized = True
                print("[{}] Connected to KUKA".format(self.name))
            except:
                 print("[{}] unable to connect, check that the server is running "
                       "at designated IP address and port".format(self.name))
        else:
            print("[{}] already connected".format(self.name))

    def squawk(self):
        self.sock.send("{}\n".format(self.squawk_string))

    def receive(self):
        self.buffer = ''
        while self.buffer.find(self.delim) == -1:
            self.buffer += self.sock.recv(self.bufferSize)
        return data_point(data = self.buffer, time = time.time())


        
class Conductor(object):
    def __init__(self):
        self.ebt = Listener('1004', 1701, 'EBT')
        self.robot = Listener('ping', 6009, 'KUKA')
        self.ebt.connect()
        self.robot.connect()
 	date = datetime.datetime.now()
	file_name = "{}-{}-{}_{}-{}-{}.txt".format(date.year,date.month,date.day,date.hour,date.minute,date.second)
        self.log_file = open(file_name,'w')
        self.log_file.write('EBT Packet Time\ttracking\ttracking time(s)\t'
                            'T[0,0]\tT[0,1]\tT[0,2]\tT[0,3]\tT[1,0]\tT[1,1]\t'
                            'T[1,2]\tT[1,3]\tT[2,0]\tT[2,1]\tT[2,2]\tT[2,3]\t'
                            'T[3,0]\tT[3,1]\tT[3,2]\tT[3,3]\tFrame Time(ms)\t'
                            'KUKA Data Time\tx\ty\tz\ta\tb\tc\n')
                            
    def __del__(self):
        self.log_file.close()
                
    def run(self):
        self.ebt.squawk()
        ebt_data = self.ebt.receive()
        self.robot.squawk()
        robot_data = self.robot.receive()
        self.log(ebt_data, robot_data)
        
    def log(self, ebt_data, robot_data):
        # Robot data parse:
        try:
            robot_json = json.loads(robot_data.data)
        except ValueError as e:
            print("Something wrong loading json with error: {}\n on data: {}".format(e,robot_data.data))
        else:
            # EBT time of data point and data
            self.log_file.write("{:.6f}\t{}\t".format(ebt_data.time,ebt_data.data[2:-2].replace(' ','\t')))
            # Robot data time (time of receipt - age):
            self.log_file.write("{:.6f}\t".format(robot_data.time-(1e-6*float(robot_json['Time']['differential']))))
            for x in ['X','Y','Z','A','B','C']:
                self.log_file.write("{:.6f}\t".format(float(robot_json['Rob']['RIst']['@'+x])))
            self.log_file.write('\n')
        
def spaces2tabs(data):
    result

bob = Conductor()
while True:
    bob.run()
