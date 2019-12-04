#!/usr/bin/python

import sys
from argparse import ArgumentParser
from threading import Thread, Lock
from serial import Serial
from time import sleep
import re
from time import time, strftime

print sys.argv
try:
    assert(len(sys.argv) == 3)
    ndoor = int(sys.argv[1])
    state = int(sys.argv[2])
    assert(state in [0,1])
except:
    print 'Error in params...'
    print './switch-gate.py <num-gate: 0..3> <0,1>'
    sys.exit(1)


s = Serial(port='/dev/ttyACM0', baudrate=115200, timeout=5)
s.write('\recho 1\r')
print 1
s.read_all()

state_str = ' 100 30' if state else ' 0 0'
send_str = 'pwm ' + str(ndoor) + state_str
print 'SENDING COMMAND: '
print send_str
s.write(send_str+'\r')
sleep(2)
print 'DONE'

s.close()

