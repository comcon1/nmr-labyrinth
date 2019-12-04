#!/usr/bin/python
'''
Use pkill maze-new for correct termination
'''

PWM_WARMUP_MS = 100
PWM_DC = 30
PWM_REFRESH_PERIOD = 2

import sys
from argparse import ArgumentParser
from threading import Thread, Lock
from serial import Serial
from time import sleep
import signal
import re, random
from time import time, strftime

class Tty(object):
	s = None

	def __init__(self, dev, baud=115200, timeout=5):
		self.s = Serial(port=dev, baudrate=baud, timeout=timeout)
		self.s.read_all()

	def __del__(self):
		if self.s:
			self.s.close()

class Rfid(Tty):
	def __init__(self, dev, baud=115200, timeout=5):
		super(rfid, self).__init__(dev, baud, timeout)
		self.s.write(".ES0\r") # Disable auto shutdown
		self.s.readline()
		self.s.write(".RT05\r") # Set read timeout
		self.s.readline()
		self.s.write(".RB1\r") # Enable read beep
		self.s.readline()

	def read(self):
		self.s.write(".r\r")
		self.s.readline()
		return self.s.readline().strip()

class Gate(Tty):
	def __init__(self, dev, baud=115200, timeout=5, ngpio=8, npwm=4):
		super(Gate, self).__init__(dev, baud, timeout)
		self.ngpio = ngpio
		self.gpio_state = tuple([0 for i in range(ngpio)]) 
                self.gpio_map = 0b00000000
		self.pwm_state = [0 for i in range(npwm)]
                self.pwm_map = 0b0000
		self.s.write("\recho 0\r")
		sleep(1)
		self.s.read_all()

	def pwm(self, i, duty_cycle, warmup_ms=100):
		self.s.write("pwm %d %d %d\r" % (i, warmup_ms, duty_cycle))
		self.pwm_state[i] = 1 if duty_cycle > 0 else 0
                self.pwm_map = (self.pwm_map | 2**i) \
                    if duty_cycle > 0 \
                    else (self.pwm_map & ~2**i)

	def gpio_poll(self, block, binary=False):
		if not block:
			return self.gpio_state
		while 1:
			s = self.s.readline()
			if s == "":
				return self.gpio_map if binary else self.gpio_state
			ss = s.split()
			if ss[0] != "gpio:":
				continue
                        x = int(ss[1], 0x10)
                        self.gpio_map = x % 0x100 # leave only 8 bits
			ret = map(lambda i, x=x: 1 if x & (1 << i) else 0,
					range(self.ngpio))
			self.gpio_state = ret
			return self.gpio_map if binary else self.gpio_state


class StateMachine(object):

    def __init__(self, _gate):
        self.gate = _gate
        self.of = sys.stdout
        
        self.pwm_lock = Lock()
        self.pwm_state = [0,0,0,0]
        self.pwm_refresh()

        self.gpio_state = [0,0,0,0,0,0,0,0]
        self.gpio_map = 0b00000000
        self.close_threads = {} # empty dict for storing threads
        self.close_kills = {}


        self.gpio_kill = False

    def start(self):
        self.gpio_thread = Thread(target=self.gpio_refresh)
        self.gpio_thread.start()

    def stop(self):
        self.of.write('TURNING OFF\n')
        self.of.flush()
        self.gpio_kill = True
        self.gpio_thread.join()
        self.of.write('MAIN killed\n')
        self.of.flush()
        self.pwm_state = [0,0,0,0]
        self.pwm_refresh()
        self.of.write('All doors are opened!\n')
        for i in [0,1]:
            try:
                self.close_kills[0] = True
                self.close_threads[0].join()
                self.of.write('Close %d killed\n' % (i))
                self.of.flush()
            except KeyError:
                self.of.write('skip kill %d\n' % (i))
                self.of.flush()


    def pwm_refresh(self):
        self.pwm_lock.acquire()
        for i in range(len(self.pwm_state)):
                self.gate.pwm(i, self.pwm_state[i]*PWM_DC, \
                        self.pwm_state[i]*PWM_WARMUP_MS)
        self.pwm_lock.release()

    def gpio_refresh(self):
        while True:
            if self.gpio_kill:
                return
            self.gpio_map = self.gate.gpio_poll(1, binary=True)
            self.of.write( strftime("%Y-%m-%d %H:%M:%S  ") + \
                    format(self.gpio_map, '08b') + ' | '+ format(self.gate.pwm_map, '04b')+'\n' )
            self.of.flush()
            if  ((  0b0001 & self.gate.pwm_map) == 0b0000 ) and \
                ( ( 0b00000111 & self.gpio_map ) == 0b00000100):
                self.setCloseLock(0, 1, wtime=2)
            if ( 0b00000011 & self.gpio_map ):
                self.setCloseLock(0, 0)
            if  ((  0b0001 & self.gate.pwm_map) == 0b0001 ) and \
                ( ( 0b00001100 & self.gpio_map ) == 0b00000000):
                self.setCloseLock(1, 1, wtime=4)
            if ( 0b00001100 & self.gpio_map ):
                self.setCloseLock(1, 0)


    def setCloseLock(self, nlock, state, wtime=4):
        self.of.write('Lock %d => %d\n' % (nlock, state))
        self.of.flush()
        if state:
            try:
                __a = self.close_threads[nlock]
                # thread was already started
            except KeyError:
                self.close_kills[nlock] = 0
                self.close_threads[nlock] = Thread(target=self.closeBody, args=(nlock, wtime) )
                self.close_threads[nlock].start()
        else:
            try:
                __a = self.close_threads[nlock]
                self.close_kills[nlock] = 1
                self.close_threads[nlock].join()
                del(self.close_threads[nlock])
            except KeyError:
                pass

    def closeBody(self, nlock, wtime):
        uid = random.randint(0,1e8)
        self.of.write('CLOSE-%d <%d> STARTED\n' % (nlock, uid) )
        self.of.flush()
        wtime  = float(wtime)
        NTICKS = 100
        for i in range(NTICKS):
            sleep(wtime/NTICKS)
            if self.close_kills[nlock]:
                self.of.write('CLOSE-%d <%d> STOPPED\n' % (nlock, uid))
                self.of.flush()
                return

        self.of.write('CLOSE-%d <%d> CLOSING\n' % (nlock, uid) )
        self.of.flush()
        self.pwm_lock.acquire()
        if (nlock == 0):
            self.of.write(' =======000000000000================ \n')
            self.of.flush()
            self.gate.pwm(0, PWM_DC, PWM_WARMUP_MS)
            self.gate.pwm(1,      0,             0)
            self.pwm_lock.release()
        elif (nlock == 1):
            self.gate.pwm(0,      0,             0)
            self.gate.pwm(1, PWM_DC, PWM_WARMUP_MS)
            self.pwm_lock.release()
            self.of.write(' =======111111111111================ \n')
            self.of.flush()
        self.of.write('CLOSE-%d <%d> CLOSED\n' % (nlock, uid))
        self.of.flush()


class ServiceExit(Exception):
    """
    Custom exception which is used to trigger the clean exit
    of all running threads and the main program.
    """
    pass


def service_shutdown(signum, frame):
    print('Caught signal %d' % signum)
    raise ServiceExit


if __name__ == "__main__":
        signal.signal(signal.SIGTERM, service_shutdown)
        signal.signal(signal.SIGINT, service_shutdown)

	p = ArgumentParser(description="maze control")
	p.add_argument('-g', dest="gate", metavar='/dev/ttyX',
				type=str, help='gate interface', required=1)
	p.add_argument('-r', dest="rfid", metavar='/dev/ttyY',
				type=str, help='RFID interface', required=0)
	p.add_argument('-l', dest="log", metavar='FILE',
				type=str, help='log file', default='maze.log')
	a = p.parse_args()

	g = Gate(a.gate)
	r = Rfid(a.rfid) if a.rfid else None

        m = StateMachine(g)
        try:
            m.start()

            while True:
                sleep(0.5)

        except ServiceExit:
            m.stop()

        print 'exiting main'
