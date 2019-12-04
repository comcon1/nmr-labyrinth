#!/usr/bin/python

maze_state_machine = {
#        Format:
#
#        "TAG: STATE_IN/STATE_OUT < STATE_IN/STATE_OUT": "CMD; CMD; ...",
#
#        TAG: use `state` command to set TAG value
#
#        STATE = "x1d1x2d2_p1p2_R"
#                 ^^^^^^^^ ^^^^ ^
#                 SENSORS  PWM  RFID_READY
#                               optional
#        CMD = "pwmX S" (X=0..3, S=0,1),
#              "rfid",
#              "reset T" (T = seconds),
#              "state TAG"
#
	": / < reset":			"state ENTER; pwm0 0; pwm1 1; pwm2 0; pwm3 1",
	"ENTER: .01._01/ < ..0._01/":	"state IN; pwm0 1",
        "IN: 0.1._11/ < ...._11/":      "state READ; rfid",
	"READ: ...._11_R/ < ...._.1/":	"state LEAV1; pwm1 0",
	"LEAV1: ..00_10/ < ..01_10/":	"state OUT1POS",
	"OUT1POS: ..1._10/ < ..0._10/":	"state LEAV1",
        "OUT1POS: ...._10/1..._.. < ...._10/0..._..": \
                                        "state OUT1SURE; pwm1 1",
        "OUT1SURE: /001._01 < /000._01": "state LEAV2; pwm2 1; pwm3 0",
        "LEAV2:  /0000_10 < /0001_10":   "state OUT2POS; pwm3 1; reset 20",
        "OUT2POS: /.1.._11 < /.0.._11":  "state LEAV2; pwm3 0; reset 0",
        "OUT2POS: /..1._11 < /..0._11":  "state LEAV2; pwm3 0; reset 0",
        "OUT2POS: /...1_11 < /...1_11":  "state LEAV2; pwm3 0; reset 0",
}

PWM_WARMUP_MS = 100
PWM_DC = 30
PWM_REFRESH_PERIOD = 2

import sys
from argparse import ArgumentParser
from threading import Thread, Lock
from serial import Serial
from time import sleep
import re
from time import time, strftime

class tty(object):
	s = None

	def __init__(self, dev, baud=115200, timeout=5):
		self.s = Serial(port=dev, baudrate=baud, timeout=timeout)
		self.s.read_all()

	def __del__(self):
		if self.s:
			self.s.close()

class rfid(tty):
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

class gate(tty):
	def __init__(self, dev, baud=115200, timeout=5, ngpio=8, npwm=4):
		super(gate, self).__init__(dev, baud, timeout)
		self.ngpio = ngpio
		self.gpio_state = tuple([0 for i in range(ngpio)]) 
		self.pwm_state = [0 for i in range(npwm)]
		self.s.write("\recho 0\r")
		sleep(1)
		self.s.read_all()

	def pwm(self, i, duty_cycle, warmup_ms=100):
		self.s.write("pwm %d %d %d\r" % (i, warmup_ms, duty_cycle))
		self.pwm_state[i] = 1 if duty_cycle > 0 else 0

	def gpio_poll(self, block):
		if not block:
			return self.gpio_state
		while 1:
			s = self.s.readline()
			if s == "":
				return self.gpio_state
			ss = s.split()
			if ss[0] != "gpio:":
				continue
			x = int(ss[1], 0x10)
			ret = map(lambda i, x=x: 1 if x & (1 << i) else 0,
					range(self.ngpio))
			self.gpio_state = ret
			return ret

class msm:
	def __init__(self, rules, gate, rfid):
		self.pwm_lock = Lock()
		self.rules = rules
		self.gate = gate
		self.rfid = rfid
		self.state = "reset"
		self._state = "RESET"
		self.reset = 0
		self.gpio_state = [0 for i in range(8)]
		self.rfid_ready = False
		self.rfid_thread = None
		self.rfid_id = ""
		self.gate.s.write("gpio 8 1\r")
		self.update()
		self.run = True
		self.pwm_thread = Thread(target=self.pwm_refresh)
		self.pwm_thread.start()

	def rfid_read(self):
		self.rfid_id = ""
		while self.run and len(self.rfid_id) < 5:
		    print "RFID: start read"
		    self.rfid_id = self.rfid.read().strip()
		self.rfid_ready = True
		print "RFID: id=%s" % self.rfid_id

	def get_state(self, block):
		self.gpio_state = self.gate.gpio_poll(block)
		self.pwm_state = self.gate.pwm_state
		rfid_ready = self.rfid_ready
		if self.rfid_ready:
			self.rfid_ready = False
			if self.rfid_thread:
				self.gate.s.write("gpio 8 1\r")
				self.rfid_thread.join()
				self.rfid_thread = None
				print "RFID: read finished"
		return ("%d%d%d%d" % tuple(self.gpio_state[:4]) +
			"_%d%d" % tuple(self.pwm_state[:2]) +
			("_R" if rfid_ready else "") +
			"/" +
			"%d%d%d%d" % tuple(self.gpio_state[4:]) +
			"_%d%d" % tuple(self.pwm_state[2:]))

	def apply(self, cmd):
		for _c in cmd.split(";"):
			c = _c.strip()
			if c[:3] == "pwm":
				try:
					i = int(c[3])
					dc = int(c.split()[1])
				except:
					print "bad command `%s`" % c
					continue

				self.pwm_lock.acquire()
				self.gate.pwm(i, PWM_DC if dc else 0, PWM_WARMUP_MS if dc else 0)
				self.pwm_lock.release()
				print "pwm %d %d" % (i, dc)
			elif c == "rfid":
				if not self.rfid:
					self.rfid_ready = 1
					continue
				if self.rfid_thread:
					continue
				self.gate.s.write("gpio 8 0\r")
				self.rfid_thread = Thread(target=self.rfid_read)
				self.rfid_thread.start()
				print "RFID thread spawned"

			elif c[:5] == "reset":
				try:
					t = int(c.split()[1])
				except:
					print "bad command `%s`" % c
					continue
				self.reset = time() + t if t > 0 else 0
			elif c[:5] == "state":
				try:
					t = c.split()[1]
					self._state = t
				except:
					print "bad command `%s`" % c
					continue
			else:
				print "unknown command `%s`" % c

	def update(self):
		self.pstate = self.state

		if self.reset and time() >= self.reset:
			self.pstate = "reset"
			self.reset = 0

		self.state = self.get_state(self.pstate != "reset")
		while self.pstate != self.state:
			print "%s: %s: %s -> %s" % (strftime("%Y-%m-%d %H:%M:%S"),
						    self._state,
						    self.pstate, self.state)
			for rule in self.rules.keys():
				s = rule.replace(" ", "")
				t, r = s.split(":")
				if len(t) and self._state != t:
					continue
				n, p = r.split("<")
				if re.search(n, self.state) and re.search(p, self.pstate):
					print "applying `%s` : `%s`" % (rule, self.rules[rule])
					self.apply(self.rules[rule])
			self.pstate = self.state
			self.state = self.get_state(0)
			sys.stdout.flush()

	def pwm_refresh(self):
		while self.run:
			self.pwm_lock.acquire()
			for i in range(len(self.gate.pwm_state)):
				if self.gate.pwm_state[i]:
					self.gate.pwm(i, PWM_DC, PWM_WARMUP_MS)
			self.pwm_lock.release()
			sleep(PWM_REFRESH_PERIOD)

if __name__ == "__main__":
	p = ArgumentParser(description="maze control")
	p.add_argument('-g', dest="gate", metavar='/dev/ttyX',
				type=str, help='gate interface', required=1)
	p.add_argument('-r', dest="rfid", metavar='/dev/ttyY',
				type=str, help='RFID interface', required=0)
	p.add_argument('-l', dest="log", metavar='FILE',
				type=str, help='log file', default='maze.log')
	a = p.parse_args()

	g = gate(a.gate)
	r = rfid(a.rfid) if a.rfid else None
	m = msm(maze_state_machine, g, r)

	try:
		while 1:
			m.update()
	except KeyboardInterrupt:
		print "FINISH"
		m.run = False
		if m.pwm_lock.locked():
			m.pwm_lock.release()
		if m.rfid_thread:
			m.rfid_thread.join()
		m.pwm_thread.join()
