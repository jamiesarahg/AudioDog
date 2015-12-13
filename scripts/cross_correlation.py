# http://stackoverflow.com/questions/4688715/find-time-shift-between-two-similar-waveforms
# https://mail.scipy.org/pipermail/scipy-user/2011-December/031177.html

from scipy import signal, fftpack, conj
import numpy

test_1 = numpy.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0])
test_2 = numpy.array([0, 0, 0, 0, 0, 3, 3, 3, 6, 6, 8, 9, 6, 6, 6, 6, 3, 3, 3])
mic_dist = .05 # Distance between microphones in meters
cirrus_sample_rate = 49000

class Runner(object):
	def __init__(self):
		ap = AudioProcessor()
    	lc = Localizer(cirrus_sample_rate, mic_dist)
		# Robot states


		self.done = False
		self.is_distanced = False
		self.angle_correct = False					# True: angled towards src
		
		# Robot speed variables
		self.speed = .1 				# Speed coefficient (0 to 1)
		self.linear = 1 				# Linear speed (-1 to 1)
		self.angular = 0				# Angular Speed (-1 to 1)

		# Robot Data
		self.ranges = []				# Laser Scan Data

		self.settings = termios.tcgetattr(sys.stdin)
		rospy.init_node('audio_dog')

		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		rospy.Subscriber("scan", LaserScan, self.process_scan)

		if DEBUG:
			self.speed = 0

	def process_key(self):
		tty.setraw(sys.stdin.fileno())
		select.select([sys.stdin], [], [], 0)
		key = sys.stdin.read(1)
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
		print ""
		
		# Shut off on "z" press
		if (key == 'z'):
			self.done = True

		# Forward on "w" press
		elif (key == 'w'):
			self.linear = 1

	def calc_error_angular(self):

		# Calculates error_angular - difference in degrees
		# if angle > 180:
		# 	self.error_angular = angle - 270.0
		# else:
		# 	self.error_angular = angle - 90.0

	def orient_angle(self):
		'''
		Determines angular velocity, depending on the robot's
		angle relative to the source. Proportional.
		'''
		self.calc_error_angular()

		# Proportional control - 
		#	Translates from 90 to 0, to 1 to 0
		turn = self.error_angular / 90.0
		self.angular = turn

		twist = Twist()

		twist.linear.x = self.linear * self.speed
		twist.linear.y = 0
		twist.linear.z = 0
		# if distance < wallDistance: postive z 
		twist.angular.x = 0 
		twist.angular.y = 0
		twist.angular.z = self.angular
		self.pub.publish(twist)

	def process_scan(self, scan):
		ranges = []
		for i in range(0, 360):  # Remove redundant 361st value
			ranges.append(scan.ranges[i])
		if ranges != []:
			self.ranges = ranges


class AudioProcessor(object):
	'''
	Calculates the timeshift between two audio signals, and aligns them. 
	'''

	def __init__(self):
		pass

	def process_signals(self, signals):
		timeshifts = self.calculate_timeshift(signals)
		aligned_signals = self.align_signals(signals, timeshifts)

		return abs(timeshifts[0])

	def calculate_timeshift(self, signals):
		'''
		Calculates timeshift (in units, not seconds) by cross-correlating
		the two audio signals.
		'''
		signal_a = signals[0]
		signal_b = signals[1]
		# Cross correlation method
		# Find the max value of the correlations
		m_ab = numpy.argmax(signal.correlate(signal_a, signal_b))
		m_ba = numpy.argmax(signal.correlate(signal_b, signal_a))
		# Convert from correlation max index to signal index
		shift_ab = m_ab - (len(signal_b)-1)
		shift_ba = m_ba - (len(signal_a)-1)
		# print "ab:", signal.correlate(signal_a, signal_b)
		# print "ba:", signal.correlate(signal_b, signal_a)
		# print "max ab: ", m_ab, ", max ba:", m_ba
		# print "shift ab: ", shift_ab,\
		# 	", shift ba:", shift_ba

		return [shift_ab, shift_ba]
		# # FFT Method A
		# print "FFT METHOD A"
		# A = fftpack.fft(signal_a)
		# B = fftpack.fft(signal_b)
		# Ar = -A.conjugate()
		# Br = -B.conjugate()
		# print ( numpy.argmax(numpy.abs(fftpack.ifft(Ar*B))) )
		# print ( numpy.argmax(numpy.abs(fftpack.ifft(A*Br))) )

		# # FFT Method B
		# print "FFT METHOD B"
		# af = fftpack.fft(signal_a)
		# bf = fftpack.fft(signal_b)
		# c = fftpack.ifft(af * conj(bf))

		# time_shift = numpy.argmax(abs(c))
		# print time_shift

	def align_signals(self, signals, timeshifts):
		'''
		Aligns the two audio signals using the timeshifts calculated in
		calculate_timeshift(). Uses either addition method or subtraction
		method:
		 - The addition method adds zeroes to the first signal.
		 - The subtraction method removes "premature" data from the 
		 last signal.
		'''
		print "align_signals"

		# Determine which signal is the lagged signal
		# Addition method
		print "		Addition Method"
		i_first = numpy.argmin(timeshifts)
		lag_diff = timeshifts[i_first]
		lag_buffer = numpy.array([0]*abs(lag_diff))
		first = signals[i_first]
		shifted = numpy.insert(first, lag_buffer, 0)
		print "		", shifted
		print "		", signals[numpy.argmax(timeshifts)]

		# return [shifted, signals[numpy.argmax(timeshifts)]  ]

		# Subtraction method
		print "		Subtraction Method"
		i_last = numpy.argmax(timeshifts)
		lag_diff = timeshifts[i_last]
		last = signals[i_last]

		del_indeces = []
		for t in range(0, lag_diff):
			del_indeces.append(t)
		
		shifted = numpy.delete(last, del_indeces, 0)
		print "		", shifted
		print "		", signals[numpy.argmin(timeshifts)]

		return [shifted, signals[numpy.argmax(timeshifts)]]


class Localizer(object):
	'''
	Locates the origin of the audio source.
	'''
	def __init__(self, samp_rate, mic_dist):
		self.c = 340.29 			# Speed of sound
		self.mic_dist = mic_dist
		self.samp_rate = samp_rate
		self.samp_itrvl = 1.0 / samp_rate


	def calculate_angle(self, timeshift):
		dist = self.samp_intvl*timeshift

		(1/2) * sqrt()
		pass



	# Determine time between samples via sample rate
	# Convert time delay into an angle using speed of sound, distance
	# of microphones, and trig.
		# c = 340.29 m / s

	def run(self):
		r = rospy.Rate(10)
		try:
			while not self.done and not rospy.is_shutdown():

				# Run
				timeshift = self.ap.process_signals([test_1, test_2])
    			self.lc.calculate_angle(timeshift)
    			# to do

				r.sleep()
		except KeyboardInterrupt:
			print "Interrupt."


		# Quit - Set linear and angular speeds to zero.
		twist = Twist()
		twist.linear.x = 0
		twist.linear.y = 0
		twist.linear.z = 0
		# if distance < wallDistance: postive z 
		twist.angular.x = 0 
		twist.angular.y = 0
		twist.angular.z = 0

		self.pub.publish(twist)

	
if __name__ == '__main__':
	node = Runner()
	node.run()
