# http://stackoverflow.com/questions/4688715/find-time-shift-between-two-similar-waveforms
# https://mail.scipy.org/pipermail/scipy-user/2011-December/031177.html

from scipy import signal, fftpack, conj
from scipy.io import wavfile
import numpy
import math

from scikits.audiolab import wavread

DEBUG = True

#  DEBUGGING VARIABLES
test_1 = numpy.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 0, 0])
test_2 = numpy.array([0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 6, 6, 8, 9, 6, 6, 6, 6, 3, 3, 3])
mic_dist = .05 # Distance between microphones in meters

class WaveReader(object):
	def __init__(self, num_chunks):
		self.num_chunks = num_chunks
		self.samp_rate = 8000;
		self.num_frames = 0;

	def read(self, filename):
		rate = 0;
		chunked_audio = []
		ch_0 = []
		ch_1 = []

		print "reading ", filename
		[rate, w] = wavfile.read(filename)
		print "		rate:", rate
		print "		result:", w
		raw_0 = w[:, 0]
		raw_1 = w[:, 1]
		print "raw_0:", raw_0
		print "raw_1:", raw_1

		self.samp_rate = rate
		self.num_frames = len(raw_0)
		self.duration = self.num_frames/rate

		chunk_len = self.num_frames / self.num_chunks;
		for c in range(0, self.num_chunks):
			chunk = []

			start = c*chunk_len
			end = (c+1)*chunk_len

			chunk_ch_0 = raw_0[start:end]
			chunk_ch_1 = raw_0[start:end]

			ch_0.append(chunk_ch_0)
			ch_1.append(chunk_ch_1)

			chunk = [chunk_ch_0, chunk_ch_1]
			chunked_audio.append(chunk)


		# print "scipy rate:", rate
		# print "scipy len:", self.num_frames
		# print "chunk_len:", chunk_len
		# print "duration:", self.duration
		
		# print "scipy ch0 :", raw_0
		# print "scipy ch1 :", raw_1

		# print "scipy ch_0 :", ch_0
		# print "scipy ch_1 :", ch_1
		# print "chunked_audio:", chunked_audio

		return [rate, chunked_audio]



class AudioProcessor(object):
	'''
	Calculates the timeshift between two audio signals, and aligns them. 
	'''

	def __init__(self, num_chunks, chunked_audio):
		self.num_chunks = num_chunks;
		self.chunked_audio = chunked_audio

	def process_signals(self):
		timeshifts = [] 			# timeshifts

		for c in range(0, self.num_chunks):
			# print "c:", c
			thischunk = self.chunked_audio[c] 
			# thischunk = [test_1, test_2]
			timeshift = self.calculate_timeshift(thischunk)
			# print "timeshift:", timeshift

			timeshifts.append(timeshift)

		print timeshifts
		return timeshifts

	def calculate_timeshift(self, signals):
		'''
		Calculates timeshift (in units, not seconds) by cross-correlating
		the two audio signals.
		'''
		signal_a = signals[0]
		signal_b = signals[1]

		# print "len a:", len(signal_a)
		# print "len b:", len(signal_b)
		# print signal_a
		# print signal_b

		# # Cross correlation method
		# # Find the max value of the correlations
		# m_ab = numpy.argmax(signal.correlate(signal_a, signal_b))
		# m_ba = numpy.argmax(signal.correlate(signal_b, signal_a))

		# # Convert from correlation max index to signal index
		# shift_ab = m_ab - (len(signal_b)-1)
		# shift_ba = m_ba - (len(signal_a)-1)
		# # return [shift_ab, shift_ba]

		# shift = (shift_ba + (-1*shift_ab)) / 2
		# print "shift:", shift
		# # return shift

		# # FFT Method A ----------------------------------
		# print "FFT METHOD A"
		A = fftpack.fft(signal_a)
		B = fftpack.fft(signal_b)
		Ar = -A.conjugate()
		Br = -B.conjugate()
		maxA = ( numpy.argmax(numpy.abs(fftpack.ifft(Ar*B))) )
		maxB = ( numpy.argmax(numpy.abs(fftpack.ifft(A*Br))) )
		shifts = [maxA, maxB]
		print "maxes:", maxA, ", ", maxB
		minshift = numpy.argmin(shifts)
		timeshift = shifts[minshift]

		# # FFT Method B
		# print "FFT METHOD B"
		# af = fftpack.fft(signal_a)
		# bf = fftpack.fft(signal_b)
		# ca = fftpack.ifft(af * conj(bf))
		# cb = fftpack.ifft(bf * conj(af))

		# time_shifta = numpy.argmax(abs(ca))
		# time_shiftb = numpy.argmax(abs(cb))
		# print time_shifta, ",", time_shiftb

		return timeshift

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
		self.samp_intvl = 1.0 / samp_rate


	def calculate_angle(self, timeshifts):
		# take mode
		angle = 0;
		timeshift = numpy.mean(timeshifts)

		tdoa = self.samp_intvl * timeshift # Time difference of arrival
		sig_dist = tdoa * self.c
		# print "		tdoa:", tdoa
		# print "		signal distance:", sig_dist

		if (sig_dist != 0):
			angle = math.atan( 
				math.sqrt( self.mic_dist**2 - sig_dist**2 ) / sig_dist )
		return angle


	# Determine time between samples via sample rate
	# Convert time delay into an angle using speed of sound, distance
	# of microphones, and trig.
		# c = 340.29 m / s

def run():
	filename = "../wav/sample.wav"
	num_chunks = 20
	chunked_audio = []

	wr = WaveReader(num_chunks)
	[rate, chunked_audio] = wr.read(filename)
	# print "chunked_audio: ", chunked_audio

	ap = AudioProcessor(num_chunks, chunked_audio)
	lc = Localizer(rate, mic_dist)

	timeshifts = ap.process_signals()
	angle = lc.calculate_angle(timeshifts)
	print "			Angle: ", angle
	return angle * 1000

	
if __name__ == '__main__':
	run()