# http://stackoverflow.com/questions/4688715/find-time-shift-between-two-similar-waveforms
# https://mail.scipy.org/pipermail/scipy-user/2011-December/031177.html

from scipy import signal, fftpack, conj
from scipy.io import wavfile
import numpy
import math

from scikits.audiolab import wavread

DEBUG = True
test = 1;

#  DEBUGGING VARIABLES
test_1 = numpy.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 0, 0])
test_2 = numpy.array([0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 6, 6, 8, 9, 6, 6, 6, 6, 3, 3, 3])
mic_dist = .30 # Distance between microphones in meters

import wave
import struct


class WaveReader(object):
	def __init__(self, num_chunks):
		self.num_chunks = num_chunks
		self.samp_rate = 0;
		self.num_frames = 0;

	def pcm_channels(self, wave_file):
		"""Given a file-like object or file path representing a wave file,
			decompose it into its constituent PCM data streams.

			Input: A file like object or file path
			Output: A list of lists of integers representing the PCM coded data stream channels
				and the sample rate of the channels (mixed rate channels not supported)
		"""
		chunked_audio = []
		ch_0 = []
		ch_1 = []
		stream = wave.open(wave_file,"rb")

		self.num_channels = stream.getnchannels()
		self.samp_rate = stream.getframerate()
		self.sample_width = stream.getsampwidth()
		self.num_frames = stream.getnframes()

		raw_data = stream.readframes( self.num_frames ) # Returns byte data
		stream.close()

		total_samples = self.num_frames * self.num_channels

		print "sample_width: ", self.sample_width
		if self.sample_width == 1: 
			fmt = "%iB" % total_samples # read unsigned chars
		elif self.sample_width == 2:
			fmt = "%if" % (total_samples/2) # read signed 2 byte shorts
		else:
			raise ValueError("Only supports 8 and 16 bit audio formats.")

		integer_data = struct.unpack(fmt, raw_data)
		del raw_data # Keep memory tidy (who knows how big it might be)

		channels = [ [] for time in range(self.num_channels) ]

		for index, value in enumerate(integer_data):
			bucket = index % self.num_channels
			channels[bucket].append(value)

		# return channels
		raw_0 = channels[0]
		raw_1 = channels[1]

		self.num_frames = len(raw_0)
		self.duration = self.num_frames/self.samp_rate

		chunk_len = self.num_frames / self.num_chunks;
		for c in range(0, self.num_chunks):
			chunk = []

			start = c*chunk_len
			end = (c+1)*chunk_len

			chunk_ch_0 = raw_0[start:end]
			chunk_ch_1 = raw_1[start:end]

			ch_0.append(chunk_ch_0)
			ch_1.append(chunk_ch_1)

			chunk = [chunk_ch_0, chunk_ch_1]
			chunked_audio.append(chunk)



		return [self.samp_rate, chunked_audio]


	def read(self, filename):
		rate = 0;
		chunked_audio = []
		ch_0 = []
		ch_1 = []

		print "reading ", filename
		[rate, w] = wavfile.read(filename)
		print "		rate:", rate
		# print "		result:", w
		raw_0 = w[:, 0]
		raw_1 = w[:, 1]
		# print "raw_0:", raw_0
		# print "raw_1:", raw_1

		self.samp_rate = rate
		self.num_frames = len(raw_0)
		self.duration = self.num_frames/rate

		chunk_len = self.num_frames / self.num_chunks;
		for c in range(0, self.num_chunks):
			chunk = []

			start = c*chunk_len
			end = (c+1)*chunk_len

			chunk_ch_0 = raw_0[start:end]
			chunk_ch_1 = raw_1[start:end]

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

		# print timeshifts
		return timeshifts

	def calculate_timeshift(self, signals):
		'''
		Calculates timeshift (in units, not seconds) by cross-correlating
		the two audio signals.
		'''
		signal_a = signals[0]
		signal_b = signals[1]

		# if (test == 1):
		# 	for i in range(0, len(signal_a)):
		# 		print signal_a[i],", ", signal_b[i]
		# 	global test
		# 	test += 1



		# print "len a:", len(signal_a)
		# print "len b:", len(signal_b)
		# print signal_a
		# print signal_b

		# # Cross correlation method
		# Find the max value of the correlations
		# m_ab = numpy.argmax(signal.correlate(signal_a, signal_b))
		# m_ba = numpy.argmax(signal.correlate(signal_b, signal_a))

		# # Convert from correlation max index to signal index
		# shift_ab = m_ab - (len(signal_b)-1)
		# shift_ba = m_ba - (len(signal_a)-1)
		# # return [shift_ab, shift_ba]

		# print "crosscorr shifts:", [shift_ab, shift_ba]
		# shift = (shift_ba + (-1*shift_ab)) / 2
		# # print "shift:", shift
		# return shift

		# FFT Method A ----------------------------------
		# print "FFT METHOD A"
		A = fftpack.fft(signal_a)
		B = fftpack.fft(signal_b)
		Ar = -A.conjugate()
		Br = -B.conjugate()
		maxA = ( numpy.argmax(numpy.abs(fftpack.ifft(Ar*B))) )
		maxB = ( numpy.argmax(numpy.abs(fftpack.ifft(A*Br))) )
		shifts = [maxA, maxB]
		print "FFT shift:", shifts
		minshift = numpy.argmin(shifts)
		timeshift = shifts[minshift]

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
		print "		tdoa:", tdoa
		print "		signal distance:", sig_dist

		if (sig_dist != 0):
			angle = math.atan( 
				math.sqrt( self.mic_dist**2 - sig_dist**2 ) / sig_dist )
		return angle


	# Determine time between samples via sample rate
	# Convert time delay into an angle using speed of sound, distance
	# of microphones, and trig.
		# c = 340.29 m / s

def run():
	filename = "test_sample.wav" 
	num_chunks = 20
	chunked_audio = []

	

	wr = WaveReader(num_chunks)
	[rate, chunked_audio] = wr.pcm_channels(filename)
	# [rate, chunked_audio] = wr.read(filename)
	print "rate:", rate
	# print "chunked_audio: ", chunked_audio

	ap = AudioProcessor(num_chunks, chunked_audio)
	lc = Localizer(rate, mic_dist)

	timeshifts = ap.process_signals()
	# angle = lc.calculate_angle(timeshifts)
	# print "			Angle: ", angle
	# return angle * 1000

	
if __name__ == '__main__':
	run()