# http://stackoverflow.com/questions/4688715/find-time-shift-between-two-similar-waveforms
# https://mail.scipy.org/pipermail/scipy-user/2011-December/031177.html

from scipy import signal, fftpack, conj
import numpy

test_1 = numpy.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0])
test_2 = numpy.array([0, 0, 0, 0, 0, 3, 3, 3, 6, 6, 8, 9, 6, 6, 6, 6, 3, 3, 3])


class AudioProcessor(object):
	def __init__(self):
		pass

	def process_signals(self, signals):
		timeshifts = self.calculate_timeshift(signals)
		aligned_signals = self.align_signals(signals, timeshifts)

	def calculate_timeshift(self, signals):
		'''

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


	
if __name__ == '__main__':
    ap = AudioProcessor()
    ap.process_signals([test_1, test_2])
