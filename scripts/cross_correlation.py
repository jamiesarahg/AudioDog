# http://stackoverflow.com/questions/4688715/find-time-shift-between-two-similar-waveforms
# https://mail.scipy.org/pipermail/scipy-user/2011-December/031177.html

from scipy import signal, fftpack, conj
import numpy


test_1 = numpy.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0])
test_2 = numpy.array([0, 0, 0, 0, 0, 3, 3, 3, 6, 6, 8, 9, 6, 6, 6, 6, 3, 3, 3])

def calculate_lag(signal_a, signal_b):
	'''

	'''
	# Cross correlation method
	print "CROSS CORRELATION METHOD"
	print "signal a:", signal_a
	print "signal b:", signal_b
	# Determine which signal came first.
	# Find the max value of the correlations
	m_ab = numpy.argmax(signal.correlate(signal_a, signal_b))
	m_ba = numpy.argmax(signal.correlate(signal_b, signal_a))
	shift_ab = m_ab - (len(signal_b)-1)
	shift_ba = m_ba - (len(signal_a)-1)
	print "ab:", signal.correlate(signal_a, signal_b)
	print "ba:", signal.correlate(signal_b, signal_a)
	print "max ab: ", m_ab, ", max ba:", m_ba


	# onvert
	print "shift ab: ", shift_ab,\
		", shift ba:", shift_ba

	# FFT Method A
	print "FFT METHOD A"
	A = fftpack.fft(signal_a)
	B = fftpack.fft(signal_b)
	Ar = -A.conjugate()
	Br = -B.conjugate()
	print ( numpy.argmax(numpy.abs(fftpack.ifft(Ar*B))) )
	print ( numpy.argmax(numpy.abs(fftpack.ifft(A*Br))) )

	# FFT Method B
	print "FFT METHOD B"
	af = fftpack.fft(signal_a)
	bf = fftpack.fft(signal_b)
	c = fftpack.ifft(af * conj(bf))

	time_shift = numpy.argmax(abs(c))
	print time_shift


	


if __name__ == '__main__':
    calculate_lag(test_1, test_2)
