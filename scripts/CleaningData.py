import numpy as np
import pyaudio
import wave
import os
import scipy
import librosa
from features import mfcc
from features import logfbank
import scipy.io.wavfile as wav
import matplotlib.pyplot as plot






class Extraction(object):
  def __init__(self):
    pass

  def MFCCextraction(self, name, emotion, n):
    #saved code incase we want to use it, but pelase delete in case we don't
    y, sr = librosa.load('trainingData/'+emotion+'/'+name+'.wav')
    mfcc = librosa.feature.mfcc(y=y, sr=sr, n_mfcc=13)
    plot.subplot(int('41'+n))
    plot.imshow(mfcc)
    plot.title(emotion)

    print mfcc

if __name__ == '__main__':
  extract = Extraction()
  emotions = ["come", "stop", "goodBoy", "fetch"]
  plot.figure(1)
  i=1
  for emotion in emotions:
    extract.MFCCextraction('Jamie_Gorson', emotion, str(i))
    i+=1

  plot.figure(2)
  plot.title('Susie')
  i=1
  for emotion in emotions:
    extract.MFCCextraction('Susie_Grimshaw', emotion, str(i))
    i+=1
  plot.show()


    # def MFCC(self, name, emotion):
    # (rate,sig) = wav.read('trainingData/'+emotion+'/'+name+'.wav')
    # mfcc_feat = mfcc(sig,rate)
    # fbank_feat = logfbank(sig,rate)
    # print mfcc_feat



