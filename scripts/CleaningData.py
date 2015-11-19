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

    #remove the first 5000 frames
    y=y[5000:] )

    #removes the time before the user speaks
    for i in range(y.size):
      if y[i]> .3: #waits for the volume to go over .3
        if y[i+500]>.3: #make sure that it's not a blip and stays over .3 for at least 500 frames
          y = y[i-1000:] #crop audio clip to 1000 places before the volume was determined to go above .3
          break #stops checking! Makes sure it doesn't continue to clip

    #removes time after user finishes speaking
    for i in reversed(range(y.size)):
      if y[i]> .3: #waits for the volume to go over .3, checking in reverse order
        y = y[:i+1000] #crop audio clip to 1000 places after the volume went below .3
        break #stops checking! Makes sure it doesn't continue to clip
    
    #calculate MFCCs
    mfcc = librosa.feature.mfcc(y=y, sr=sr, n_mfcc=13)

    #plot results
    plot.subplot(int('41'+n))
    plot.plot(y)
    plot.imshow(mfcc)
    plot.title(emotion)


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
