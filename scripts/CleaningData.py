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

class DataIntake(object):
  def __init__(self):
    self.trainingDataX = []
    self.trainingDataY = []
    self.y_current = None
    self.sr_current = None
    self.emotionMap = {"come":0, "stop":1, "goodBoy":2, "fetch":3}

  def CleanData(self, filename):
    #saved code incase we want to use it, but pelase delete in case we don't
    y, sr = librosa.load(filename)

    #remove the first 5000 frames
    y=y[5000:] 

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
    self.y = y
    self.sr = sr

  def MFCCextraction(self, target, dataType): 
    #calculate MFCCs
    mfcc = librosa.feature.mfcc(y=self.y, sr=self.sr, n_mfcc=13)
    if dataType == 'training':
      self.trainingDataX.append(mfcc)
      self.trainingDataY.append(int(target))

  def PlotMFCC(self):
    #plot results
    plot.subplot(int('41'+n))
    plot.plot(y)
    plot.imshow(mfcc)
    plot.title(emotion)

  def CollectTrainingData(self, name, emotions):
    for emotion in emotions:
      emotNum = self.emotionMap[emotion]
      filename = 'trainingData/'+emotion+'/'+name+'.wav'
      self.CleanData(filename)
      self.MFCCextraction(emotNum, 'training')
    print self.trainingDataX
    print self.trainingDataY

if __name__ == '__main__':
  extract = DataIntake()
  emotions = ["come", "stop", "goodBoy", "fetch"]
  extract.CollectTrainingData('Jamie_Gorson', emotions)
  # plot.figure(1)
  # i=1
  # name = 'Jamie_Gorson'
  

  # plot.figure(2)
  # plot.title('Susie')
  # name = 'Susie_Grimshaw'
  # i=1
  # for emotion in emotions:
  #   filename = 'trainingData/'+emotion+'/'+name+'.wav'
  #   extract.MFCCextraction('Susie_Grimshaw', emotion, str(i))
  #   i+=1
  # plot.show()
