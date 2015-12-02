import numpy as np
import pyaudio
import os
import scipy
import librosa
from features import mfcc
from features import logfbank
import scipy.io.wavfile as wav
import matplotlib.pyplot as plot
from sklearn.mixture import GMM

class DataIntake(object):
  ''' Exracts MFCCs from audio files
  '''
  def __init__(self):
    pass


  def CleanData(self, filename):
    ''' Cleans data by removing the time before and after the user speaks

        filename: name of audio file
    '''
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

  def MFCCextraction(self): 
    '''  Extracts MFCCs from audio file

          target: emotion being targeted as an int
          dataType: Type of data being analyzed (training or test)
    '''
    #calculate MFCCs
    mfcc = librosa.feature.mfcc(y=self.y, sr=self.sr, n_mfcc=13)
    return mfcc


  def PlotMFCC(self):
    ''' Visualizes the MFCCs 
    '''
    #plot results
    plot.subplot(int('41'+n))
    plot.plot(y)
    plot.imshow(mfcc[0:2])
    plot.title(emotion)




  def CollectTrainingData(self, filename):
    ''' Extracts MFCCs from audio files

        name: name of user speaking
        emotions: list of emotions taken in by user
    '''
    self.CleanData(filename)
    mfcc = self.MFCCextraction()
    return mfcc



if __name__ == '__main__':
  extract = DataIntake()
  filename = 'trainingData/come/Jamie_Gorson.wav'
  extract.CollectTrainingData(filename)
