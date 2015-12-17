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
    self.trainingDataX = {}
    self.y_current = None
    self.sr_current = None
    self.emotionMap = {}
    self.come = []
    self.modelsDict={}


  # def CreateEmotionMap (self, emotions):
  #   ''' Creates a map from emotions to integers

  #       emotions: list of emotions taken in by user
  #   '''
  #   for i in range(0,len(emotions)): 
  #     self.emotionMap[emotions[i]] = i

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

  def MFCCextraction(self, emotion, dataType): 
    '''  Extracts MFCCs from audio file

          target: emotion being targeted as an int
          dataType: Type of data being analyzed (training or test)
    '''
    #calculate MFCCs
    mfcc = librosa.feature.mfcc(y=self.y, sr=self.sr, n_mfcc=13)
    if dataType == 'training':
      if emotion in self.trainingDataX.keys():
        self.trainingDataX[emotion].append(mfcc)
      else:
        self.trainingDataX[emotion] = mfcc
        print emotion

  def PlotMFCC(self):
    ''' Visualizes the MFCCs 
    '''
    #plot results
    plot.subplot(int('41'+n))
    plot.plot(y)
    plot.imshow(mfcc[0:2])
    plot.title(emotion)


  def CollectTrainingData(self, name, emotions):
    ''' Extracts MFCCs from audio files

        name: name of user speaking
        emotions: list of emotions taken in by user
    '''
    for emotion in emotions:
      # emotNum = self.emotionMap[emotion]
      filename = 'trainingData/'+emotion+'/'+name+'.wav'
      self.CleanData(filename)
      self.MFCCextraction(emotion, 'training')
    print self.trainingDataX


  def CreateModelDictionary(self, emotions):
    for i in range (0,len(emotions)):
      # print emotions[i]
      if emotions[i] not in self.modelsDict.keys():
        model = self.CreateModel(emotions[i])
        self.modelsDict[emotions[i]] = model


  def CreateModel(self, emotion):
    model = GMM().fit(self.trainingDataX[emotion])
    return model

  # def predict(self, emotions, testMFCCs):
  #   bestEmotion = ""
  #   bestScore = 0
  #   for emotion in emotions: 
  #     score = modelsDict[emotion].score(testMFCCs)
  #     if score > bestScore:
  #       bestScore=score
  #       bestEmotion = emotion




if __name__ == '__main__':
  extract = DataIntake()
  emotions = ["come", "stop", "goodBoy", "fetch"]
  # extract.CreateEmotionMap(emotions)
  print extract.emotionMap
  extract.CollectTrainingData('Jamie_Gorson', emotions)
  extract.CreateModelDictionary(emotions)
  # extract.TrainingData()
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
