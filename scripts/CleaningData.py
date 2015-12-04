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
  def __init__(self, emotions):
    self.trainingDataX = {}
    self.testingDataX = {}
    self.y = None
    self.sr = None
    self.come = []
    self.modelsDict={}
    self.emotions = emotions


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
        self.trainingDataX[emotion] = np.hstack((self.trainingDataX[emotion], mfcc))
      else:
        self.trainingDataX[emotion] = mfcc
    if dataType == 'testing':
        self.testingDataX[emotion] = mfcc

  def PlotMFCC(self):
    ''' Visualizes the MFCCs 
    '''
    #plot results
    plot.subplot(int('41'+n))
    plot.plot(y)
    plot.imshow(mfcc[0:2])
    plot.title(emotion)


  def CollectTrainingData(self, name):
    ''' Extracts MFCCs from audio files

        name: name of user speaking
        emotions: list of emotions taken in by user
    '''
    for emotion in self.emotions:
      filename = 'trainingData/'+emotion+'/'+name+'.wav'
      self.CleanData(filename)
      self.MFCCextraction(emotion, 'training')

  def CollectTestingData(self, name):
    ''' Extracts MFCCs from audio files

        name: name of user speaking
        emotions: list of emotions taken in by user
    '''
    for emotion in self.emotions:
      filename = 'trainingData/'+emotion+'/'+name+'.wav'
      self.CleanData(filename)
      self.MFCCextraction(emotion, 'testing')

  def CreateModel(self, emotion):

    model =GMM()
    model.fit(self.trainingDataX[emotion].transpose())
    return model

  def CreateModelDictionary(self):
    for i in range(0,len(self.emotions)):
        model = self.CreateModel(self.emotions[i])
        self.modelsDict[self.emotions[i]] = model

  def predict(self, testMFCC):
    bestEmotion = ""
    bestScore = -10000
    for emotion in self.emotions: 
      self.modelsDict[emotion].predict(testMFCC)
      score = self.modelsDict[emotion].score(testMFCC)
      aveScore = sum(score)/len(score)
      if aveScore > bestScore:
        bestScore = aveScore
        bestEmotion = emotion
    return bestEmotion, bestScore

  def crossValidation(self, names):
    for i in range(len(names)):
      for j in range(len(names)):
        if i == j:
          self.CollectTestingData(names[j])
        else:
          self.CollectTrainingData(names[j])

      self.CreateModelDictionary()

      for emotion in self.emotions:
        bestEmotion, bestScore = self.predict(self.testingDataX[emotion].transpose())
        print 'result', emotion, bestEmotion, bestScore
      
      self.trainingDataX = {}
      self.testingDataX = {}
      self.modelsDict = {}



if __name__ == '__main__':
  emotions = ["come", "stop", "goodBoy", "fetch"]
  names = ['Jamie_Gorson', 'Jamie_Gorson1','Jamie_Gorson2', 'Jamie_Gorson3']

  extract = DataIntake(emotions)
  extract.crossValidation(names)
