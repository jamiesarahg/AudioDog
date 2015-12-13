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

  def cleanData(self, filename):
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

  def mfccExtraction(self, emotion, dataType): 
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

  def plotMFCC(self):
    ''' Visualizes the MFCCs 
    '''
    #plot results
    plot.subplot(int('41'+n))
    plot.plot(y)
    plot.imshow(mfcc[0:2])
    plot.title(emotion)


  def collectTrainingData(self, name):
    ''' Extracts MFCCs from audio files

        name: name of user speaking
        emotions: list of emotions taken in by user
    '''
    for emotion in self.emotions:
      filename = 'trainingData/'+emotion+'/'+name+'.wav'
      self.cleanData(filename)
      self.mfccExtraction(emotion, 'training')

  def collectTestingData(self, name):
    ''' Extracts MFCCs from audio files

        name: name of user speaking
        emotions: list of emotions taken in by user
    '''
    for emotion in self.emotions:
      filename = 'trainingData/'+emotion+'/'+name+'.wav'
      self.cleanData(filename)
      self.mfccExtraction(emotion, 'testing')

  def createModel(self, emotion):
    ''' fits GMM model to training data for an emotion

        emotion: corresponding emotion to train to
    '''

    model =GMM()
    model.fit(self.trainingDataX[emotion].transpose())
    return model

  def createModelDictionary(self):
    ''' Builds gausian model for each emotion in self.emotions'''

    for i in range(0,len(self.emotions)):
        model = self.createModel(self.emotions[i])
        self.modelsDict[self.emotions[i]] = model

  def predict(self, testMFCC):
    ''' Predicts emotion from self.emotions of input soundclip

        testMFCC: extracted MFCCs from a short soundclip

        returns: 
          bestEmotion: predicted emotion based on scores for matching of each emotion
          bestScore: the score that predicted that emotion
    '''
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
    ''' Runs a cross validation on input sound files. Uses one test sample each time

        names: list of strings, each the filename of the set of soundclips to be used in the training or testing
    '''
    for i in range(len(names)):
      for j in range(len(names)):
        if i == j:
          self.collectTestingData(names[j])
        else:
          self.collectTrainingData(names[j])

      self.createModelDictionary()

      for emotion in self.emotions:
        bestEmotion, bestScore = self.predict(self.testingDataX[emotion].transpose())
        print 'result', emotion, bestEmotion, bestScore
      
      self.trainingDataX = {}
      self.testingDataX = {}
      self.modelsDict = {}



if __name__ == '__main__':
  emotions = ["come", "stop", "goodBoy", "fetch"]
  names = ['Jamie_Gorson', 'Jamie_Gorson1','Jamie_Gorson2', 'Jamie_Gorson3', 'Susie_Grimshaw', 'Susie_Grimshaw1', 'Susie_Grimshaw2', 'Susan_Grimshaw3']

  extract = DataIntake(emotions)
  extract.crossValidation(names)
