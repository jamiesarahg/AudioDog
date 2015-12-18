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
import pickle



class DataIntake(object):
  ''' Exracts MFCCs from audio files
  '''
  def __init__(self, emotions, people):


    self.trainingDataX = {}
    self.testingDataX = {}
    self.y = None
    self.sr = None
    self.come = []
    self.modelsDict={}
    self.emotions = emotions
    self.names = []

    self. createNamesList(people)


  def createNamesList(self, people):
    ''' Creates list of names from training data from folders'''
    direction = {'Close': 4, 'Far':4}#, 'Turn':2

    for person in people:
      for key in direction.keys():
        for i in range(1,1+direction[key]):
          self.names.append(person+key+str(i))

  def cleanData(self, filename):
    ''' Cleans data by removing the time before and after the user speaks

        filename: name of audio file
    '''
    #saved code incase we want to use it, but pelase delete in case we don't
    y, sr = librosa.load(filename)

    # remove the first 5000 frames

    # used for debugging the cropping of wav file
    # plot.subplot(2, 1, 1)
    # plot.plot(y)
    # plot.title(filename)


    y=y[5000:] 
    print "y size:", y.size


    #removes the time before the user speaks
    if (y.size == 0):
      print "ERR: SIZE OF Y IS ZERO"
      return y, sr

    for j in range(y.size):
      if y[j]> .18 or y[j]< -.15: #waits for the volume to go over .3
          break #stops checking! Makes sure it doesn't continue to clip

    #removes time after user finishes speaking
    for i in reversed(range(y.size)):
      if y[i]> .18 or y[i] < -.15: #waits for the volume to go over .3, checking in reverse order
        y = y[:i+2000] #crop audio clip to 1000 places after the volume went below .3
        break #stops checking! Makes sure it doesn't continue to clip

    if (y.size != 0):
      y = y[j-1000:] #crop audio clip to 1000 places before the volume was determined to go above .3

    # used for debugging of wav file
    # plot.subplot(2, 1, 2)
    # plot.plot(y)
    # plot.title(str(j+5000)+'/'+str(i+5000))
    # plot.show()

    self.y = y
    self.sr = sr

    return y, sr


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
        print "model:", model
        self.modelsDict[self.emotions[i]] = model
    pickle.dump(self.modelsDict, open( "save.p", "wb" ) )
    return self.modelsDict

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

  def crossValidation(self):
    ''' Runs a cross validation on input sound files. Uses one test sample each time
    '''
    for i in range(len(self.names)):
      for j in range(len(self.names)):
        if i == j:
          self.collectTestingData(self.names[j])
        else:
          self.collectTrainingData(self.names[j])

      self.createModelDictionary()

      for emotion in self.emotions:
        bestEmotion, bestScore = self.predict(self.testingDataX[emotion].transpose())
        print 'result', emotion, bestEmotion, bestScore
      
      self.trainingDataX = {}
      self.testingDataX = {}
      self.modelsDict = {}



def run():
  #emotions that we will be calculating
  emotions = ["come", "stop", "goodBoy", "fetch"]

  #collect training data names from folders
  people = ['jamie', 'susie']
  direction = {'Close': 4, 'Far':4}#, 'Turn':2}



  extract = DataIntake(emotions, people)
  extract.crossValidation()

if __name__ == '__main__':
  run()
