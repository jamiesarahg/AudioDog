import numpy as np
import os
import scipy
from sklearn.mixture import GMM
import CleaningData as cd

class Prediction(object):
	def __init__(self):
		self.testMFCCs=[]

	def PrepData (self, filename):
		data = cd.DataIntake()
		
		self.testMFCCs = data.CollectTrainingData(filename)

  def predict(self, emotions, modelsDict):
    bestEmotion = ""
    bestScore = 0
    for emotion in emotions: 
      score = modelsDict[emotion].score(self.testMFCCs)
      if score > bestScore:
        bestScore=score
        bestEmotion = emotion
    return bestEmotion


if __name__ == '__main__':
  prediction = Prediction()
  filename = 'trainingData/come/Jamie_Gorson.wav'
  prediction.PrepData(filename)
  prediction.predict(emotions, modelsDict)  #TODO- figure out how to pass in models




