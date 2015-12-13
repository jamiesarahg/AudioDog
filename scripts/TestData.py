import numpy as np
import os
import scipy
from sklearn.mixture import GMM
import CleaningData as cd
import FitData as Fit

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
  fit = Fit.Fit()
  emotions = ["come", "stop", "goodBoy", "fetch"]
  data = fit.PrepData('Jamie_Gorson', emotions)
  models = fit.CreateModelDictionary(data, emotions)


  prediction = Prediction()
  filename = 'trainingData/come/Jamie_Gorson.wav'
  prediction.PrepData(filename)
  prediction.predict(emotions, modelsDict)  #TODO- figure out how to pass in models




