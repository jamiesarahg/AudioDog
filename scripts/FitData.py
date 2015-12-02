import numpy as np
import os
import scipy
from sklearn.mixture import GMM
import CleanData as cd

class Fit(object):

	def __init__(self):
		self.trainingData={}
		self.modelsDict={}

	def PrepData (self, name, emotions):
		data = cd.DataIntake()
		for emotion in emotions: 
			filename = 'trainingData/'+emotion+'/'+name+'.wav'
			mfcc = data.CollectTrainingData(filename)

			if emotion in self.trainingData.keys():
				self.trainingData[emotion].append(mfcc)
			else:
				self.trainingData[emotion] = mfcc
		return self.trainingData

	def CreateModelDictionary(self, data, emotions):
		self.trainingData=data
		for i in range (0,len(emotions)):
			# print emotions[i]
			if emotions[i] not in self.modelsDict.keys():
			  model = self.CreateModel(emotions[i])
			  self.modelsDict[emotions[i]] = model
		return self.modelsDict


	def CreateModel(self, emotion):
		model = GMM().fit(self.trainingData[emotion])
		return model

if __name__ == '__main__':
	fit = Fit()
	emotions = ["come", "stop", "goodBoy", "fetch"]
	data = fit.PrepData('Jamie_Gorson', emotions)
	models = fit.CreateModelDictionary(data, emotions)
  
