import CleaningData as cd

def predict(self, filename, modelsDict):
  ''' Predicts emotion from emotions of input soundclip
      emotions: list of emotions used

      testMFCC: extracted MFCCs from a short soundclip

      modelsDict: dictionary with keys as emotions and values as the model that matches that emotion

      returns: 
        index of best emotion
  '''
  emotions = ["come", "stop", "goodBoy", "fetch"]

  y, sr = cd.cleanData(filename)
  mfcc = librosa.feature.mfcc(y=y, sr=sr, n_mfcc=13)


  bestEmotion = ""
  bestScore = -10000
  for emotion in emotions: 
    modelsDict[emotion].predict(mfcc)
    score = modelsDict[emotion].score(mfcc)
    aveScore = sum(score)/len(score)
    if aveScore > bestScore:
      bestScore = aveScore
      bestEmotion = emotion
  return emotions.index(bestEmotion)