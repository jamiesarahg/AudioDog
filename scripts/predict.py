def predict(self, emotions, testMFCC, modelsDict):
  ''' Predicts emotion from emotions of input soundclip
      emotions: list of emotions used

      testMFCC: extracted MFCCs from a short soundclip

      modelsDict: dictionary with keys as emotions and values as the model that matches that emotion

      returns: 
        index of best emotion
  '''
  
  bestEmotion = ""
  bestScore = -10000
  for emotion in emotions: 
    modelsDict[emotion].predict(testMFCC)
    score = modelsDict[emotion].score(testMFCC)
    aveScore = sum(score)/len(score)
    if aveScore > bestScore:
      bestScore = aveScore
      bestEmotion = emotion
  return emotions.index(bestEmotion)