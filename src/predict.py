from sklearn.mixture import GMM
import CleaningData as cd
import createModels as cm
import trained_dict as td
import librosa
import pickle

def predict(filename, modelsDict):
  ''' Predicts emotion from emotions of input soundclip
      emotions: list of emotions used

      testMFCC: extracted MFCCs from a short soundclip

      modelsDict: dictionary with keys as emotions and values as the model that matches that emotion

      returns: 
        index of best emotion
  '''
  emotions = ["come", "stop", "goodBoy", "fetch"]
  people = ['jamie', 'susie']

  di= cd.DataIntake(emotions, people)
  y, sr = di.cleanData(filename)
  mfcc = librosa.feature.mfcc(y=y, sr=sr, n_mfcc=13)
  mfcc = mfcc.transpose()
  # print "mfcc: " + str(mfcc)
  # print "mfcc shape: " + str(mfcc.shape)

  bestEmotion = ""
  bestScore = -10000
  for emotion in emotions: 
    model = modelsDict[emotion]
    # print emotion
    # print "model+ " + str(model)
    model.predict(mfcc)
    score = modelsDict[emotion].score(mfcc)
    aveScore = sum(score)/len(score)
    if aveScore > bestScore:
      bestScore = aveScore
      bestEmotion = emotion

  return emotions.index(bestEmotion)

def predict_wrapper():
  #change this filname
  filename = "../wav/sample.wav"
  dictionary = pickle.load(open( "save.p", "rb" ) )
  return predict(filename, dictionary)

if __name__ == '__main__':
  predict_wrapper()
