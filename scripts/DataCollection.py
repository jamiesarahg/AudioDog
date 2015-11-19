import numpy
import pyaudio
import wave
import os

class DirManager(object):
  """handles image directory
  """
  def __init__(self):
    pass

  def mkdirs(self, emotions):
    """ creates directories to store audiofiles in local storage if they don't already exist
    """
    try: #make an audioFiles directory
      os.mkdir('trainingData')
    except OSError: #if it already exists
      pass

    for emotion in emotions:
      try:
        os.mkdir("trainingData/"+emotion)
      except OSError:
        pass

class DataCollection(DirManager):
  """Ask user to speak in the emotion given and record into folders.
  """

  def __init__(self, emotions):
    self.emotions = emotions
    self.mkdirs(emotions)
    self.CHUNK = 1024
    self.FORMAT = pyaudio.paInt16
    self.CHANNELS = 2
    self.RATE = 44100
    self.RECORD_SECONDS = 2
    self.paudio = pyaudio.PyAudio()
    self.name = None


  def recordAudio(self, emotion):
      self.WAVE_OUTPUT_FILENAME = "trainingData/"+emotion+"/"+self.name+".wav"
      self.stream = self.paudio.open(format=self.FORMAT,
                channels=self.CHANNELS,
                rate=self.RATE,
                input=True,
                frames_per_buffer=self.CHUNK)

      print("* recording")

      frames = []

      for i in range(0, int(self.RATE / self.CHUNK * self.RECORD_SECONDS)):
          data = self.stream.read(self.CHUNK)
          frames.append(data)

      print("* done recording")

      self.stream.stop_stream()
      self.stream.close()
        

      wf = wave.open(self.WAVE_OUTPUT_FILENAME, 'wb')
      wf.setnchannels(self.CHANNELS)
      wf.setsampwidth(self.paudio.get_sample_size(self.FORMAT))
      wf.setframerate(self.RATE)
      wf.writeframes(b''.join(frames))
      wf.close()

  def run(self, emotions):
    print "Type your unique name"
    self.name = raw_input()
    os.system('clear')

    print "Hi " + self.name
    print "Please speak when asked like you would instruct a dog with the given emotion or command. You will have two seconds for each emotion"
    print "Press enter when you are ready"
    raw_input()
    os.system('clear')

    for emotion in emotions:
      print "Like you are commanding or talking to a dog, say a corresponding command with the following emotion:"
      print emotion
      print "When you are ready press enter"
      raw_input()
      self.recordAudio(emotion)
      os.system('clear')

    self.paudio.terminate()

    print 'Thank you! Data Collection Complete'


if __name__ == '__main__':
  emotions = ["come", "stop", "goodBoy", "fetch"]
  objectRecording = DataCollection(emotions)
  objectRecording.run(emotions)
  # for item in emotions:
  #   objectRecording.recordAudio(item, 'Jamie')

