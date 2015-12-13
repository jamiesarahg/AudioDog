Dependencies:
librosa
     pip install librosa

-pyaudio
-     sudo apt-get install python-pyaudio python3-pyaudio 

To record data from the microphone:

arecord -t wav -r 176400 -c 2 -f S16_LE > test.wav
Stop with control c
