Dependencies:
librosa
     pip install librosa
     go to usr/local/lib/python2.7/dist-packages/librosa, open __init__.py, comment out "import display"

-pyaudio
-     sudo apt-get install python-pyaudio python3-pyaudio 

To record data from the microphone:

arecord -t wav -r 176400 -c 2 -f S16_LE > test.wav
Stop with control c


roslaunch neato_node bringup.launch host:=192.168.17.203
ssh pi@192.168.17.203 arecord -t wav -r 176400 -c 2 -f S16_LE | rosrun audioLocAndRec master_node

