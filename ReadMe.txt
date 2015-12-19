
#AUDIODOG
A collection of scripts meant to be run on with a rasppi-equipped NEATO robot.


## Authors and Contact
This software was created by a tean of students at Olin College.

Authors:  
 - Antonia Elsen
 - Jamie Gorson
 - Susie Grimshaw



## Usage
Connect to the neato:  
roslaunch neato_node bringup.launch host:=192.168.17.203

Run the master node:  
ssh pi@[your raspberry pi's IP] arecord -t wav -r 176400 -c 2 -f S16_LE | rosrun audioLocAndRec master_node

Exit the master node with a keyboard interrupt (ctrl-c).


## Dependencies:
python 2.7.6 - python 2.7.9
 - numpy
 - scipy (signal, fftpack, cong, wavfile)
 - librosa [pip install librosa -> go to usr/local/lib/python2.7/dist-packages/librosa, open __init__.py, comment out "import display"]
 - pyaudio [sudo apt-get install python-pyaudio python3-pyaudio]


 ## Main Scripts
 - cleanData.py
 - cleaningData.py
 - createModels.py
 - cross_correlation.py
 - DataCollection.py
 - master_node.cpp
 - predict.py
 - TestData.py
 - trained_dict.py



