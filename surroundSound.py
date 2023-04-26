import numpy as np
import matplotlib.pyplot as plt
from openal.audio import SoundSink, SoundSource, SoundData
from openal.loaders import load_wav_file, load_file
import math
import heapq
import time
#Edit /home/pi/.bashrc to run on startup

maxDistance = 12
sink = SoundSink()
sink.activate()
#primeSource = SoundSource()
#primeSource.looping = False

sampNums = 100
base = 360 / sampNums
base2 = 500 / sampNums
sourceL = []
parL = []
for iy in range(sampNums):
	rnd = int(round(iy * base))
	sources = SoundSource()
	sources.looping = False
	#datas = load_file("/home/pi/newdaredevil/waveFiles/wave" + str(int(round(iy * base2))) + ".wav")
	#sources.queue(datas)
	sourceL.append(sources)
	parL.append(rnd)
sourceList = sourceL
parList = parL

lp = 0
while True:
    try:
        otherSin = (math.sin(lp) * 5.0) + 10.0
        cosN = math.cos(lp) * 10.0
        sinN = math.sin(lp) * 10.0
        print(cosN, sinN)
        indx = 50
        sourceDatas = load_file("waveFiles/wave" + str(100) + ".wav")
        sourceList[indx].position = [sinN, cosN, 0]
        sourceList[indx].queue(sourceDatas)
        #if less than 1m away
        sink.play(sourceList[indx])
        sink.update()
        lp += 0.01
        time.sleep(0.01)
    except Exception as e:
    	print(e)
    	break

print("Closed")
