#!/usr/bin/env bash

rocks run host tile-0-0 command="DISPLAY=tile-0-0:0.0 /research/jwwalker/vrlab/ivs/gesture-artwork/GestureResponseSlave -0.5 0 -0.5  -0.25  2 FALSE" &
rocks run host tile-0-1 command="DISPLAY=tile-0-1:0.0 /research/jwwalker/vrlab/ivs/gesture-artwork/GestureResponseSlave -0.5 0 -0.25  0     2 FALSE" &
rocks run host tile-0-2 command="DISPLAY=tile-0-2:0.0 /research/jwwalker/vrlab/ivs/gesture-artwork/GestureResponseSlave -0.5 0 0       0.25 2 FALSE" &
rocks run host tile-0-3 command="DISPLAY=tile-0-3:0.0 /research/jwwalker/vrlab/ivs/gesture-artwork/GestureResponseSlave -0.5 0 0.25   0.5   2 FALSE" &

rocks run host tile-0-4 command="DISPLAY=tile-0-4:0.0 /research/jwwalker/vrlab/ivs/gesture-artwork/GestureResponseSlave 0 0.5 -0.5    -0.25 2 FALSE" &
rocks run host tile-0-5 command="DISPLAY=tile-0-5:0.0 /research/jwwalker/vrlab/ivs/gesture-artwork/GestureResponseSlave 0 0.5 -0.25   0     2 FALSE" &
rocks run host tile-0-6 command="DISPLAY=tile-0-6:0.0 /research/jwwalker/vrlab/ivs/gesture-artwork/GestureResponseSlave 0 0.5 0        0.25 2 FALSE" &
rocks run host tile-0-7 command="DISPLAY=tile-0-7:0.0 /research/jwwalker/vrlab/ivs/gesture-artwork/GestureResponseSlave 0 0.5 0.25    0.5   2 FALSE" &

#./GestureResponseMaster FALSE 10.2.255.255 25884 OutputFile FlagObject ObjectsToTrack

./GestureResponseMaster FALSE 10.2.255.255 25884 testoutput.txt Wand HandL HandR
