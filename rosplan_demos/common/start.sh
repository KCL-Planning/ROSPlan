#!/bin/bash
#Usage: start.sh files.tar 1|0
#(c) Alexandre Albore 2011

MEGAS_MEM=2400
MIN_TIME=180
SESSANTA=60
ulimit -S -v $((1024 * $MEGAS_MEM))
ulimit -S -t $(($MIN_TIME * $SESSANTA))

f=`tar tf $1 |tee files`
#f=`unzip $1 | grep inflating|cut -d ':' -f2 |tee files`
files=`cat files`
files2=`cat files|tail -1; cat files| head -1`
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:./lib

tar -xvf $1
echo $files

./run-clg.sh -$2 `echo $files`

echo -n 'File size: ' && ls -lh new-d.pddl
