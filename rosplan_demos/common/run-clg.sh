#!/bin/bash
ONLINE=1 
while getopts ":01q:" opt; do
  case $opt in
    0)
      ONLINE=1
      USE_Q=0
      echo "-0 online was triggered!" >&2
      ;;
    q)
      ONLINE=1
      USE_Q=1      
      echo "-q was triggered with $OPTARG value" >&2
      Q_VAL=$OPTARG
      ;;
    1)
      echo "-1 offline was triggered!" >&2
      ONLINE=0
      USE_Q=0
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      echo "Usage:"
      echo "$0 -0/1 {-q N} domain.pddl problem.pddl"
      echo "  -0   off-line mode"
      echo "  -1   on-line mode"
      echo "  -q   sensing probability, an int between 0--100 (on-line mode only)"
      exit 1
      ;;
  :)
      echo "Option -$OPTARG requires an argument, an int between 0 and 100" >&2
      exit 1
      ;;
  esac
done
shift $(( $OPTIND-1 ))
./cf2cs -sn -t0 -cond -cod -cmr -csl -ckit -ckinl -cminl -cmit -cdisjk0 -cdisjm0 -mac  -cfc -fp $1 $2
if [ $ONLINE = "0" ];then
#  echo "off line mode"
  ./clg -a 1 -c 1 -v 1 -k 0 -p ./ -o new-d.pddl -f new-p.pddl
  exit 1
fi
if [ $ONLINE = "1" ];then
#  echo "onLine mode -q $OPTARG"
    if [ $USE_Q = "1" ]; then
	./clg -a 1 -c 1 -v 0 -k 1 -p ./ -o new-d.pddl -f new-p.pddl -q $Q_VAL
	exit 1
    else
	./clg -a 1 -c 1 -v 0 -k 1 -p ./ -o new-d.pddl -f new-p.pddl
	exit 1
    fi
fi
