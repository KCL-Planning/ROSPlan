#!/bin/bash
# Usage: ./show_plan.sh <path_to_plan>.pdf
# i.e. ./show_plan.sh /tmp/my_plan.pdf

OUTPUT="/tmp/plan.pdf"
if [ ! -z "$1" ]; then
    OUTPUT=$1
fi
rostopic echo /rosplan_plan_dispatcher/plan_graph -n 1 -p | sed "N;s/.*digraph/digraph/g" | dot -Tpdf > $OUTPUT
xdg-open $OUTPUT &
echo "Esterel plan has been printed in $OUTPUT"
