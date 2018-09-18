#!/usr/bin/env bash

 while sleep 1; do
     if [ $(pgrep apt | wc -l) -lt 1 ] ; then
         echo "apt process done"
         break
     else
         echo "apt process has not done yet"
     fi
 done
