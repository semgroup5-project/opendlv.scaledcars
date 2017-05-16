#!/bin/bash

#define parameters which are passed in.
ANS=$1
ANS2=$2
IP=$3
P=$4
I=$5
D=$6
FUNCTION=$7
FUNCTION2=$8

case $ANS in
    y|Y) 
        if [ -f configuration ];
        then
            echo "Removing old file configuration..." >&2
            rm configuration
        else
            echo "File configuration does not exist!" >&2
        fi
        ./configODV.sh $ANS2 $IP $P $I $D $FUNCTION $FUNCTION2 > configuration;;
    *) 
        echo "Using the old configuration file!" >&2;;
esac

#$ANS2 $IP $P $I $D $FUNCTION $FUNCTION2

