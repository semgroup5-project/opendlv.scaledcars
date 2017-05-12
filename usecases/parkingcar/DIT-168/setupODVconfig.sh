#!/bin/bash

#define parameters which are passed in.
#CAMID=$1
#MDEBUG=$2
#SIM=$3
#P=$4
#I=$5
#D=$6
#FUNCTION=$7

echo "Do you wish to modify the configuration file [y,n]?" >&2
read ANSWER
case $ANSWER in
    y|Y) 
        if [ -f configuration ];
        then
            echo "Removing old file configuration..." >&2
            rm configuration
        else
            echo "File configuration does not exist!" >&2
        fi
        ./configODV.sh > configuration;;
    *) 
        echo "Using the old configuration file!" >&2;;
esac

#$CAMID $MDEBUG $SIM $P $I $D $FUNCTION 

