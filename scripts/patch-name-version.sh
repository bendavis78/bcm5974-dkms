#!/bin/sh
#1: name
#2: version
#3: file
cat $3 \
| sed "s/^NAME=.*/NAME=$1/" \
| sed "s/^VERSION=.*/VERSION=$2/" \
> /tmp/$1.$$
mv /tmp/$1.$$ $3
