#!/bin/bash
## Share

# I use this script to share the package with other computers - and robots
#

# Usage: rosrun pioneer share.sh aadi1@aadi1
# 

# Destination host
dest=$1

# Working directory src, relative to user home
srcdir='ros-ws/mobile/src/'

# Package name
pfile=$(dirname $0)/../package.xml
pname=$(xmllint --xpath '/package/name/text()' $pfile)

# Removing old package
com="ssh $dest rm -Rf $srcdir$pname"
echo $com
$com

# Copying new package
com="scp -r ${HOME}/$srcdir$pname $dest:$srcdir"
echo $com
$com
