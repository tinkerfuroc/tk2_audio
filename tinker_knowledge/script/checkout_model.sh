#! /bin/bash

echo "checkout pocketsphinx model, please wait..."
if [ -d "$1" ]; then
    echo "model exists"
else
    svn checkout svn://svn.code.sf.net/p/cmusphinx/code/trunk/pocketsphinx/model@r13098 $1
fi
