#!/bin/bash
set -e

source setup.sh

if [ ! -f src/.idea/workspace.xml ]; then
    cp src/.idea/workspace.xml.template src/.idea/workspace.xml
fi

nohup ~/clion/bin/clion.sh src >/dev/null 2>&1 &