#!/bin/bash

export LC_NUMERIC="en_US.UTF-8"

mkdir output
pushd output



for f in ../recording*.dat_0_0; do
    export PREFIX="$(echo $f | tr -cd '[:alnum:]_')"
    rtirecconv -format csv -decodeChar text -decodeOctet hex -time epoch -compact no -filePrefix $PREFIX $f
done

