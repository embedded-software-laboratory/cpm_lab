#!/bin/bash

export LC_NUMERIC="en_US.UTF-8"

rtirecconv -format csv -decodeChar text -decodeOctet hex -time epoch -compact no -filePrefix topic $1