#!/bin/bash
# Shows the mechanism with which the Bash scripts in this repo determine their current location
# This is required to refer to other scripts within the software folder
# Does not work with symlinks
# Also does not work if a sub-folder leading to a bash script uses the word "software"

# From Stackoverflow
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )/"

# Transform DIR to directory .../software/lab_control_center/bash for test purposes
TEST_DIR="${DIR}../software/lab_control_center/bash"

OUT="$(realpath "${TEST_DIR}")"

echo $DIR
echo $TEST_DIR
echo $OUT