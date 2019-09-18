#!/bin/bash

# Argument 1: Screen ID
tmux kill-session -t "middleware"
tmux kill-session -t "hlc"