#!/bin/bash

rticonverter -cfgFile LEGACY_CONVERTER.xml -cfgName test_loop
rticonverter -cfgFile LEGACY_CONVERTER.xml -cfgName test_loop_b

matlab -sd . -batch sql_to_mat