#!/bin/bash

rticonverter -cfgFile LEGACY_CONVERTER.xml -cfgName test_loop
rticonverter -cfgFile LEGACY_CONVERTER.xml -cfgName test_loop_b

matlab -sd . -batch sql_to_mat

rm -r recording_vehicles_2_3_test_loop
rm -r recording_vehicles_2_3_test_loop_b