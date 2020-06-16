addpath('../../lab_control_center/recording/visualization');
databyveh = preprocessing(0, './output/recording_vehicles_2_3_test_loop.dat');
save('recording_vehicles_2_3_test_loop.mat', 'databyveh');
databyveh = preprocessing(0, './output/recording_vehicles_2_3_test_loop_b.dat');
save('recording_vehicles_2_3_test_loop_b.mat', 'databyveh');
delete('dds_json_sample.mat')
delete('dds_record.mat')