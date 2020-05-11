addpath('../../dds_record/recording_evaluation');
databyveh = preprocessing(0, './recording_vehicles_2_3_test_loop/converted.dat');
save('recording_vehicles_2_3_test_loop.mat', 'databyveh');
databyveh = preprocessing(0, './recording_vehicles_2_3_test_loop_b/converted.dat');
save('recording_vehicles_2_3_test_loop_b.mat', 'databyveh');
delete('dds_json_sample.mat')
delete('dds_record.mat')