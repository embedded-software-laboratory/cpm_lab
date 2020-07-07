% MIT License
% 
% Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.
% 
% This file is part of cpm_lab.
% 
% Author: i11 - Embedded Software, RWTH Aachen University

function overviewPlot(vehicle_id, dds_domain, recording_file)
%% Check for plotable data else call function preprocessing for provision. 

% if exist('dds_record.mat', 'file')
%     load('dds_record','DataByVehicle')
% else 
%     [DataByVehicle] = preprocessing(dds_domain, recording_file);
% end

[DataByVehicle] = preprocessing(dds_domain, recording_file);

vehicle_fieldname = ['vehicle_' int2str(vehicle_id)];

%if all(strcmp(vehicle_fieldname, fieldnames(DataByVehicle)) == 0) 
%   disp('The vehicle you have chosen is not available. Please enter id [int] of one of the following vehicles:')
%   disp(fieldnames(DataByVehicle))
%else
%end
%% 
close all
%figure('visible','off','position',[100 100 600 650],'color',[1 1 1]);
figure('position',[100 100 600 800],'color',[1 1 1]);

%% Plot x vs. y

subplot(3,1,1);
hold on
box on
plot(DataByVehicle.(vehicle_fieldname).observation.x, DataByVehicle.(vehicle_fieldname).observation.y,'Linewidth',1)
xlabel('$x\ [m]$','Interpreter','LaTex')
ylabel('$y\ [m]$','Interpreter','LaTex')
title('Pose','Interpreter','LaTex')
axis equal
xlim([0 4.5])
ylim([0 4])

%% Plot speed over time

subplot(3,1,2);
hold on
box on
plot(DataByVehicle.(vehicle_fieldname).state.create_stamp, DataByVehicle.(vehicle_fieldname).state.speed, 'Linewidth',1)
%axis equal
xlabel('$t\ [s]$','Interpreter','LaTex') 
ylabel('$v\ [m/s]$','Interpreter','LaTex')
title('Speed','Interpreter','LaTex')
%xlim([0 VehicleState.(vehicle_fieldname_in).T_ticks(numel(VehicleState.(vehicle_fieldname_in).T_ticks))+0.5])


%% Plot acceleration over time

subplot(3,1,3);
hold on
box on
plot(DataByVehicle.(vehicle_fieldname).state.create_stamp, DataByVehicle.(vehicle_fieldname).state.imu_acceleration_forward,'Linewidth',1);
%axis equal
xlabel('$t\ [s]$','Interpreter','LaTex') 
ylabel('$a\ [m/s^2]$','Interpreter','LaTex')
title('Acceleration','Interpreter','LaTex')
%xlim([0 VehicleState.(vehicle_fieldname_in).T_ticks(numel(VehicleState.(vehicle_fieldname_in).T_ticks))+0.5])


%% set suptitle
addpath('suplabel');
fig = gcf; % current figure handle
%fig.Color = [0 0.5 0.5];
[~, h1] = suplabel(['Overview Vehicle '  int2str(vehicle_id)],'t');  
set(h1,'Fontsize', 16, 'Interpreter','Latex')

%% todo: export_fig
end