function overviewPlot(vehicle_in)
%% Check for plotable data else call function preprocessing for provision. 

if exist('dds_record.mat', 'file')
    load('dds_record','DataByVehicle')
else 
    [DataByVehicle] = preprocessing();
end

vehicle_fieldname = ['vehicle_' int2str(vehicle_in)];

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

fig = gcf; % current figure handle
%fig.Color = [0 0.5 0.5];
[~, h1] = suplabel(['Overview Vehicle '  int2str(vehicle_in)],'t');  
set(h1,'Fontsize', 16, 'Interpreter','Latex')

%% todo: export_fig
end