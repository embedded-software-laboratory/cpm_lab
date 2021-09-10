function main(recording_file, dds_domain)
    if nargin ~= 2
        % standard settings
        recording_folders = dir('/tmp/cpm_lab_recordings/*');
        current_folder = recording_folders(end);
        assert(current_folder.isdir);
        recording_file = fullfile(...
            current_folder.folder, ...
            current_folder.name, ...
            'recording.dat' ...
        );
        dds_domain = getenv("DDS_DOMAIN");
    end
    
    % Preprocess SQL database
    DataByVehicle = preprocessing(dds_domain, recording_file);
    
    % plot x-y, t-v, t-a for each vehicle
    for iVeh = 1:numel(DataByVehicle)
        if isempty(DataByVehicle(iVeh).observation.x)
            % Skip vehicles with no data
            continue
        end
        
        fig = figure('position',[100 100 600 1000],'color',[1 1 1]);
        t = tiledlayout(fig,4,1);
        title(t,['Overview Vehicle ', int2str(iVeh)],'Interpreter','LaTex')

        %% Tile 1: Plot x vs. y
        nexttile([2,1])
        hold on
        box on
        plot(DataByVehicle(iVeh).observation.x, DataByVehicle(iVeh).observation.y,'Linewidth',1)
        xlabel('$x$ [m]','Interpreter','LaTex')
        ylabel('$y$ [m]','Interpreter','LaTex')
        title('Position','Interpreter','LaTex')
        axis equal
        xlim([0 4.5])
        ylim([0 4])

        %% Tile 2: Plot speed over time
        nexttile
        hold on
        box on
        plot(DataByVehicle(iVeh).state.create_stamp, DataByVehicle(iVeh).state.speed, 'Linewidth',1)
        xlabel('$t$ [s]','Interpreter','LaTex') 
        ylabel('$v$ [m/s]','Interpreter','LaTex')
        title('Speed','Interpreter','LaTex')

        %% Tile 3: Plot acceleration over time
        nexttile
        hold on
        box on
        plot(DataByVehicle(iVeh).state.create_stamp, DataByVehicle(iVeh).state.imu_acceleration_forward,'Linewidth',1);
        xlabel('$t$ [s]','Interpreter','LaTex') 
        ylabel('$a$ [m/s$^2$]','Interpreter','LaTex')
        title('Acceleration','Interpreter','LaTex')

        %% todo: export_fig
    end
end