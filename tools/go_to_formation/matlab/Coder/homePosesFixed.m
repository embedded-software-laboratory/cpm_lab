function homePoses = homePosesFixed()

    homePoses = repmat(Pose2D, 1, 30);
    mapY = 4.0;

    % vehicle dimensions [m]
    vehicleRearOverhang = 0.03;
    vehicleLength = 0.2200;
    vehicleWidth= 0.1070;
    vehicleHalfWidth = vehicleWidth/2;

    % arrangement of vehicles
    maxNoVehicles = 20;
    vehicleRows = 3;
    vehicleColumns = ceil(maxNoVehicles/vehicleRows);
    clearance = vehicleHalfWidth; % safety distance vehicle to vehicle and vehicle to map edge [m]


    for i = 1:length(homePoses)
        if mod(i, vehicleColumns) == 0
            column = vehicleColumns; % vehicle count in y (=rows)
            row = floor(i/vehicleColumns); % vehicle count in x (=columns)
        else 
            column = mod(i, vehicleColumns);
            row = 1 + floor(i/vehicleColumns); % vehicle count in x (=columns)
        end
        homePoses(i).x = 2 * vehicleWidth + column * clearance + (column-1) * vehicleWidth ;
        homePoses(i).y = mapY - (vehicleLength-vehicleRearOverhang) - row * clearance - (row-1) * vehicleLength * 2;
        homePoses(i).yaw = 90;

    end

    save('homePoses.mat', 'homePoses')

end