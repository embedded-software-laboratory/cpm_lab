function steering_calibration

    clc
    files = ...
        {'signal_1100.txt',...
        'signal_1200.txt',...
        'signal_1300.txt',...
        'signal_1350.txt',...
        'signal_1400.txt',...
        'signal_1450.txt',...
        'signal_1500.txt',...
        'signal_1600.txt',...
        'signal_1700.txt'} ;

    signals = [1100, 1200, 1300, 1350, 1400, 1450, 1500, 1600, 1700];
    curvatures = nan(size(signals));

    for i = 1:length(files)
        messages = read_json_log(files{i});
        signal = signals(i);
        x = arrayfun(@(j)messages(j).message.pose.position.x, 1:length(messages));
        y = arrayfun(@(j)messages(j).message.pose.position.y, 1:length(messages));        
        qz = arrayfun(@(j)messages(j).message.pose.orientation.z,1:length(messages));
        qw = arrayfun(@(j)messages(j).message.pose.orientation.w,1:length(messages));
        yaw = atan2(qz, qw) * 2;
        s = [0 cumsum(sqrt(diff(x).^2+diff(y).^2))];
        curvature = diff(yaw)./diff(s);
        curvature = curvature(and(abs(diff(yaw)) < 5 , abs(diff(s)) > 1e-7)); % filter angle wrap jumps
        if length(curvature) <= 10
            fprintf('Warning: Only %i datapoints\n', length(curvature));
        end
        subplot(2,1,1)
        plot(x,y)
        axis equal
        subplot(2,1,2)
        histogram(curvature)
        curvatures(i) = median(curvature);
    end


    clf
    hold on
    plot(curvatures, signals)
    scatter(curvatures, signals)
    
    params = polyfit(curvatures, signals, 1);
    
    curvature_fn = linspace(min(curvatures),max(curvatures));
    
    signals_fn = curvature_fn * params(1) + params(2);
    plot(curvature_fn, signals_fn,'--');
    
    fprintf('steering_signal = curvature * (%f) + (%f)\n\n',params(1), params(2));
    
    % result: steering_signal = curvature * (-120.520668) + (1419.494262)
    
end

