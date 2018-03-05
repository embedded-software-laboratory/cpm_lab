function format_path_for_controller(path)

clc

% trim to make a perfect loop
dist = sqrt((path(:,1)-path(end,1)).^2 + (path(:,2)-path(end,2)).^2);
i=find(diff(dist)>0);
i=i(1)+1;
path = path(i:end,:);

% output
f = fopen('path.txt','w');

fprintf(f,'const vector<double> path_x {');
fprintf(f,'%f,',path(:,1)');
fprintf(f,'};\n');

fprintf(f,'const vector<double> path_y {');
fprintf(f,'%f,',path(:,2)');
fprintf(f,'};\n');

fprintf(f,'const vector<double> path_yaw {');
fprintf(f,'%f,',path(:,3)');
fprintf(f,'};\n');

fprintf(f,'const vector<double> path_curvature {');
fprintf(f,'%f,',path(:,4)');
fprintf(f,'};\n');

fclose(f);

end

