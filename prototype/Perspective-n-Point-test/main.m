clear

R = [   -0.1990   -0.1218    0.9724;
        -0.1953   -0.9674   -0.1612;
         0.9603   -0.2220    0.1687];
     
t =  -2*[ 0.8   -0.2    0.16];

R = [1 0 0; 0 -1 0; 0 0 -1];
t = [0 0 2];

% points3D = ...
%  [ -1.0616    0.1390   -0.1386;
%     0.0605    2.8584    0.4320;
%     1.5133    0.9618    2.2530;
%     1.3295    1.2449   -0.4596;
%     0.3882    0.6071    0.0905;
%     1.0619   -1.1578    2.6081;
%    -1.1526   -1.8989   -1.0372;
%     0.8241   -0.0802    0.2531;
%     0.9136   -1.0158   -0.2666;
%    -1.0292   -1.4445    1.3175;];
% 
% points3D(:,3) = 0;

[X,Y] = meshgrid([0 7]);
points3D = [X(:) Y(:) zeros(size(X(:)))];
points3D(5,3) = 1;

% Forward calculation
points2D = (points3D - t) / R;
assert(all(points2D(:,3) > 0.1)) % all points in front of camera?
points2D = points2D ./ points2D(:,3);
points2D = points2D(:,1:2);

cameraParams = cameraParameters;

% Reverse calculation
[R2,t2, inlierIdx, status] = ...
    estimateWorldCameraPose(points2D + 1e-7 * randn(size(points2D)),points3D,cameraParams,...
    'MaxNumTrials',1000,...
    'MaxReprojectionError',1)

% Quality test
fprintf('Err R: %e\n', max(max(abs(R\R2-eye(3)))))
fprintf('Err R: %e\n', max(max(abs(R-R2))))
fprintf('Err t: %e\n', max(abs((t-t2))))


% Reprojection error
points2D_rep = (points3D - t2) / R2;
points2D_rep = points2D_rep ./ points2D_rep(:,3);
points2D_rep = points2D_rep(:,1:2);
fprintf('Err rep: %e\n', max(max(abs(points2D_rep - points2D))))

