clear

R = [   -0.1990   -0.1218    0.9724;
        -0.1953   -0.9674   -0.1612;
         0.9603   -0.2220    0.1687];
     
t =  [0.3192    0.3129    -2.1351];

points3D = ...
 [ -1.0616    0.1390   -0.1386;
    0.0605    2.8584    0.4320;
    1.5133    0.9618    2.2530;
    1.3295    1.2449   -0.4596;
    0.3882    0.6071    0.0905;
    1.0619   -1.1578    2.6081;
   -1.1526   -1.8989   -1.0372;
    0.8241   -0.0802    0.2531;
    0.9136   -1.0158   -0.2666;
   -1.0292   -1.4445    1.3175;];

% Forward calculation
points2D = (points3D - t) / R;
points2D = points2D ./ points2D(:,3);
points2D = points2D(:,1:2);

cameraParams = cameraParameters;

% Reverse calculation
[R2,t2, inlierIdx, status] = ...
    estimateWorldCameraPose(points2D + 1e-4 * randn(size(points2D)),points3D,cameraParams,...
    'MaxNumTrials',1000,...
    'MaxReprojectionError',1)

% Quality test
R\R2-eye(3)
(t-t2)./t
