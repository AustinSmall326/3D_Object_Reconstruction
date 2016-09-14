clear;
G = 4;              % Additional images to register.
featureIDSeen = []; % Features registered thus far.
                    % X - a ~x3 sized matrix to store all global points.

% Parse data files.
[~,~,K]          = generate_matching_feature_cell ('SfMProjectData_1');
[matches]        = parser();
matches(1, :, :) = [];

% Compute inliers for all possible image pairings.
inliers = cell(G, G);

for i = 1:G
    for j = (i + 1):G
        disp('index')
        disp(i)
        disp(j)
        [ID, x1, x2] = getCorrespondences(matches, i, j);
    	[~, ~, idx, ~] = GetInliersRANSAC(x1, x2);

        inliers{i, j} = horzcat(ID(idx), x1(idx, :), x2(idx, :));
        inliers{j, i} = horzcat(ID(idx), x2(idx, :), x1(idx, :));
    end
end

[initialFeatureIDs, x1, x2] = extractInliers(inliers, 1, 2);
[F] = ComputeFundamentalMatrix(x1, x2);

[E] = EssentialMatrixFromFundamentalMatrix(F, K);

[CSet, RSet] = ExtractCameraPose(E);

c1 = CSet(:, 1);
c2 = CSet(:, 2);
c3 = CSet(:, 3);
c4 = CSet(:, 4);

r1 = RSet(:, :, 1);
r2 = RSet(:, :, 2);
r3 = RSet(:, :, 3);
r4 = RSet(:, :, 4);

[X1] = LinearTriangulation(K, zeros(3, 1), eye(3), c1, r1, x1, x2);
[X2] = LinearTriangulation(K, zeros(3, 1), eye(3), c2, r2, x1, x2);
[X3] = LinearTriangulation(K, zeros(3, 1), eye(3), c3, r3, x1, x2);
[X4] = LinearTriangulation(K, zeros(3, 1), eye(3), c4, r4, x1, x2);

N = size(X1, 1); 
XSet = zeros(N, 3, 4);

XSet(:, :, 1) = X1;
XSet(:, :, 2) = X2;
XSet(:, :, 3) = X3;
XSet(:, :, 4) = X4;

[C, R, X0] = DisambiguateCameraPose(CSet, RSet, XSet);

% Visualize results.
figure;
XD = X0(:, 1);
YD = X0(:, 2);
ZD = X0(:, 3);
plot3(XD, YD, ZD, '.', 'MarkerSize', 1);
title('Nonlinear Triangulation');
hold on;

axis([-50 50 -50 50 -50 50]);

cam1 = plotCamera('Location',C','Orientation',R,'Opacity',0, 'Size', 0.8);

% Adjust 3D points from images 1 and 2.
[X0] = NonlinearTriangulation(K, zeros(3, 1), eye(3), C, R, x1, x2, X0);

%FileData = load(fullfile(tempdir, 'pnpOutput.mat'));
%X0 = FileData.X0;
 
% CSet is a 3x1x6 matrix that stores positions of all cameras.
% RSet is a 3x3x6 matrix that stores orientation of all cameras.
CSet = zeros(3, 1, 6);
RSet = zeros(3, 3, 6);

% Store C and R for first two cameras.
CSet(:, :, 1) = zeros(3, 1);
RSet(:, :, 1) = eye(3, 3);

CSet(:, :, 2) = C;
RSet(:, :, 2) = R;

% Store global points computed via correspondences between first two
% cameras.
X = horzcat(initialFeatureIDs, X0);
featureIDSeen = vertcat(featureIDSeen, initialFeatureIDs);

% 
% load('tempdata.mat');
% 

% Register cameras 3 through G.
for i = 3:G
    % Determine position and rotation for new camera.
    [~, xTemp, XTemp] = getRelevantGlobalPoints(matches, i, featureIDSeen, X);

    [Cnew, Rnew] = PnPRANSAC(XTemp, xTemp, K);
    [Cnew, Rnew] = NonlinearPnP(XTemp, xTemp, K, Cnew, Rnew);

    CSet(:, :, i) = Cnew;
    RSet(:, :, i) = Rnew;
    
    % Iterate through all pairings of camera i with previously registered
    % cameras in order to triangulate new points.
    for j = (i - 1):(i - 1)
        [featureIDSecondary, y1, y2] = extractInliers(inliers, i, j);
        
        Xnew = LinearTriangulation(K, CSet(:, :, i), RSet(:, :, i), CSet(:, :, j), RSet(:, :, j), y1, y2);
        Xnew = NonlinearTriangulation(K, CSet(:, :, i), RSet(:, :, i), CSet(:, :, j), RSet(:, :, j), y1, y2, Xnew); 
        
        % Determine which points to add to set of global points.
        bin = ismember(featureIDSecondary, featureIDSeen);
        bin = 1 - bin;
        
        indices = all(bin, 2);
    
        newPoints   = Xnew(indices, :);
        newPointIDs = featureIDSecondary(indices, :);
        
        X = vertcat(X, horzcat(newPointIDs, newPoints));
        featureIDSeen = vertcat(featureIDSeen, newPointIDs); 
    end
end

%
% load('checkpoint2.mat');
% 

% Build trajectory matrix.
traj = zeros(size(X, 1), 1 + G);
traj(:, 1) = X(:, 1);

for image = 1:G
    featureIDS  = matches(:, 1, 1);
    imagePoints = matches(:, (2 + 2 * (image - 1)) : (3 + 2 * (image - 1)), 1);
    imageBin    = matches(:, (2 + 2 * (image - 1)) : (3 + 2 * (image - 1)), 2);
    
    % Create a binary vector to indicate if the global point has been
    % registered.
    globalBin = ismember(featureIDS, featureIDSeen);
    
    combined = horzcat(featureIDS, imageBin, globalBin);
    
    % Filter for features that have been registered and can be seen
    % by image.
    indices = all(combined, 2);

    bin = ismember(traj(:, 1), featureIDS(indices, 1));
    
    traj(:, image + 1) = bin;
end

[V] = BuildVisibilityMatrix(traj);

% Construct X and x for bundle adjustment.
XBundle = X(:, 2:4);

indices = X(:, 1);
x = matches(indices, 2:end, 1);

tic;
[CSet, RSet, XDisp] = BundleAdjustment(CSet, RSet, XBundle, x, K, traj, V, G);
BAElapsedTime = toc;

% Visualize results.
figure;
XD = XDisp(:, 1);
YD = XDisp(:, 2);
ZD = XDisp(:, 3);
plot3(XD, YD, ZD, '.', 'MarkerSize', 1);
title('Bundle Adjustment');
hold on;

axis([-50 50 -50 50 -50 50]);

cam1 = plotCamera('Location',CSet(:, :, 1)','Orientation',RSet(:, :, 1),'Opacity',0, 'Size', 0.8);
cam2 = plotCamera('Location',CSet(:, :, 2)','Orientation',RSet(:, :, 2),'Opacity',0, 'Size', 0.8);
cam3 = plotCamera('Location',CSet(:, :, 3)','Orientation',RSet(:, :, 3),'Opacity',0, 'Size', 0.8);
cam4 = plotCamera('Location',CSet(:, :, 4)','Orientation',RSet(:, :, 4),'Opacity',0, 'Size', 0.8);
% cam5 = plotCamera('Location',CSet(:, :, 5)','Orientation',RSet(:, :, 5),'Opacity',0, 'Size', 0.8);
% cam6 = plotCamera('Location',CSet(:, :, 6)','Orientation',RSet(:, :, 6),'Opacity',0, 'Size', 0.8);

% load('postBundleAdjustment.mat');

tic;
[CSet, RSet, XDisp] = BundleAdjustmentWithJacobian(CSet, RSet, XBundle, x, K, traj, V, G)
BAJElapsedTime = toc;

% Visualize results.
figure;
XD = XDisp(:, 1);
YD = XDisp(:, 2);
ZD = XDisp(:, 3);
plot3(XD, YD, ZD, '.', 'MarkerSize', 1);
title('Bundle Adjustment with Jacobian');
hold on;

axis([-50 50 -50 50 -50 50]);

cam1 = plotCamera('Location',CSet(:, :, 1)','Orientation',RSet(:, :, 1),'Opacity',0, 'Size', 0.8);
cam2 = plotCamera('Location',CSet(:, :, 2)','Orientation',RSet(:, :, 2),'Opacity',0, 'Size', 0.8);
cam3 = plotCamera('Location',CSet(:, :, 3)','Orientation',RSet(:, :, 3),'Opacity',0, 'Size', 0.8);
cam4 = plotCamera('Location',CSet(:, :, 4)','Orientation',RSet(:, :, 4),'Opacity',0, 'Size', 0.8);
% cam5 = plotCamera('Location',CSet(:, :, 5)','Orientation',RSet(:, :, 5),'Opacity',0, 'Size', 0.8);
% cam6 = plotCamera('Location',CSet(:, :, 6)','Orientation',RSet(:, :, 6),'Opacity',0, 'Size', 0.8);