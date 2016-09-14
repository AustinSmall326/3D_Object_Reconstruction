% Optimize the camera pose and 3D points using Jacobian matrix.
%
% params: CSet Camera position for images 1 thru G.
% params: RSet Camera rotation for images 1 thru G.
% params: X    Reconstructed 3D points.
% params: x    Corresponding 2D points corresponding to registered 3D
%              points and cameras 1 thru G.
% params: K    Intrinsic parameter matrix.
% params: traj
% params: V    Visibility matrix.
% params: G    Number of images used in demo.
%
% return: CSet Camera position for images 1 thru G.
% return: RSet Camera rotation for images 1 thru G.
% return: X    Reconstructed 3D points.
%
% Author:  Austin Small

function [CSet, RSet, X] = BundleAdjustmentWithJacobian(CSet, RSet, X, x, K, traj, V, G)
    numImages = size(CSet, 3);
    
    lengthParams = 3 * numImages + 4 * numImages + 3 * size(X, 1);
    params0 = [];
    
    % Load CSet.
    for i = 1:numImages
        params0 = vertcat(params0, CSet(:, :, i));
    end
    
    CSetEndIndex = size(params0, 1);
    
    % Load RSet.
    % Construct parameters into a vector for lsqnonlin.
    QSet = zeros(4, 1, numImages);
    
    for i = 1:numImages
        QSet(:, :, i) = RotationToQuaternion(RSet(:, :, i));  
    end
    
    for i = 1:numImages
        params0 = vertcat(params0, QSet(:, :, i));
    end
    
    QSetEndIndex = size(params0, 1);
    
    % Load 3D points.
    for i = 1:3
        params0 = vertcat(params0, X(:, i));
    end
    
    XEndIndex = size(params0, 1);    
    
    opts = optimoptions(@lsqnonlin, 'Jacobian', 'on', 'Algorithm', 'levenberg-marquardt', 'Display', 'iter');

    fun = @(params) bundleErrorWithJacobian(params, CSetEndIndex, QSetEndIndex, XEndIndex, x, K, V, G);

    XCol = lsqnonlin(fun, params0, [], [], opts);

    % Reconstruct XCol into proper output.
    % Reconstruct CSet.
    CSet = reshape(XCol(1:CSetEndIndex), 3, 1, numImages);
    
    % Reconstruct RSet.
    QSet = reshape(XCol((1 + CSetEndIndex) : QSetEndIndex), 4, 1, numImages);
    RSet = zeros(3, 3, numImages);
    
    for i = 1:numImages
        RSet(:, :, i) = QuaternionToRotation(QSet(:, :, i));
    end

    % Reconstruct XSet.
    X = reshape(XCol((1 + QSetEndIndex) : end), (XEndIndex - QSetEndIndex) / 3, 3);
end