function [error] = bundleError(params, CSetEndIndex, QSetEndIndex, XEndIndex, x, K, V, G)
    numImages  = CSetEndIndex / 3;
    
    % Reconstruct params into CSet, QSet, and X.
    CSet = reshape(params(1:CSetEndIndex), 3, 1, numImages);
    QSet = reshape(params((1 + CSetEndIndex):QSetEndIndex), 4, 1, numImages);

    X = reshape(params((1 + QSetEndIndex):end), (XEndIndex - QSetEndIndex) / 3, 3);
    
    I = G;          % Number of camera poses.
    J = size(x, 1); % Number of point correspondences.

    error = zeros(2 * I * J, 1);

    count = 1;
    
    % Iterate through all possible camera poses.
    for i = 1:I
        % Compute relevant camera pose.
        C = CSet(:, :, i);
        R = QuaternionToRotation(QSet(:, :, i));
        
        P = K * R * [eye(3) -C];
    
        % Iterate through all 3D points.
        for j = 1:J
            XHom = vertcat(X(j, :)', 1);

            % Compute iteration error.
            error(count)     = V(i, j) * (x(j, 2 * (i - 1) + 1) -  (P(1, :) * XHom) / (P(3, :) * XHom));
            error(count + 1) = V(i, j) * (x(j, 2 * (i - 1) + 2) -  (P(2, :) * XHom) / (P(3, :) * XHom));

            count = count + 2; 
        end
    end
end