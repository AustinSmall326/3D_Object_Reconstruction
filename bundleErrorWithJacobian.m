function [error, Jacobian] = bundleErrorWithJacobian(params, CSetEndIndex, QSetEndIndex, XEndIndex, x, K, V, G)

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

    % Generate Jacobian.
    Jacobian = zeros(size(error, 1), size(params, 1));
    
    count = 1;

    % Re-iterate through all possible camera poses.
    for i = 1:I
        % Compute relevant camera pose.
        C = CSet(:, :, i);
        Q = QSet(:, :, i);
        
        qw = Q(1);
        qx = Q(2);
        qy = Q(3);
        qz = Q(4);
        
        % Iterate through all 3D points.
        for j = 1:J
            XIHom = X(j, :)'; 
            XHom  = vertcat(X(j, :)', 1);

            % Calibration parameters.
            px = K(1, 3);
            py = K(2, 3);
            f  = K(1, 1);
            
            u = [(f * R(1,1) + px * R(3,1)) (f * R(1,2) + px * R(3,2)) (f * R(1,3) + px * R(3,3))] * [XIHom - C];
            v = [(f * R(2,1) + py * R(3,1)) (f * R(2,2) + py * R(3,2)) (f * R(2,3) + py * R(3,3))] * [XIHom - C];
            w = [ R(3,1)                     R(3,2)                     R(3,3)]                    * [XIHom - C];
            
            % partial f / R
            pUpR = [f * (XIHom - C)'  zeros(1, 3)        px * (XIHom - C)'];
            pVpR = [zeros(1, 3)       f * (XIHom - C)'   px * (XIHom - C)'];
            pWpR = [zeros(1, 3)       zeros(1, 3)             (XIHom - C)'];
                
            pFpR = [(w * pUpR - u * pWpR) / w^2;
                    (w * pVpR - v * pWpR) / w^2];
                
            % partial q / R (note: quaternion is defined as [qw qx qy qz])
            
                   %  w         x        y         z
            pRpQ = [ 0         0        -4 * qy   -4 * qz;
                    -2 * qz    2 * qy    2 * qx   -2 * qw;
                     2 * qy    2 * qz    2 * qw    2 * qx;
                     2 * qz    2 * qy    2 * qx    2 * qw;
                     0        -4 * qx    0        -4 * qz;
                    -2 * qx   -2 * qw    2 * qz    2 * qy;
                    -2 * qy    2 * qz    2 * qw    2 * qx;
                     2 * qx    2 * qw    2 * qz    2 * qy;
                     0        -4 * qx   -4 * qy    0     ];  

            % partial f / C
            pUpC = -1 * [(f * R(1,1) + px * R(3,1)) (f * R(1,2) + px * R(3,2)) (f * R(1,3) + px * R(3,3))];
            pVpC = -1 * [(f * R(2,1) + py * R(3,1)) (f * R(2,2) + py * R(3,2)) (f * R(2,3) + py * R(3,3))]; 
            pWpC = -1 * [ R(3,1)                     R(3,2)                     R(3,3)];
            
            pFpC = [(w * pUpC - u * pWpC) / w^2;
                    (w * pVpC - v * pWpC) / w^2];
            
            % partial f / X
            pUpX = [(f * R(1,1) + px * R(3,1)) (f * R(1,2) + px * R(3,2)) (f * R(1,3) + px * R(3,3))];
            pVpX = [(f * R(2,1) + py * R(3,1)) (f * R(2,2) + py * R(3,2)) (f * R(2,3) + py * R(3,3))];
            pWpX = [ R(3,1)                     R(3,2)                     R(3,3)];
            
            pFpX = [(w * pUpX - u * pWpX) / w^2;
                    (w * pVpX - v * pWpX) / w^2];
            
            Jacobian(count : (count + 1), :) = [pFpR * pRpQ pFpC zeros(2, 3 * (j - 1)) pFpX zeros(2, size(params, 1) - 10 - 3 * (j - 1))];

            count = count + 2; 
        end
    end
    
    size(Jacobian)
end