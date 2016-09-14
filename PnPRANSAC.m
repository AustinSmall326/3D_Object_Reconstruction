% Given N >= 6 3D-2D correspondences, X <-> x, estimate camera pose via
% RANSAC.
%
% params: X/x Nx3 and Nx2 matrices whose rows represent correspondences 
%         between 3D and 2D points.
% params: K   3x3 intrinsic parameter matrix.
%
% return: C/R Camera pose.
%
% Author:  Austin Small

function [C, R] = PnPRANSAC(X, x, K)
    M = 10000;      % Number of RANSAC iterations.
    N = size(X, 1); % Number of point correspondences.
    disp(size(X, 1))
    disp(size(x, 1))
    n = 0;
    eps = 1;
    SOut = [];
    CBest = zeros(3, 1);
    RBest = zeros(3, 3);

    % Perform RANSAC subroutine.
    for i = 1:M
        % Choose 6 correspondences randomly.        
        indices = randperm(N);
        
        Xhat = zeros(6, 3);
        xhat = zeros(6, 2);
        
        for j = 1:6
            Xhat(j, :) = X(indices(j), :);
            xhat(j, :) = x(indices(j), :);
        end
        
        [C, R] = LinearPnP(Xhat, xhat, K);
        P = K * R * [eye(3) -C];
        STemp = [];

        % Iterate through all point correspondences.
        for j = 1:N
            u = x(j, 1);
            v = x(j, 2);
            XHom = vertcat(X(j, :)', 1);
            
            e = (u - (P(1, :) * XHom) / (P(3, :) * XHom))^2 + (v - (P(2, :) * XHom) / (P(3, :) * XHom))^2;

            if (e < eps)
                STemp = horzcat(STemp, [j]);
            end
        end
        
        if (n < size(STemp, 2))
            n = size(STemp, 2);
            CBest = C;
            RBest = R;
        end
    end
    
    C = CBest;
    R = RBest;
end