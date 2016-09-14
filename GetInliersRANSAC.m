% Given N correspondences between two images (N >= 8), x1 <-> x2, estimate
% inlier correspondences using fundamental matrix based RANSAC.
%
% params: x1  Nx2 matrix whose rows represent a correspondence from image 1
% params: x2  Nx2 matrix whose rows represent a correspondence from image 2
%
% return: y1  Nx2 matrix whose rows represent an inlier correspondence.
%         y2  Nx2 matrix whose rows represent an inlier correspondence.
%         idx Nx1 vector that indicates ID of inlier y1.
%
% Author:  Austin Small

function [y1, y2, idx, FBest] = GetInliersRANSAC(x1, x2)
    N = size(x1, 1);      % Total number of correspondences.
    n = 0;                % Current number of inliers.
    eps = 0.1;            % Inlier error threshold.
    SOut = [];            % Indices of inliers.
    FBest = zeros(3, 3);
    
    % Check that there are at least 8 point correspondences.
    if (N >= 8)
        for i = 1:2000
            % Choose 8 correspondences randomly.        
            indices = randperm(N);

            x1Hat = zeros(8, 2);
            x2Hat = zeros(8, 2);

            for i = 1:8
                x1Hat(i, 1) = x1(indices(i), 1);
                x1Hat(i, 2) = x1(indices(i), 2);

                x2Hat(i, 1) = x2(indices(i), 1);
                x2Hat(i, 2) = x2(indices(i), 2);
            end

            F = ComputeFundamentalMatrix(x1Hat, x2Hat);

            STemp = [];

            % Check for inliers.
            for j = 1:N
                x2Point = [x2(j,1); x2(j,2); 1];
                x1Point = [x1(j,1); x1(j,2); 1];

                  if (norm(x2Point' * F * x1Point) < eps)
                      STemp = horzcat(STemp, [j]);
                  end
            end

            if (n < size(STemp, 2))
                n = size(STemp,2);
                SOut = STemp;
                FBest = F;
            end
        end
    
        % Generate output inliers.
        idx = SOut';
        y1 = zeros(size(SOut, 2), 2);
        y2 = zeros(size(SOut, 2), 2);

        for i = 1:size(SOut, 2)
            y1(i, 1) = x1(idx(i), 1);
            y1(i, 2) = x1(idx(i), 2);

            y2(i, 1) = x2(idx(i), 1);
            y2(i, 2) = x2(idx(i), 2);
        end
    else
        y1    = [];
        y2    = [];
        idx   = [];
        FBest = [];
    end
end