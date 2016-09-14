% Given two camera poses, and correspondences x1 <-> x2, triangulate 3D 
% points using linear least squares.
%
% params: C1/R1 The first camera pose.
% params: C2/R2 The second camera pose.
% params: x1/x2 Nx2 matrices whose rows represent correspondences bewteen
%               the first and second image.
%
% return: X     Nx3 matrix whose rows represent 3D triangulated points.
%
% Author:  Austin Small

function [X] = LinearTriangulation(K, C1, R1, C2, R2, x1, x2)

N = size(x1, 1); % Number of point correspondences.

P1 = K * R1 * [eye(3) -C1];
P2 = K * R2 * [eye(3) -C2];

X = zeros(N, 3);

% Iterate through all point correspondences and compute 3d points.
for i = 1:size(x1, 1)
     x1CrossMatrix = [ 0          -1         x1(i, 2);
                       1           0        -x1(i, 1);
                      -x1(i, 2)    x1(i, 1)  0];
    
     x2CrossMatrix = [ 0          -1         x2(i, 2);
                       1           0        -x2(i, 1);
                      -x2(i, 2)    x2(i, 1)  0];
  
    A = [x1CrossMatrix * P1; 
         x2CrossMatrix * P2];
     
     % Solve Ax = 0 with SVD.
    [U,D,V] = svd(A);
    xTemp = V(:, end) / V(end, end);
    xTemp = xTemp(1:3, 1);
    
    X(i, :) = xTemp';
end