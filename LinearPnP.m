% Given 2D-3D correspondences, X <-> x, and the intrinsic parameter K,
% estimate a camera pose using linear least squares.
%
% params: X/x   Nx3/Nx2 matrices whose rows represent correspondences 
%               between 3D and 2D points.
% params: K     3x3 intrinsic parameter matrix.
%             
% return: C/R   Camera pose.
%
% Author:  Austin Small

function [C, R] = LinearPnP(X, x, K)
    N = size(X, 1); % Number of point correspondences.

    A = zeros(3 * N, 12);

    for i = 1:N
        u = x(i, 1);
        v = x(i, 2);
        XHom = vertcat(X(i, :)', 1);

        A(4 * (i - 1) + 1 : 4 * (i - 1) + 3, :) = [zeros(1, 4)    -1 * XHom'     v * XHom';
                                                   XHom'           zeros(1, 4)  -1 * u * XHom';
                                                   -1 * v * XHom'  u * XHom'     zeros(1, 4)];
    end

    % Solve Ax = 0 with SVD.
    [U,D,V]  = svd(A);
    PTemp    = V(:, end);
    PTemp    = reshape(PTemp, 4, 3);
    RTMatrix = inv(K) * PTemp';

    % Extract R and t from camera pose.
    R = RTMatrix(1:3, 1:3);
    t = RTMatrix(:, 4);

    % Enforce orthogonality of rotation matrix.
    [U,D,V] = svd(R);
    R       = U * V';

    if (det(R) < 0)
        R = -1 * R;
    end

    % Compute camera position.
    C = -1 * R' * t;
end