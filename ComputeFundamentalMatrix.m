% Given N >= 8 correspondences between two images, x1 <-> x2, this function
% linearly estimates a fundemantel matrix, F, such that x2' F x1 = 0.
%
% parmas: x1  Nx2 matrix whose rows represent a correspondence from image 1
% parmas: x2  Nx2 matrix whose rows represent a correspondence from image 2
%
% return: 3x3 fundamental matrix with rank 2.
%
% Author:  Austin Small

function [F] = ComputeFundamentalMatrix(x1, x2)

N = size(x1, 1); % Number of point corespondences.

A = zeros(N, 9);

for i = 1:N
    A(i, 1) = x2(i, 1) * x1(i, 1);
    A(i, 2) = x2(i, 1) * x1(i, 2);
    A(i, 3) = x2(i, 1);
    A(i, 4) = x2(i, 2) * x1(i, 1);
    A(i, 5) = x2(i, 2) * x1(i, 2);
    A(i, 6) = x2(i, 2);
    A(i, 7) = x1(i, 1);
    A(i, 8) = x1(i, 2);
    A(i, 9) = 1;
end

[~,~,V] = svd(A);

% Decompose F matrix using SVD and reshape singular values matrix in order
% to enforce rank 2 of fundamental matrix.
x = V(:, size(V, 2));

F = [x(1) x(2) x(3);
     x(4) x(5) x(6);
     x(7) x(8) x(9)];

[U,D,V] = svd(F);

F = U * diag([D(1,1), D(2,2), 0]) * V';