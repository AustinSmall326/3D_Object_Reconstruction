function [error] = NonLinAlgError(X, x, K, Q)

N = size(X, 1); % Number of point correspondences.

C = Q(1:3);

% Compose rotation matrix from quaternion elements.
q0 = Q(4);
q1 = Q(5);
q2 = Q(6);
q3 = Q(7);

R = [q0^2 + q1^2 - q2^2 - q3^2   2 * (q1*q2 - q0*q3)         2 * (q1*q3 + q0*q2);
     2 * (q1*q2 + q0*q3)         q0^2 - q1^2 + q2^2 - q3^2   2 * (q2*q3 - q0*q1);
     2 * (q1*q3 - q0*q2)         2 * (q2*q3 + q0*q1)         q0^2 - q1^2 - q2^2 + q3^2];
 
P = K * R * [eye(3) -C];

error = zeros(2 * N, 1);

for i = 1:N
    % Homogeneous representation of world point.
    XHom = vertcat(X(i, :)', 1);
    u    = x(i, 1);
    v    = x(i, 2);
    
    % Compute reprojection error.
    error(2 * (i - 1) + 1) = u - (P(1, :) * XHom) / (P(3, :) * XHom);
    error(2 * (i - 1) + 2) = v - (P(2, :) * XHom) / (P(3, :) * XHom);
end