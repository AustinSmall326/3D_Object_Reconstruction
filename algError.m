function [error] = algError(K, C1, R1, C2, R2, x1, x2, XCol)
N = size(x1, 1); % Number of point correspondences.

X = reshape(XCol, N, 3);

P1 = K * R1 * [eye(3) -C1];
P2 = K * R2 * [eye(3) -C2];

error = zeros(4 * N, 1);

for i = 1:N
    % Homogeneous representation of world point.
    XHom = vertcat(X(i, :)', 1);

    % Camera 1.
    error(4 * (i - 1) + 1) = (x1(i, 1) -  (P1(1, :) * XHom) / (P1(3, :) * XHom));
    error(4 * (i - 1) + 2) = (x1(i, 2) -  (P1(2, :) * XHom) / (P1(3, :) * XHom));

    
    % Camera 2.
    error(4 * (i - 1) + 3) = (x2(i, 1) -  (P2(1, :) * XHom) / (P2(3, :) * XHom));
    error(4 * (i - 1) + 4) = (x2(i, 2) -  (P2(2, :) * XHom) / (P2(3, :) * XHom));
end