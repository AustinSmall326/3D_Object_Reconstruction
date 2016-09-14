% Given two camera poses and linearly triangulated points, X, refine the 
% locations of the 3D points that minimizes reprojection error.
%
% params: K      3x3 intrinsic parameter matrix.
% params: C1/R1  The first camera pose.
% params: C2/R2  The second camera pose.
% params: x1/x2  Nx2 matrices whose rows represent correspondences between
%                the first and second images.
% params: X      Nx3 matrix whose rows represent 3D triangulated points.
%
% return: X      Nx3 matrix whose rows represent 3D triangulated points.
%
% Author:  Austin Small

function [X] = NonlinearTriangulation(K, C1, R1, C2, R2, x1, x2, X0)
    N = size(X0, 1);

    opts = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', 'TolX', 0.01, 'TolFun', 0.01, 'MaxFunEvals', 1e64, 'MaxIter', 1e64, 'Display', 'iter');
    params0 = reshape(X0, 3 * N, 1);
    fun = @(params) algError(K, C1, R1, C2, R2, x1, x2, params);

    XCol = lsqnonlin(fun, params0, [], [], opts);

    X = reshape(XCol, N, 3);
end