% Given 3D-2D correspondences, X <-> x, and linearly estimated camera pose
% C, R refine the camera pose that minimizes reprojection error.
%
% params: X/x Nx3 and Nx2 matrices whose rows represent correspondences
%             between 3D and 2D points respectively.
% params: K   Intrinsic parameter matrix.
% params; C/R Camera pose.
%
% return: C/R Camera pose.
%
% Author:  Austin Small

function [C, R] = NonlinearPnP(X, x, K, C, R)
    % Convert 3x3 rotation matrix to quaternion.
    % 1. Find largest squared element.
    q0Square = 0.25 * (1 + R(1, 1) + R(2, 2) + R(3, 3));
    q1Square = 0.25 * (1 + R(1, 1) - R(2, 2) - R(3, 3));
    q2Square = 0.25 * (1 - R(1, 1) + R(2, 2) - R(3, 3));
    q3Square = 0.25 * (1 - R(1, 1) - R(2, 2) + R(3, 3));
    
    % Solve for quaternion elements.
    if ((q0Square > q1Square) && (q0Square > q2Square) && (q0Square > q3Square))
        q0 = sqrt(q0Square);
        q1 = 0.25 * (R(3, 2) - R(2, 3)) / q0;
        q2 = 0.25 * (R(1, 3) - R(3, 1)) / q0;
        q3 = 0.25 * (R(2, 1) - R(1, 2)) / q0;
    elseif ((q1Square > q0Square) && (q1Square > q2Square) && (q1Square > q3Square))
        q1 = sqrt(q1Square);
        q0 = 0.25 * (R(3, 2) - R(2, 3)) / q1;
        q2 = 0.25 * (R(1, 2) + R(2, 1)) / q1;
        q3 = 0.25 * (R(1, 3) + R(3, 1)) / q1;
    elseif ((q2Square > q0Square) && (q2Square > q1Square) && (q2Square > q3Square))
        q2 = sqrt(q2Square);
        q0 = 0.25 * (R(1, 3) - R(3, 1)) / q2;
        q1 = 0.25 * (R(1, 2) + R(2, 1)) / q2;
        q3 = 0.25 * (R(2, 3) + R(3, 2)) / q2;
    else
        q3 = sqrt(q3Square);
        q0 = 0.25 * (R(2, 1) - R(1, 2)) / q3;
        q1 = 0.25 * (R(1, 3) + R(3, 1)) / q3;
        q2 = 0.25 * (R(2, 3) + R(3, 2)) / q3;
    end
            
    opts = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', 'TolX', 0.001, 'TolFun', 0.001, 'MaxFunEvals', 1e64, 'MaxIter', 1e64, 'Display', 'iter');

    params0 = [C; q0; q1; q2; q3];
    fun = @(params) NonLinAlgError(X, x, K, params);        
    output = lsqnonlin(fun, params0, [], [], opts);
    
    % Parse minimized output into C and R.
    C = output(1:3);
    
    q0 = output(4);
    q1 = output(5);
    q2 = output(6);
    q3 = output(7);

    R = [q0^2 + q1^2 - q2^2 - q3^2   2 * (q1*q2 - q0*q3)         2 * (q1*q3 + q0*q2);
         2 * (q1*q2 + q0*q3)         q0^2 - q1^2 + q2^2 - q3^2   2 * (q2*q3 - q0*q1);
         2 * (q1*q3 - q0*q2)         2 * (q2*q3 + q0*q1)         q0^2 - q1^2 - q2^2 + q3^2];
end