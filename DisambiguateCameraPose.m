% Given four camera pose configuration and their triangulated points,
% find the unique camera pose by checking the cheriality condition - the
% reconstructed points must be in front of the cameras.
%
% params: CSet (3x4)   Four configurations of camera centers.
% params: RSet (3x3x4) Four configurations of camera rotations.
% params: XSet (Nx3x4) Four sets of triangulated points from four camera pose
%                      configurations.
%
% return: C/R  The correct camera pose.
% return: X0   The 3D triangulated points from the correct pose.
%
% Author:  Austin Small

% Cset is 3 x 4
% Rset is 3 x 3 x 4
% Xset is Nx3x4

function [C, R, X0] = DisambiguateCameraPose(Cset, Rset, Xset)
    N = size(Xset, 1);     % Number of triangulated points.
    maxPointsInFront = -1;
    normThreshold = 2000;

    % Iterate through the four camera poses.
    for i = 1:4
        tempPointsInFront = 0;
        tempNorm = 0;

        CTemp = Cset(:, i);
        RTemp = Rset(:, :, i);

        % Count number of points in front of camera for a given pose.
        for j = 1:N
            X = [Xset(j, 1, i); Xset(j, 2, i); Xset(j, 3, i)];
            tempNorm = tempNorm + sqrt(Xset(j, 1, i)^2 + Xset(j, 2, i)^2 + Xset(j, 3, i)^2);

            if ((RTemp(3, :) * (X - CTemp)) > 0)
                tempPointsInFront = tempPointsInFront + 1;
            end
        end

        % Check if output should be updated.
        if (tempPointsInFront > maxPointsInFront && tempNorm > normThreshold)
            maxPointsInFront = tempPointsInFront;
            C = CTemp;
            R = RTemp;
            X0 = Xset(:, :, i);
        end
    end
end