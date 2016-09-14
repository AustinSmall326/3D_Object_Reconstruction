% Construct an IxJ binary matrix, V where V(i,j) is one if the jth point
% is visible from the ith camera and zero otherwise.
%
% traj:  NxG matrix whose rows correspond to registered 3D poins, and 
%            whose columns (excluding the first) correspond to camera
%            poses 1 thru G.  The first column stores 3D point ID.
%
% return: V  GxN visibility matrix as described above.
%
% Author:  Austin Small

function [V] = BuildVisibilityMatrix(traj)
    V = traj';
    V(1, :) = [];
end