% Given F, estimate E = K'FK
%
% params: K   3x3 camera intrinsic parameter
% params: F   fundamental matrix
%
% return: E   3x3 essential matrix with singular values (1, 1, 0).
%
% Author:  Austin Small

function [E] = EssentialMatrixFromFundamentalMatrix(F, K)
    E = K' * F * K;
    
    [U,D,V] = svd(E);
        
    E = U * diag([1 1 0]) * V';
end