% Given E, enumerate four camera pose configurations where C is the camera
% center and R is the rotation matrix.  Note:  P = KR[I -C].
%
% params: E    Essential matrix.
%
% return: CSet Four configurations of camera centers.
% return: RSet Four configurations of camera rotations.
%
% Author:  Austin Small

function [Cset, Rset] = ExtractCameraPose(E)
    [U,~,V] = svd(E);
    
    W = [0 -1 0; 
         1  0 0; 
         0  0 1];
    
    Cset = zeros(3, 4);
    Rset = zeros(3, 3, 4);
        
    % Compute rotations.
    Rset(:, :, 1) = U*W*V';
    Rset(:, :, 2) = U*W*V';
    Rset(:, :, 3) = U*W'*V';
    Rset(:, :, 4) = U*W'*V';
    
    % Compute camera center.
    Cset(:, 1) = -Rset(:, :, 1)' *  U(:, 3);
    Cset(:, 2) = -Rset(:, :, 2)' * -U(:, 3);
    Cset(:, 3) = -Rset(:, :, 3)' *  U(:, 3);
    Cset(:, 4) = -Rset(:, :, 4)' * -U(:, 3);
    
    for i = 1:4
       R = Rset(:, :, i);
       
       if (det(R) < 0)
          Rset(:, :, i) = -R;
          Cset(:, i)    = -Cset(:, i);
       end
    end
end