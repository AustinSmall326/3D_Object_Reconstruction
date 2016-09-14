% This function extracts correspoinding points from the matches matrix.
% matches matrix
% 
function [featureID, x1, x2] = getCorrespondences(matches, imageOne, imageTwo)
    
    featureIDS     = matches(:, 1, 1);
    imageOnePoints = matches(:, (2 + 2 * (imageOne - 1)) : (3 + 2 * (imageOne - 1)), 1);
    imageTwoPoints = matches(:, (2 + 2 * (imageTwo - 1)) : (3 + 2 * (imageTwo - 1)), 1);
    
    combined = horzcat(featureIDS, imageOnePoints, imageTwoPoints);
    
    combined = combined(all(combined, 2), :);
    
    featureID = combined(:, 1);
    x1        = combined(:, 2:3);
    x2        = combined(:, 4:5);
end