function [featureID, x1, x2] = extractInliers(cellArray, imageOne, imageTwo)
    featureID = cellArray{imageOne, imageTwo}(:, 1);
    x1        = cellArray{imageOne, imageTwo}(:, 2:3);
    x2        = cellArray{imageOne, imageTwo}(:, 4:5);
end