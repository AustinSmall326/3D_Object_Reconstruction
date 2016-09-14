function [featureIDsout, x, X] = getRelevantGlobalPoints(matches, image, registeredFeatures, globalPoints)
    featureIDS  = matches(:, 1, 1);
    imagePoints = matches(:, (2 + 2 * (image - 1)) : (3 + 2 * (image - 1)), 1);
    imageBin    = matches(:, (2 + 2 * (image - 1)) : (3 + 2 * (image - 1)), 2);

    % Create a binary vector to indicate if the global point has been
    % registered.
    globalBin = ismember(featureIDS, registeredFeatures);
    
    combined = horzcat(featureIDS, imageBin, globalBin);
    
    % Filter for features that have been registered and can be seen
    % by image.
    indices = all(combined, 2);
    
    combined = horzcat(featureIDS, imagePoints);
    combined = combined(indices, :);
    
    % Parse output.
    featureIDsout = combined(:, 1);
    x             = combined(:, 2:3);
    
    % Return global points as well.
    bin = ismember(globalPoints(:, 1), featureIDsout);
    X = globalPoints(bin, 2:4);
    
    [n, bin] = histc(globalPoints(:, 1), unique(globalPoints(:, 1)));
    multiple = find (n > 1);
    index = find(ismember(bin, multiple));
    
    disp(5)
end