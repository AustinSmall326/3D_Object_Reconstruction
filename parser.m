function [matches] = parser()

fileOne   = 'SfMProjectData_1/matching1.txt';
fileTwo   = 'SfMProjectData_1/matching2.txt';
fileThree = 'SfMProjectData_1/matching3.txt';
fileFour  = 'SfMProjectData_1/matching4.txt';
fileFive  = 'SfMProjectData_1/matching5.txt';

% Unique feature ID to track each feature.
featureID = 1;

% Matches matrix, backed by a binary array that indicates if a feature
% is present in a particular image.
matches = zeros(1, 13, 2);

for i = 1:5
    fid = 0;
    
    % File ID.
    if     (i == 1) 
        fid = fopen(fileOne);
    elseif (i == 2) 
        fid = fopen(fileTwo);
    elseif (i == 3) 
        fid = fopen(fileThree);
    elseif (i == 4) 
        fid = fopen(fileFour);
    elseif (i == 5) 
        fid = fopen(fileFive);
    elseif (i == 6) 
        fid = fopen(fileSix);
    end
    
    % Skip first line of txt file and begin reading.
    fgets(fid);
    textLine = fgets(fid);
    
    
    % Iterate through all lines of text file.
    while (ischar(textLine))
        numbers = sscanf(textLine, '%f ');
        lineLength = size(numbers);

        % Parse data into a featureRow.
        featureRow = zeros(1, 13, 2);
        featureRow(1, 1, 1)   = featureID;
        featureRow(1, 2:3, 1) = numbers(5:6);
        featureRow(1, 2:3, 2) = [1 1];

        % Iterate through remaining characters in line.
        for k = 7:3:lineLength
            featureRow(1, 1, 1) = featureID;

            imageID = round(numbers(k));
            featureRow(1, 2 + 2 * (imageID - 1), 1) = numbers(k + 1);
            featureRow(1, 3 + 2 * (imageID - 1), 1) = numbers(k + 2);

            % Update binary backing matrix.
            featureRow(1, 2 + 2 * (imageID - 1), 2) = 1;
            featureRow(1, 3 + 2 * (imageID - 1), 2) = 1;
        end

        matches = cat(1, matches, featureRow);

        textLine = fgets(fid); % Load next line.
        featureID = featureID + 1;
    end
end
end