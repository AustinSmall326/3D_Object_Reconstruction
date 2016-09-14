function  [Match ,feature_cell, K_matrix] = generate_matching_feature_cell(match_dir_string)
%This function can be called with:
%{
match_dir_string = 'SfMProjectData_1';
[Match ,feature_cell, K_matrix] = generate_matching_feature_cell(match_dir_string);
%}
%Note: this function must be in same directory as python file 'matching_feature_reader.py'
%INPUT: This function takes in the string of the directory name where both
%the matching.txt are stored as well as the calibration matrix K from calibration.txt
%OUTPUT: 
%K_matrix: calibration matrix from the txt file
%feature_cell: cell matrix containing the name and matches for each text file respectively
%Match: cell matrix, where Match{i,j} is a struct containing the matching pixels between imgs i and j. To access them use (ui,vi) = Match{i,j}.x{i}
%note: that for i,j = 1,2,3,4,5... this notation doesn't change so you can use it in a for loop to parse example:
%{
for idx = 1:5
    for jdx = 1:5
        if jdx > idx
        x1 = Match{i,j}.x{i};
        x2 = Match{i,j}.x{j};
        % then for instance run ransac then repopulate 
        end
    end
end
%}


file =  strcat(match_dir_string, '/calibration.txt');
K_filetext = fileread(file);
eval(K_filetext);
K_matrix = K;

file = strcat(match_dir_string, '/matching*');
match_names = dir(file);

%load matching files, (maybe: round to pixels, then find unique features between all frames)
cmdstr = 'python matching_feature_reader.py';
[status, commandOut] = system(cmdstr);
fprintf(commandOut);
load('feature_list.mat')
feature_cell; %is the name of the cell of all features

sorted_feature_cell = cellmat;
for jdx = 1:length(match_names)
    for idx = 1:length(feature_cell)
        tf = strcmp(match_names(jdx).name(1:9), feature_cell{idx}.match_names);
        if tf == 1
            sorted_feature_cell{jdx} = feature_cell{idx};
        end
    end
end
feature_cell = sorted_feature_cell;


for idx = 1:length(feature_cell)
    for jdx = 1:length(feature_cell{idx}.features_list)
        feature_cell{idx}.features_list{jdx} = str2num(feature_cell{idx}.features_list{jdx}');
    end
end
Match = cellmat;
%correspondences with Img1
Match{1,2}.x{1} = []; Match{1,2}.x{2} = [];
Match{1,3}.x{1} = []; Match{1,3}.x{3} = [];
Match{1,4}.x{1} = []; Match{1,4}.x{4} = [];
Match{1,5}.x{1} = []; Match{1,5}.x{5} = [];
Match{1,6}.x{1} = []; Match{1,6}.x{6} = [];
%correspondences with Img2
Match{2,3}.x{2} = []; Match{2,3}.x{3} = [];
Match{2,4}.x{2} = []; Match{2,4}.x{4} = [];
Match{2,5}.x{2} = []; Match{2,5}.x{5} = [];
Match{2,6}.x{2} = []; Match{2,6}.x{6} = [];
%correspondences with Img3
Match{3,4}.x{3} = []; Match{3,4}.x{4} = [];
Match{3,5}.x{3} = []; Match{3,5}.x{5} = [];
Match{3,6}.x{3} = []; Match{3,6}.x{6} = [];
%correspondences with Img4
Match{4,5}.x{4} = []; Match{4,5}.x{5} = [];
Match{4,6}.x{4} = []; Match{4,6}.x{6} = [];
%correspondences with Img5
Match{5,6}.x{5} = []; Match{5,6}.x{6} = [];
for idx = 1:length(feature_cell)-1
    for jdx = 1:length(feature_cell{idx}.features_list)
        %iterate through all, and build matching list
        %                 keyboard
        for k_first_dx = idx:length(feature_cell)
            for k_last_dx = 2:(length(feature_cell)+1)
                if k_last_dx > k_first_dx
                    
                    [~,test_last]= find(feature_cell{idx}.features_list{jdx} == k_last_dx);
                    if ~isempty(test_last)
                        
                        if length(test_last) == 2
                            if test_last(2) > 4 && mod((test_last(2) - 6),3) == 1
                                Match{k_first_dx,k_last_dx}.x{k_first_dx}(end+1,:) = feature_cell{idx}.features_list{jdx}(5:6);
                                Match{k_first_dx,k_last_dx}.x{k_last_dx}(end+1,:) = feature_cell{idx}.features_list{jdx}(test_last(2)+1:test_last(2)+2);
                            end
                        elseif test_last > 4 && mod((test_last - 6),3) == 1
                            Match{k_first_dx,k_last_dx}.x{k_first_dx}(end+1,:) = feature_cell{idx}.features_list{jdx}(5:6);
                            Match{k_first_dx,k_last_dx}.x{k_last_dx}(end+1,:) = feature_cell{idx}.features_list{jdx}(test_last+1:test_last+2);
                        end
                    end
                end
            end
        end
    end
end

%% The following can be used with users ransac for first step of algorithm: 
%{
Step 1: Reject outlier Correspodences
1. for all possible pair of images do
2: [x1 x2] = GetInliersRANSAC(x1, x2);  %Reject outlier correspondences.
3: end for
%}
%{
for idx = 1:length(feature_cell)-1
    for jdx = 1:length(feature_cell)
        k_last_dx = jdx; k_first_dx = idx; %intuitive naming convention
        if k_last_dx > k_first_dx
            if ~isempty(Match{k_first_dx,k_last_dx}.x{k_first_dx}) && ~isempty(Match{k_first_dx,k_last_dx}.x{k_last_dx})
                x1 = Match{k_first_dx,k_last_dx}.x{k_first_dx};
                x2 = Match{k_first_dx,k_last_dx}.x{k_last_dx};
                [y1,y2,ID] = GetInliersRANSAC(x1,x2, k_first_dx, k_last_dx);
                Match{k_first_dx,k_last_dx}.x{k_first_dx} = y1;
                Match{k_first_dx,k_last_dx}.x{k_last_dx} = y2;
            end
        end
    end
end
%}