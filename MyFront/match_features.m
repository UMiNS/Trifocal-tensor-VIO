
function [matches, confidences] = match_features(features1, features2)

ratio = 0.8;

num_matches = 0;

num_features1 = size(features1,1);

num_features2 = size(features2,1);

matches = zeros(num_features1,2);

confidences = zeros(num_features1);

for i = 1:num_features1

    distances = dist(features1(i,:),features2');

    [dists,index] = sort(distances);

    nndr = dists(1)/dists(2); 

    if nndr < ratio

      %num_matches = num_matches + 1;

      matches(i,:) = [i,index(1)];

      confidences(i) = 1 - nndr;

 %    else 

 %        confidences(i) = [];

%         matches(i,:) = [];

    end

end


if 0
%keep only those matched

matches_index = find(confidences>0);

matches = matches(matches_index,:);

confidences = confidences(matches_index);

 

% Sort the matches so that the most confident onces are at the top of the

% list. You should probably not delete this, so that the evaluation

% functions can be run on the top matches easily.

%[confidences, ind] = sort(confidences, 'descend');

%matches = matches(ind,:);
end
