function [smoothedVector] = VectorSmooth(vector)
% Input:    vector: an array of unsmoothed values
% Output:   smoothedVector: an array of smoothed values
% Author:   Victoria Skeggs
% Date:     7 November 2017

% Find length of the array
[vecLength, ~] = size(vector);

% Create empty array for smoothed values
smoothedVector = zeros(vecLength, 1);

% Deal with beginning point
smoothedVector(1) = (vector(1) + vector(2))/2;

% Smooth each value using two neighbouring values
for i = 2:vecLength-1
    smoothedVector(i) = (vector(i+1) + vector(i) + vector(i-1))/3;
end

% Deal with end point
smoothedVector(vecLength) = (vector(vecLength) + vector(vecLength-1))/2;

end

