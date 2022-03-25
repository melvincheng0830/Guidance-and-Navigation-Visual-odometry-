function [H, score, inliersIndex] = helperComputeHomography(matchedPoints1, matchedPoints2)

[H, inlierPoints1, inlierPoints2] = estimateGeometricTransform( ...
    matchedPoints1, matchedPoints2, 'projective', ...
    'MaxNumTrials', 1e3, 'MaxDistance', 4, 'Confidence', 90);

[~, inliersIndex] = intersect(matchedPoints1.Location, ...
    inlierPoints1.Location, 'row', 'stable');

locations1 = inlierPoints1.Location;
locations2 = inlierPoints2.Location;
xy1In2     = transformPointsForward(H, locations1);
xy2In1     = transformPointsInverse(H, locations2);
error1in2  = sum((locations2 - xy1In2).^2, 2);
error2in1  = sum((locations1 - xy2In1).^2, 2);

outlierThreshold = 6;

score = sum(max(outlierThreshold-error1in2, 0)) + ...
    sum(max(outlierThreshold-error2in1, 0));
end