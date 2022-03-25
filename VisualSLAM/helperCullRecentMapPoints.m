function mapPoints = helperCullRecentMapPoints(mapPoints, keyFrames, newPointIdx)

for i = 1: numel(newPointIdx)
    idx =  newPointIdx(i);
    % If a map point is observed in less than 3 key frames, drop it
    if numel(mapPoints.Observations{idx, 1})< 3 &&...
            max(mapPoints.Observations{idx, 1}) < keyFrames.Views.ViewId(end)
        mapPoints = updateValidity(mapPoints, idx, false);
    end
end
end