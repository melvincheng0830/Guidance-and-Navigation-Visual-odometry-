%--------get the color of 3D points---------%

function[PointColor] = getColor(tracks, allColors, row, col)

PointNum = size(tracks, 2);
PointColor = zeros(PointNum, 3);
for k = 1: 1: PointNum
    ImgIdx = tracks(k).ViewIds(1);
    PixelLocation = tracks(k).Points(1, :);
    x = round(PixelLocation(1));
    y = round(PixelLocation(2));
    if (x <= 1 || x >= row || y <= 1 || y >= col)
        continue;
    end
    colorIdx = sub2ind([row, col], y, x);
    PointColor(k, :) = allColors{ImgIdx}(colorIdx, :)/255;
end

end
