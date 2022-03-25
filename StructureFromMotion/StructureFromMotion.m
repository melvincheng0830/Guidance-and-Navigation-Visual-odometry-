%----------------Structure From Motion From Multiple Views----------------%

% Use |imageDatastore| to get a list of all image file names in a directory.

imageDir = fullfile('');
imds = imageDatastore('Dataset1');

% Load the images and get the color of each pixel and convert them to grayscale.
images = cell(1, numel(imds.Files));
allColors = cell(1, numel(imds.Files));

I = readimage(imds, 1);
row = size(I, 1);
col = size(I, 2);
numPixels = row * col;
for i = 1:numel(imds.Files)
    I = readimage(imds, i);
    allColors{i} = reshape(double(I), [numPixels, 3]);
    images{i} = rgb2gray(I);   
end

% Load the cameraParameters object created using the Camera Calibrator.
data = load(fullfile(imageDir, 'cameraParams.mat'));
cameraParams = data.cameraParams;

% Get intrinsic parameters of the camera
intrinsics = cameraParams.Intrinsics;

% Undistort the first image.
 I = undistortImage(images{1}, intrinsics); 
%I = images{1};
% Detect features. Increasing 'NumOctaves' helps detect large-scale
% features in high-resolution images. Use an ROI to eliminate spurious
% features around the edges of the image.
border = 50;
roi = [border, border, size(I, 2)- 2*border, size(I, 1)- 2*border];
prevPoints   = detectSURFFeatures(I, 'NumOctaves', 8, 'ROI', roi);

% Extract features. Using 'Upright' features improves matching, as long as
% the camera motion involves little or no in-plane rotation.
% 'Upright': the rotation invariance flag
[prevFeatures, valid_corners] = extractFeatures(I, prevPoints, 'Upright', true);

% show the features
figure(1);
imshow(I);
hold on;
plot(valid_corners);

% Create an empty imageviewset object to manage the data associated with each view.
vSet = imageviewset;

% Add the first view. Place the camera associated with the first view
% and the origin, oriented along the Z-axis.
viewId = 1;
vSet = addView(vSet, viewId, rigid3d, 'Points', prevPoints);

%% Add the Rest of the Views
figure(2);
for i = 2:numel(images)
    % Undistort the current image.
     I = undistortImage(images{i}, intrinsics);
    %I = images{i};
    % Detect, extract and match features.
    % or detectBRISKFeatures | detectFASTFeatures | detectORBFeatures | detectSIFTFeatures ...
    currPoints   = detectSURFFeatures(I, 'NumOctaves', 8, 'ROI', roi);
    [currFeatures, valid_corners] = extractFeatures(I, currPoints, 'Upright', true);    
    indexPairs   = matchFeatures(prevFeatures, currFeatures, 'MaxRatio', .7, 'Unique',  true);

    % Select matched points.
    matchedPoints1 = prevPoints(indexPairs(:, 1));
    matchedPoints2 = currPoints(indexPairs(:, 2));
    
    % Estimate the camera pose of current view relative to the previous view.
    % The pose is computed up to scale, meaning that the distance between
    % the cameras in the previous view and the current view is set to 1.
    % This will be corrected by the bundle adjustment.
    [relativeOrient, relativeLoc, inlierIdx] = helperEstimateRelativePose(matchedPoints1, matchedPoints2, intrinsics);
    
    % Get the table containing the previous camera pose.
    prevPose = poses(vSet, i-1).AbsolutePose;
    relPose  = rigid3d(relativeOrient, relativeLoc);
        
    % Compute the current camera pose in the global coordinate system 
    % relative to the first view.
    currPose = rigid3d(relPose.T * prevPose.T);
    
    % Add the current view to the view set.
    vSet = addView(vSet, i, currPose, 'Points', currPoints);

    % Store the point matches between the previous and the current views.
    vSet = addConnection(vSet, i-1, i, relPose, 'Matches', indexPairs(inlierIdx,:));
    
    % Find point tracks across all views.
    tracks = findTracks(vSet);

    % Get the table containing camera poses for all views.
    camPoses = poses(vSet);

    % Triangulate initial locations for the 3-D world points.
    xyzPoints = triangulateMultiview(tracks, camPoses, intrinsics);
    
    % Refine the 3-D world points and camera poses using Bundle Adjustment.
    [xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(xyzPoints,tracks, camPoses, intrinsics, ...
        'FixedViewId', 1, 'PointsUndistorted', true);

    % Store the refined camera poses.
    vSet = updateView(vSet, camPoses);

    prevFeatures = currFeatures;
    prevPoints   = currPoints;  
    
    % Get the color of each reconstructed point
    PointColor = getColor(tracks, allColors, row, col);

    goodIdx = (reprojectionErrors < 5);
    ptCloud = pointCloud(xyzPoints(goodIdx, :), 'Color', PointColor(goodIdx, :));
    %show the camera pose and 3D points
    hold on;
    plotCamera(camPoses(1,:), 'Size', 0.2);
    hold on;
    plotCamera(camPoses(i,:), 'Size', 0.2);
    hold on;
    pcshow(ptCloud, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', 'MarkerSize', 45);
    loc1 = camPoses.AbsolutePose(1).Translation;
    xlim([loc1(1)-5, loc1(1)+4]);
    ylim([loc1(2)-5, loc1(2)+4]);
    zlim([loc1(3)-1, loc1(3)+20]);
    grid on
    hold on;

end

%% Display the refined camera poses and 3-D world points.

% Display camera poses.
camPoses = poses(vSet);
% figure(3);
% plotCamera(camPoses, 'Size', 0.2);
% hold on

% Get the color of each reconstructed point
PointColor = getColor(tracks, allColors, row, col);

% Display the dense 3-D world points.
% Exclude noisy 3-D points.
goodIdx = (reprojectionErrors < 5);
ptCloud = pointCloud(xyzPoints(goodIdx, :), 'Color', PointColor(goodIdx, :));
% Display the 3-D points.
pcshow(ptCloud, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', 'MarkerSize', 45);
grid on
hold off
% Specify the viewing volume.
loc1 = camPoses.AbsolutePose(1).Translation;
xlim([loc1(1)-5, loc1(1)+4]);
ylim([loc1(2)-5, loc1(2)+4]);
zlim([loc1(3)-1, loc1(3)+20]);
camorbit(0, -30);

title('Refined Camera Poses');

%% Compute Dense Reconstruction
% Go through the images again. This time detect a dense set of corners, ...
% and track them across all views usingvision.PointTracker.

% Read and undistort the first image
I = undistortImage(images{1}, intrinsics); 

% Detect corners in the first image.
prevPoints = detectMinEigenFeatures(I, 'MinQuality', 0.001);

% Create the point tracker object to track the points across views.
tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 6);

% Initialize the point tracker.
prevPoints = prevPoints.Location;
initialize(tracker, prevPoints, I);

% Store the dense points in the view set.

vSet = updateConnection(vSet, 1, 2, 'Matches', zeros(0, 2));
vSet = updateView(vSet, 1, 'Points', prevPoints);

% Track the points across all views.
for i = 2:numel(images)
    % Read and undistort the current image.
    I = undistortImage(images{i}, intrinsics); 
    
    % Track the points.
    [currPoints, validIdx] = step(tracker, I);
    
    % Clear the old matches between the points.
    if i < numel(images)
        vSet = updateConnection(vSet, i, i+1, 'Matches', zeros(0, 2));
    end
    vSet = updateView(vSet, i, 'Points', currPoints);
    
    % Store the point matches in the view set.
    matches = repmat((1:size(prevPoints, 1))', [1, 2]);
    matches = matches(validIdx, :);        
    vSet = updateConnection(vSet, i-1, i, 'Matches', matches);
end

% Find point tracks across all views.
tracks = findTracks(vSet);

% Find point tracks across all views.
camPoses = poses(vSet);

% Triangulate initial locations for the 3-D world points.
xyzPoints = triangulateMultiview(tracks, camPoses,...
    intrinsics);

% Refine the 3-D world points and camera poses.
[xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(xyzPoints, tracks, camPoses, intrinsics, ...
    'FixedViewId', 1, 'PointsUndistorted', true);

%% Display the camera poses and the dense point cloud.

% Display the refined camera poses.
figure(4);
plotCamera(camPoses, 'Size', 0.2);
hold on

% Exclude noisy 3-D world points.
goodIdx = (reprojectionErrors < 5);

% Get the color of each reconstructed point
PointColor = getColor(tracks, allColors, row, col);

ptCloud = pointCloud(xyzPoints(goodIdx, :), 'Color', PointColor(goodIdx, :));
% Display the dense 3-D world points.

pcshow(ptCloud, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', 'MarkerSize', 45);
grid on
hold off

% Specify the viewing volume.
loc1 = camPoses.AbsolutePose(1).Translation;
xlim([loc1(1)-5, loc1(1)+5]);
ylim([loc1(2)-5, loc1(2)+5]);
zlim([loc1(3)-1, loc1(3)+10]);
camorbit(0, -30);

title('Dense Reconstruction');
