function animate_spider_gait()
    % animate_spider_gait - Animate a full spider gait from GA output
    %
    % This script loads a saved gait matrix (300x24) and visualizes
    % each pose in sequence using plot_spider_pose.m.
    %
    % Make sure this file, plot_spider_pose.m, and forward_leg_kinematics2.m
    % are all in the same folder, and that you've exported your gait
    % from Python as 'best_gait.mat' or 'best_gait.csv'.

    % ======== Load gait data ========
    if isfile('best_gait.mat')
        data = load('best_gait.mat');
        gait = data.best_gait;
    elseif isfile('best_gait.csv')
        gait = readmatrix('best_gait.csv');
    else
        error('No gait file found. Please export best_gait.mat or best_gait.csv from Python.');
    end

    % Check dimensions
    [num_frames, num_angles] = size(gait);
    if num_angles ~= 24
        error('Expected gait to be Nx24, but got %dx%d.', num_frames, num_angles);
    end

    fprintf('Animating spider gait with %d frames...\n', num_frames);

    % ======== Animation loop ========
    figure('Color','w');
    for frame = 1:num_frames
        clf; % Clear previous frame
        plot_spider_pose(gait(frame, :));
        title(sprintf('Spider Gait Animation - Frame %d of %d', frame, num_frames), 'FontSize', 14);
        drawnow; % Render frame immediately
        pause(0.3); % Adjust playback speed (smaller = faster)
    end

    fprintf('Animation complete!\n');
end
