clear;clc;

laser_map = pcread('0.ply');

tform_init = affine3d([1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1]);

robot_tf{1} = tform_init;
my_robot_tf{1} = tform_init;

for i = 1:1:9
    fprintf("-------Start------\n");
    fprintf("Processing No.%d PC\n",i);
    
    % read
    str = [num2str(i) , '.ply'];
    curr_ply = pcread(str);
    
    % icp TODO
    [tform_init, ~] = pcregistericp(curr_ply, laser_map, 'Metric','pointToPoint', 'InitialTransform', tform_init, 'MaxIterations', 100, 'Tolerance', [0.01, 0.001]);
    clear findNearest;
    [my_tform_init, curr_ply] = my_icp(curr_ply, laser_map, 100, 0, 0.01);
    robot_tf{i+1} = tform_init;
    my_robot_tf{i+1} = my_tform_init;
    
    % merge
    laser_map = pcmerge(laser_map, curr_ply, 0.01);
    fprintf("--------End-------\n");
    
    % debug
    % pcshow(laser_map, 'MarkerSize', 20);
    % pause;
    
end

figure;
pcshow(laser_map, 'MarkerSize', 20);

for i = 2:10
    fprintf("No.%d difference: %f\n", i, mean(mean(abs(robot_tf{i}.T - my_robot_tf{i}))));
end


