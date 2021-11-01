clear;clc;
clear findNearest;

PC1 = pcread('0.ply');

%% PC1 -> R,t -> PC2
eul = [0 0 0];
R = eul2rotm(eul);
t = [1,10,0];
tform = rigid3d(R,t);
T_real = tform.T;
PC2 = pctransform(PC1,tform); % forward transform
T = getTransform(PC2.Location, PC1.Location);
pc1_ideal = T(1:3,4) + T(1:3,1:3) * (PC2.Location)';
pc1 = (PC1.Location)';

% ICP
[tform_init, new_PC2] = my_icp(PC2, PC1, 100, 1e-50, 0.01);
pc1_icp = tform_init(1:3,4) + tform_init(1:3,1:3) * (PC2.Location)';
disp(mean(mean(abs(pc1 - pc1_icp))));
disp(mean(mean(abs(pc1 - transpose(new_PC2.Location)))));
PC1_icp = pointCloud(pc1_icp');

disp(tform_init - T);

PC_merge = pcmerge(PC1, new_PC2, 0.01);
pcshow(PC_merge, "MarkerSize", 20);

%% Show origin merged PointClouds
laser_map = pcread('0.ply');
for i = 1:9
    str = [num2str(i) , '.ply'];
    laser_map1 = pcread(str);
    laser_map = pcmerge(laser_map, laser_map1, 0.01);
end
pcshow(laser_map, 'MarkerSize', 20);

%% Extract features
pc1 = PC1.Location;
mask = (pc1(:,2) > 35) & (pc1(:,2) < 40) & (pc1(:,1) > -0.2) & (pc1(:,1) < 2.2); 
points = pc1(mask,1:2);
pcshow(PC1, "MarkerSize", 20);
disp(sum(mask));
