function [tform,PC1Transformed] = my_icp(PC1,PC2,maxIteration,tolerance,fianlDist_th)
% MY_ICP
% [Input]
% pc1 : current point cloud -> new pc [src]
% pc2 : last point cloud    -> old pc [tar]
% [Output]
% Transform_acc : the transform matrix that fits "pc2 = R * pc1 + t"
% tform : affine3d(Transform_acc)

%% Initialization
pc1 = PC1.Location;
pc2 = PC2.Location;
Transform_acc = eye(4);
pre_error = 0.0;

%% ICP main iterations
for i = 1:maxIteration
    [pc1_indices, pc2_indices, distances] = findNearest(pc1,pc2,fianlDist_th);
    curr_T = getTransform(pc1(pc1_indices,:), pc2(pc2_indices,:));
    curr_R = curr_T(1:3,1:3);
    curr_t = curr_T(1:3,4);

    Transform_acc = curr_T * Transform_acc;
    pc1 = transpose(curr_t + curr_R * pc1');
    
    curr_error = mean(distances);
    if (abs(pre_error - curr_error) < tolerance)
        fprintf("It takes me %d steps to finish iteration\n", i);
        break;
    end
    fprintf("No.%d iteration, Improvement: %.15f\n", i, abs(pre_error - curr_error));
    pre_error = curr_error;
end

%% Extract features process
mask1 = (pc1(:,2) > 30) & (pc1(:,2) < 50)  & (pc1(:,1) > -0.2) & (pc1(:,1) < 2.2); 
points1 = pc1(mask1,1:2);
y1 = mean(points1(:,2));
mask2 = (pc2(:,2) > 35) & (pc2(:,2) < 40)  & (pc2(:,1) > -0.2) & (pc2(:,1) < 2.2);
points2 = pc2(mask2,1:2);
y2 = mean(points2(:,2));
% pc1 = pc1 + [0, y2 - y1, 0];

%% Produce tform
curr_T = [eye(3),[0; y2 - y1; 0];...
          [0,0,0,1]];
Transform_acc = curr_T * Transform_acc;
R = Transform_acc(1:3,1:3);
t = Transform_acc(1:3,4);
pc1 = transpose([0; y2 - y1; 0] + eye(3) * pc1');
tformMatrix = [R, zeros(3,1);...
               t', 1];
tform = affine3d(tformMatrix);
PC1Transformed = pointCloud(pc1);

end