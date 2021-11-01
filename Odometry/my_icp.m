function [Transform_acc,PC1Transformed] = my_icp(PC1,PC2,maxIteration,tolerance,fianlDist_th)
% MY_ICP
% [Input]
% pc1 : current point cloud -> new pc [src]
% pc2 : last point cloud    -> old pc [tar]
% [Output]
% Transform_acc : the transform matrix that fits "pc2 = R * pc1 + t"

% Initialize
pc1 = PC1.Location;
pc2 = PC2.Location;
Transform_acc = eye(4);
pre_error = 0.0;

% ICP main iterations
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

PC1Transformed = pointCloud(pc1);

end