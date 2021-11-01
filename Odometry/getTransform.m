function [T] = getTransform(pc1,pc2)
% GETTRANSFORM 
% [Input]
% pc1 : current point cloud -> new pc (n, 3)
% pc2 : last point cloud    -> old pc (n, 3)
% [Output]
% T : the transform matrix that fits "pc2 = R * pc1 + t"

% centralization
centroid_pc1 = pc1 - mean(pc1,1); % (n,3)
centroid_pc2 = pc2 - mean(pc2,1); % (n,3)

% compute R, t
W = centroid_pc1' * centroid_pc2; % (3,3)
[U,~,V] = svd(W);
R = V * U'; % (3,3)
t = mean(pc2,1)' - R * mean(pc1,1)'; % (3,1)

% stack T
T = [R,t];
T = [T;[0,0,0,1]];

end