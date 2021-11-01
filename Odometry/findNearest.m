function [pc1_indices,pc2_indices,distances] = findNearest(pc1,pc2,finalDist_th)
% FINDNEAREST
% [Input]
% pc1 : current point cloud -> new pc (n1, 3)
% pc2 : last point cloud    -> old pc (n2, 3)
% [Output]
% pc1_indices : the indices of nearest points in pc1
% pc2_indices : the indices of nearest points in pc2
% distances : the distances between nearest points

%%
% Define three static variables
persistent dist_th;
persistent firstDist_th;
persistent count;

n1 = size(pc1,1);
n2 = size(pc2,1);
dist = zeros(n1,n2);

% compute dist between each pair of point [TODO Broadcast]
for i = 1:n1
    curr_pc1 = pc1(i,1:2);
    for j = 1:n2
        curr_pc2 = pc2(j,1:2);
        dist(i,j) = hypot(curr_pc1(1) - curr_pc2(1), curr_pc1(2) - curr_pc2(2));
    end
end

[min_dist, pc1_indices] = min(dist); % (1,n2)

if isempty(dist_th)
    % fetch 0.2 top dist for 80 percents of points to compute R,t 
    [sorted_dist, ~] = sort(min_dist);
    dist_th = sorted_dist(n2 - floor(0.2 * n2));
    if (dist_th < finalDist_th)
        dist_th = mean(min_dist);
        if (dist_th < finalDist_th)
            finalDist_th = dist_th;
        end
    end
end

if isempty(firstDist_th)
    firstDist_th = dist_th;
end

if isempty(count)
    count = 0;
end
count =  count + 1;

%% Decide dist_th computing formula
% dist_th = median(min_dist);
% dist_th = mean(min_dist);
dist_th = dist_th - (firstDist_th - finalDist_th) / 100; % 1 dim
% dist_th = -((firstDist_th - finalDist_th) / (100 ^ 2)) * (count ^ 2) + firstDist_th; % negative 2 dims
% dist_th = ((firstDist_th - finalDist_th) / (100 ^ 2)) * ((count - 100) ^ 2) + finalDist_th; % positive 2 dims
fprintf("dist_th: %f\n", dist_th);

%% Get final nearest neighbors
mask = min_dist < dist_th;
fprintf("%d/%d dists pass the threshold!\n", sum(mask), size(mask,2));
pc1_indices = pc1_indices(mask);
full_indices = linspace(1,n2,n2);
pc2_indices = full_indices(mask);
distances = min_dist(mask);

end