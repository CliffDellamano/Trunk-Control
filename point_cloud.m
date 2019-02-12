%% Blank Workspace

clear
close
clc

%% Load Data

load('Coffee Test Bounding Box0002.mat');
bb_std = Coffee_Test_Bounding_Box0002;
load('Coffee Test Bounding Box Lean0001.mat');
bb_dyn = Coffee_Test_Bounding_Box_Lean0001;


%% Calculate Right Side Point Cloud

shell_pts = bb_std.Trajectories.Labeled.Data(20,:,1);

for n = 2:size(bb_std.Trajectories.Labeled.Data,3)
    
    if abs(bb_std.Trajectories.Labeled.Data(20,1,n)) > max(abs(shell_pts(:,1))) || abs(bb_std.Trajectories.Labeled.Data(20,2,n)) > max(abs(shell_pts(:,2))) ...
                || abs(bb_std.Trajectories.Labeled.Data(20,3,n)) > max(abs(shell_pts(:,3)))
            
            shell_pts = [shell_pts; bb_std.Trajectories.Labeled.Data(20,:,n)];
            
    end
    
end

for n = 2:size(bb_std.Trajectories.Labeled.Data,3)
    
    
    if abs(bb_std.Trajectories.Labeled.Data(21,1,n)) > max(abs(shell_pts(:,1))) || abs(bb_std.Trajectories.Labeled.Data(21,2,n)) > max(abs(shell_pts(:,2))) ...
            || abs(bb_std.Trajectories.Labeled.Data(21,3,n)) > max(abs(shell_pts(:,3)))

        shell_pts = [shell_pts; bb_std.Trajectories.Labeled.Data(21,:,n)];

    end
    
end

%% Calculate Left Side Point Cloud

shell_pts = bb_dyn.Trajectories.Labeled.Data(20,:,1);

for n = 2:size(bb_dyn.Trajectories.Labeled.Data,3)
    
    if abs(bb_dyn.Trajectories.Labeled.Data(20,1,n)) > max(abs(shell_pts(:,1))) || abs(bb_dyn.Trajectories.Labeled.Data(20,2,n)) > max(abs(shell_pts(:,2))) ...
                || abs(bb_dyn.Trajectories.Labeled.Data(20,3,n)) > max(abs(shell_pts(:,3)))
            
            shell_pts = [shell_pts; bb_dyn.Trajectories.Labeled.Data(20,:,n)];
            
    end
    
end

for n = 2:size(bb_dyn.Trajectories.Labeled.Data,3)
    
    
    if abs(bb_dyn.Trajectories.Labeled.Data(21,1,n)) > max(abs(shell_pts(:,1))) || abs(bb_dyn.Trajectories.Labeled.Data(21,2,n)) > max(abs(shell_pts(:,2))) ...
            || abs(bb_dyn.Trajectories.Labeled.Data(21,3,n)) > max(abs(shell_pts(:,3)))

        shell_pts = [shell_pts; bb_dyn.Trajectories.Labeled.Data(21,:,n)];

    end
    
end

%% Find Local Maxima 

for n = 1:size(bb_std.Trajectories.Labeled.Data,3)
    r_hand_pts(n) = norm(reshape(bb_std.Trajectories.Labeled.Data(20,1:3,n),3, 1)');
    l_hand_pts(n) = norm(reshape(bb_std.Trajectories.Labeled.Data(21,1:3,n),3, 1)');
end

r_peaks = islocalmax(r_hand_pts);
l_peaks = islocalmax(l_hand_pts);

%% Plot Volume

plot3(reshape(bb_std.Trajectories.Labeled.Data(20,1,:),1,4800),reshape(bb_std.Trajectories.Labeled.Data(20,2,:),1,4800),reshape(bb_std.Trajectories.Labeled.Data(20,3,:),1,4800))
hold on
scatter3(bb_std.Trajectories.Labeled.Data(20,1,r_peaks),bb_std.Trajectories.Labeled.Data(20,2,r_peaks),bb_std.Trajectories.Labeled.Data(20,3,r_peaks))

plot3(reshape(bb_std.Trajectories.Labeled.Data(21,1,:),1,4800),reshape(bb_std.Trajectories.Labeled.Data(21,2,:),1,4800),reshape(bb_std.Trajectories.Labeled.Data(21,3,:),1,4800))
hold on
scatter3(bb_std.Trajectories.Labeled.Data(21,1,l_peaks),bb_std.Trajectories.Labeled.Data(21,2,l_peaks),bb_std.Trajectories.Labeled.Data(21,3,l_peaks))

%% Interpolate

peaks = [reshape(bb_std.Trajectories.Labeled.Data(20,:,r_peaks),4,size(bb_std.Trajectories.Labeled.Data(20,:,r_peaks),3)) ...
    , reshape(bb_std.Trajectories.Labeled.Data(21,:,l_peaks),4,size(bb_std.Trajectories.Labeled.Data(21,:,l_peaks),3))];


dt = delaunayTriangulation(peaks(1,:)',peaks(2,:)');

tri = dt.ConnectivityList;
xi = dt.Points(:,1);
yi = dt.Points(:,2);

F = scatteredInterpolant(peaks(1,:)',peaks(2,:)',peaks(3,:)');
zi = F(xi,yi);

hold on
trisurf(tri,xi,yi,zi)
view(2)
shading interp
colormap winter

%% Clear Variables

clearvars dt F l_hand_pts r_hand_pts l_peaks r_peaks peaks tri xi yi zi

hold off
figure

%% Find Local Maxima 

for n = 1:size(bb_dyn.Trajectories.Labeled.Data,3)
    r_hand_pts(n) = norm(reshape(bb_dyn.Trajectories.Labeled.Data(20,1:3,n),3, 1)');
    l_hand_pts(n) = norm(reshape(bb_dyn.Trajectories.Labeled.Data(21,1:3,n),3, 1)');
end

r_peaks = islocalmax(r_hand_pts);
l_peaks = islocalmax(l_hand_pts);

%% Plot Volume

plot3(reshape(bb_dyn.Trajectories.Labeled.Data(20,1,:),1,4800),reshape(bb_dyn.Trajectories.Labeled.Data(20,2,:),1,4800),reshape(bb_dyn.Trajectories.Labeled.Data(20,3,:),1,4800))
hold on
scatter3(bb_dyn.Trajectories.Labeled.Data(20,1,r_peaks),bb_dyn.Trajectories.Labeled.Data(20,2,r_peaks),bb_dyn.Trajectories.Labeled.Data(20,3,r_peaks))

plot3(reshape(bb_dyn.Trajectories.Labeled.Data(21,1,:),1,4800),reshape(bb_dyn.Trajectories.Labeled.Data(21,2,:),1,4800),reshape(bb_dyn.Trajectories.Labeled.Data(21,3,:),1,4800))
hold on
scatter3(bb_dyn.Trajectories.Labeled.Data(21,1,l_peaks),bb_dyn.Trajectories.Labeled.Data(21,2,l_peaks),bb_dyn.Trajectories.Labeled.Data(21,3,l_peaks))

%% Interpolate

peaks = [reshape(bb_dyn.Trajectories.Labeled.Data(20,:,r_peaks),4,size(bb_dyn.Trajectories.Labeled.Data(20,:,r_peaks),3)) ...
    , reshape(bb_dyn.Trajectories.Labeled.Data(21,:,l_peaks),4,size(bb_dyn.Trajectories.Labeled.Data(21,:,l_peaks),3))];


dt = delaunayTriangulation(peaks(1,:)',peaks(2,:)');

tri = dt.ConnectivityList;
xi = dt.Points(:,1);
yi = dt.Points(:,2);

F = scatteredInterpolant(peaks(1,:)',peaks(2,:)',peaks(3,:)');
zi = F(xi,yi);

hold on
trisurf(tri,xi,yi,zi)
view(2)
shading interp
colormap autumn


