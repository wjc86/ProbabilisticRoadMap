clearvars
close all

%%% Base obstacle Map
obs_image = imread('/home/amrlab/Documents/will/maps/testmap3_obs.png');
% image = imresize(image,0.01,'nearest');
obs_grayimage = rgb2gray(obs_image);
obs_bwimage = flipud(obs_grayimage < 1);
res=1;
obs_collision_map = binaryOccupancyMap(flipud(obs_bwimage),res);

Npts=200;
MaxConnectDist=150;

startCrd=[20 20];
goalCrd=[90 20];

locs=[50 30; 40 25; 60 50];

prm=ProbabilisticRoadMap(obs_collision_map,Npts)

prm=prm.getPath(startCrd,goalCrd);

figure
prm.plot()

prm=prm.rm_n_nearest_node_and_replan(locs,30);
figure
prm.plot()
