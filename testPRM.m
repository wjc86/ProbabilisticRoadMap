%%% Base obstacle Map
obs_image = imread('/home/amrlab/Documents/will/maps/testmap3_obs.png');
% image = imresize(image,0.01,'nearest');
obs_grayimage = rgb2gray(obs_image);
obs_bwimage = flipud(obs_grayimage < 1);
res=1;
obs_collision_map = binaryOccupancyMap(flipud(obs_bwimage),res);

Npts=200;
MaxConnectDist=150;

prm=ProbabilisticRoadMap(obs_collision_map,Npts)
figure
prm.BinOccMap.show()
hold on;
prm.Graph.plot(XData=prm.Graph.Nodes.Pts(:,1),YData=prm.Graph.Nodes.Pts(:,2))