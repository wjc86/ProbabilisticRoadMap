classdef ProbabilisticRoadMap
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        Graph
        BinOccMap
        Npts
        MaxConnectDist
        edge_checker_pts
    end

    methods
        function obj = ProbabilisticRoadMap(BinOccMap,Npts,MaxConnectDist,edge_checker_pts)
            %UNTITLED2 Construct an instance of this class
            %   Detailed explanation goes here
            arguments
                BinOccMap binaryOccupancyMap = binaryOccupancyMap(zeros(100))%if no map provided, empty 100x100 map used at default scale
                Npts {mustBeInteger(Npts),mustBeGreaterThanOrEqual(Npts,2)}= 2; %need at least two points to avoid jamming up or writing a lot of special case code
                MaxConnectDist {mustBeNumeric,mustBeNonnegative}=50;
                edge_checker_pts {mustBeInteger,mustBeNonnegative}= 2*ceil(norm(BinOccMap.GridSize)); %worst case line is along the diagonal; we want double the number of cells along this axis
            end
            obj.BinOccMap=BinOccMap;
            obj.Npts=Npts;
            obj.MaxConnectDist=MaxConnectDist;
            obj.edge_checker_pts=edge_checker_pts;
            obj=obj.setGraph();
        end

        function obj=setGraph(obj)
            %randomly sample from the rectangle defined by the World
            %COORDINATE SYSTEM of the binary occupancy map
            Pts=[];
            tries=1;
            MaxTries=20;
            while size(Pts,1)<obj.Npts && tries<=MaxTries
%                 x=randi(obj.BinOccMap.XWorldLimits,[obj.Npts 1]);
%                 y=randi(obj.BinOccMap.YWorldLimits,[obj.Npts 1]);
                x=obj.BinOccMap.XWorldLimits(1)+(obj.BinOccMap.XWorldLimits(2)-obj.BinOccMap.XWorldLimits(1))*rand(obj.Npts,1);
                y=obj.BinOccMap.YWorldLimits(1)+(obj.BinOccMap.YWorldLimits(2)-obj.BinOccMap.YWorldLimits(1))*rand(obj.Npts,1);
                samps=[x y];
                Pts=[Pts;samps(obj.BinOccMap.checkOccupancy(samps)==0,:)];%reject points in obstacles (or out of bounds, though that shouldn't happen w/ rectangular maps)
                tries=tries+1;
            end
            Pts=Pts(1:obj.Npts,:); %delete excess pts
            
            %now we have a full field of valid points; we need to connect
            %the graph, deleting edges that cross boundaries

            M=pdist2(Pts,Pts);
            A=M<obj.MaxConnectDist;
            
            g=graph(A,table(Pts),'omitselfloops');

            
            edges_to_rm=[];
            for i=1:size(g.Edges.EndNodes,1) %this loop is a speed bottleneck. is there a faster way
                e=g.Edges.EndNodes(i,:);
                s_crd=g.Nodes(e(1),:).Pts;
                t_crd=g.Nodes(e(2),:).Pts;
                
                x=linspace(s_crd(1),t_crd(1),obj.edge_checker_pts);
                y=linspace(s_crd(2),t_crd(2),obj.edge_checker_pts);
                test_pts=[x; y]';
                edge_occ=obj.BinOccMap.checkOccupancy(test_pts);
                if sum(edge_occ)>0
                    edges_to_rm=[edges_to_rm;e];
                end
            end
            
            g=g.rmedge(edges_to_rm(:,1),edges_to_rm(:,2))
%             for i=1:size(edges_to_rm,1)
%                 g=rmedge(g,edge(1),e(2));
% 
%             end
            obj.Graph=g;

        end

        
        function obj=getPath(obj,startCrd,GoalCrd)
            disp('TODO')
        end
        function plot(obj)
            disp('TODO')
        end
    end
end