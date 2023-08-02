classdef ProbabilisticRoadMap
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        Graph
        BinOccMap
        InflOccMap
        InflScale
        Npts
        MaxConnectDist
        edge_checker_pts
        path_crds
        path_nodes
        path_len

        startCrd
        goalCrd
       
    end

    methods
        function obj = ProbabilisticRoadMap(BinOccMap,Npts,MaxConnectDist,edge_checker_pts,inflation_scale)
            %UNTITLED2 Construct an instance of this class
            %   Detailed explanation goes here
            arguments
                BinOccMap binaryOccupancyMap = binaryOccupancyMap(zeros(100))%if no map provided, empty 100x100 map used at default scale
                Npts {mustBeInteger(Npts),mustBeGreaterThanOrEqual(Npts,2)}= 2; %need at least two points to avoid jamming up or writing a lot of special case code
                MaxConnectDist {mustBeNumeric,mustBeNonnegative}=50;
                edge_checker_pts {mustBeInteger,mustBeNonnegative}= 2*ceil(norm(BinOccMap.GridSize)); %worst case line is along the diagonal; we want double the number of cells along this axis
                inflation_scale {mustBeNumeric,mustBeNonnegative} = 2;
            end
            obj.BinOccMap=BinOccMap;
            obj.InflScale=inflation_scale;
            obj.InflOccMap=BinOccMap.copy();
            obj.InflOccMap.inflate(obj.InflScale);

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
                Pts=[Pts;samps(obj.InflOccMap.checkOccupancy(samps)==0,:)];%reject points in obstacles (or out of bounds, though that shouldn't happen w/ rectangular maps)
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
                edge_occ=obj.InflOccMap.checkOccupancy(test_pts);
                if sum(edge_occ)>0
                    edges_to_rm=[edges_to_rm;e];
                end
            end
            
            if ~isempty(edges_to_rm)
                g=g.rmedge(edges_to_rm(:,1),edges_to_rm(:,2));
            end

            %calc edge lengths for weights
            a=g.Nodes.Pts(g.Edges.EndNodes(:,1),:);
            b=g.Nodes.Pts(g.Edges.EndNodes(:,2),:);
            edgelen=vecnorm(b-a,2,2);
            g.Edges.Weight=edgelen;

            obj.Graph=g;

        end

        
        function obj=getPath(obj,startCrd,goalCrd)
            [~,start_mindex]=min(vecnorm(obj.Graph.Nodes.Pts-startCrd,2,2));
            [~,end_mindex]=min(vecnorm(obj.Graph.Nodes.Pts-goalCrd,2,2));
            [nodes,dist]=obj.Graph.shortestpath(start_mindex,end_mindex);
            obj.startCrd=startCrd;
            obj.goalCrd=goalCrd;
            obj.path_crds=[obj.Graph.Nodes.Pts(nodes,:) ;goalCrd];
            obj.path_nodes=nodes;
            obj.path_len=dist;

        end

        function obj=rm_n_nearest_node_and_replan(obj,col_crds,n,pose_crd,goal_crd)
            arguments
                obj;
                col_crds (:,2) {mustBeNumeric}; % crds of collisions
                n {mustBeInteger} = 1;
                
                pose_crd (1,2) {mustBeNumeric}=obj.startCrd;
                goal_crd (1,2) {mustBeNumeric}=obj.goalCrd;
                

            end
            Ds=[];
            Idxs=[];
            uids=[];
            while length(uids)<n || isempty(uids)
                for i=1:size(col_crds,1)
                    [ds,nearest_idxs]=mink(vecnorm(obj.Graph.Nodes.Pts-col_crds(i,:),2,2),n);
                    Ds=[Ds ds'];
                    Idxs=[Idxs nearest_idxs'];
                end
                [d,ids]=mink(Ds,n);
                Ids=Idxs(ids)
                uids=[uids unique(Ids)];
            end
           
            
            
            obj.Graph=obj.Graph.rmnode(uids(1:n));
            obj=obj.getPath(pose_crd,goal_crd);
            
        end
        function plot(obj)
            fig=gcf();
            ax=gca();
            hold on
            obj.BinOccMap.show();
            hold on;
            obj.Graph.plot(XData=obj.Graph.Nodes.Pts(:,1),YData=obj.Graph.Nodes.Pts(:,2));
            plot([obj.path_crds(:,1)],[obj.path_crds(:,2)],'r-o')
        end

        function nobj=clone(obj)
            %I named this function 'clone' instead of 'copy' because I'm
            %not totally sure that it follows the matlab standard for
            %object copying. Should be good enough
            
            nobj=ProbabilisticRoadMap();
            props_to_clone=properties(obj);
            for i=1:length(props_to_clone)
                prop=props_to_clone{i};
                nobj.(prop)=obj.(prop);
            end

        end
    end
end