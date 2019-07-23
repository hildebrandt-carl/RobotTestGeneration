classdef myPRM  
    properties
        V
        E
        map
        total_nodes
    end
    methods
        function obj = myPRM(map_in, node_num)
            obj.map = map_in;
            obj.total_nodes = node_num;
            obj.V = zeros(2,node_num);
            obj.E = [];
        end
        
        function finished = plan(obj)
            for i = 1:obj.total_nodes
                % Generate a random positon
                
                X = rand() * length(obj.map(:,1));
                Y = rand() * length(obj.map(1,:));
                while(obj.map(ceil(X),ceil(Y)) ~= 0)
                    disp(X)
                    disp(Y)
                    disp("--------------")
                    X = rand() * length(obj.map(:,1));
                    Y = rand() * length(obj.map(1,:));
                end
                obj.V(:,i) = [X,Y];
            end
            % Add the node to the graph
            finished = true;
        end
        
        function v = getVerticies(obj)
            v = obj.V;
        end
    end
end

