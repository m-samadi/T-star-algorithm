%%% T* algorithm
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Path ProcessedNodes Length] = T_star(TheNumberOfNodes,W,Location,Start,Target)
    Alpha=0.3;
    Beta=0.4;
    Gamma=0.3;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ProcessedNodes=1;    
    
    CloseSet=[];
    CloseSet_Index=0;
    
    OpenSet(1,1)=Start;
    OpenSet(1,2)=0;
    OpenSet_Index=1;
    
    TravelledPaths=[];
    TravelledPaths_Index=0;
    TravelledPaths_Rows_Index=[];
    
    Parent=[]; % Parent ID of each node
    G=zeros(TheNumberOfNodes,1); % The travelled distance on each discovered path
   
    while OpenSet_Index~=0
        % Select the node with the least value f
        Min_Node_ID=OpenSet(1,1);
        Min_Value=OpenSet(1,2);
        for i=2:OpenSet_Index
            if OpenSet(i,2)<Min_Value
                Min_Node_ID=OpenSet(i,1);
                Min_Value=OpenSet(i,2);                
            end;
        end;
        Current=Min_Node_ID;
        
        % Add the current node to travelled paths
        if TravelledPaths_Index~=0
            for i=1:TravelledPaths_Index
                if TravelledPaths(i,TravelledPaths_Rows_Index(i))==Parent(Current)                    
                    TravelledPaths_Index=TravelledPaths_Index+1;                    
                    for j=1:TravelledPaths_Rows_Index(i)
                        TravelledPaths(TravelledPaths_Index,j)=TravelledPaths(i,j);
                    end;
                    TravelledPaths_Rows_Index(TravelledPaths_Index)=TravelledPaths_Rows_Index(i)+1;
                    TravelledPaths(TravelledPaths_Index,TravelledPaths_Rows_Index(TravelledPaths_Index))=Current;
                    
                    break;
                end;
            end;
        else
            TravelledPaths_Index=1;
            TravelledPaths(TravelledPaths_Index,1)=Current;
            TravelledPaths_Rows_Index(TravelledPaths_Index)=1;
        end;
        
        % Remove the current node from OpenSet
        for i=1:OpenSet_Index
            if OpenSet(i,1)==Current
                for j=i:(OpenSet_Index-1)
                    OpenSet(j,1)=OpenSet(j+1,1);
                    OpenSet(j,2)=OpenSet(j+1,2);
                end;
                OpenSet_Index=OpenSet_Index-1;
            
                break;
            end;
        end;

        % Add the current node to CloseSet
        Found=0;
        for i=1:CloseSet_Index
            if CloseSet(i)==Current
                Found=1;
            end;
        end;        
        if Found==0
            CloseSet_Index=CloseSet_Index+1;
            CloseSet(CloseSet_Index)=Current;
        end;
            
        % Whether the current node is the target node, or not
        if Current==Target
            for i=1:TravelledPaths_Index
                if TravelledPaths(i,TravelledPaths_Rows_Index(i))==Current                                       
                    for j=1:TravelledPaths_Rows_Index(i)
                        Path(j)=TravelledPaths(i,j);
                    end;
                    Length=G(Current);
                    
                    OpenSet_Index=0;
                    break;
                end;
            end;            
        else            
            % Add not-travelled neighbors of the current node to OpenSet
            for i=1:TheNumberOfNodes
                if W(Current,i)~=0
                    Neighbor=i;                    
                    Found=0;                    
                    
                    % Search the neighbor node in OpenSet
                    for j=1:OpenSet_Index
                        if OpenSet(j,1)==Neighbor
                            Found=1;
                        end;
                    end;  
                    
                    % Add the neighbor node to OpenSet, if it is not the start node and is not existed in OpenSet
                    if (Neighbor~=Start)&&(Found==0)
                        Parent(Neighbor)=Current;                    
                        G(Neighbor,1)=G(Current,1)+W(Current,Neighbor);

                        OpenSet_Index=OpenSet_Index+1;
                        OpenSet(OpenSet_Index,1)=Neighbor;                    
                        x0=Location(Neighbor,1);
                        y0=Location(Neighbor,2);                        
                        x1=Location(Start,1);
                        y1=Location(Start,2);
                        x2=Location(Target,1);
                        y2=Location(Target,2);                        
                        h1=abs((y2-y1)*x0-(x2-x1)*y0+x2*y1-y2*x1)/sqrt((y2-y1)^2+(x2-x1)^2);                        
                        h2=sqrt((x2-x0)^2+(y2-y0)^2);                        
                        OpenSet(OpenSet_Index,2)=Alpha*G(Neighbor,1)+Beta*h1+Gamma*h2;
                        
                        ProcessedNodes=ProcessedNodes+1;
                    end;
                end;
            end;
        end;
    end;
