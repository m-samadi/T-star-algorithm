%%% Find the shortest path between two points by using T*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
clc;

TheNumberOfNodes=100; % The number of all points
W=zeros(TheNumberOfNodes,TheNumberOfNodes); % The distance between all of the points
NeighboringProbability=0.3;
SearchArea_X=1000;
SearchArea_Y=1000;
Location=zeros(TheNumberOfNodes,2); % X | Y 
Start=1; % Start point
Target=100; % Target point
      


%%% Initialization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Location
for i=1:TheNumberOfNodes
    % X
    Location(i,1)=round(rand(1)*SearchArea_X);
    if Location(i,1)==0
        Location(i,1)=1;
    end;

    % Y
    Location(i,2)=round(rand(1)*SearchArea_Y);
    if Location(i,2)==0
        Location(i,2)=1;
    end;    
end;

% Weight
for i=1:TheNumberOfNodes
    for j=1:TheNumberOfNodes
        if i>j
            if rand(1)<=NeighboringProbability
                Weight=round(sqrt((Location(j,1)-Location(i,1))^2+(Location(j,2)-Location(i,2))^2));
                if Weight==0
                    Weight=1;
                end;

                W(i,j)=Weight;
                W(j,i)=Weight;
            end;
        end;
    end;
end;



%%% Find the shortest path between Start and Target
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    

[Path ProcessedNodes Distance]=T_star(TheNumberOfNodes,W,Location,Start,Target);
Start
Target
Path
Distance


