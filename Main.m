%%% Simulation of the T* algorithm and its comparison with the Greedy and A* algorithms
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
clc;

TheNumberOfNodes=10:10:200;
W=zeros(TheNumberOfNodes,TheNumberOfNodes);
NeighboringProbability=0.3;
SearchArea_X=1000;
SearchArea_Y=1000;
Location=zeros(TheNumberOfNodes,2); % X | y 
Start=1;
Cycle=10;

HitRate_Greedy=[];
ProcessedNodes_Greedy=[];
HitRate_A_star=[];
ProcessedNodes_A_star=[];
HitRate_T_star=[];
ProcessedNodes_T_star=[];

for n=1:length(TheNumberOfNodes)
    n
    disp('processing...');
    
    Target=TheNumberOfNodes(n);
    
    CorrectSearch_Greedy=0;
    ProcessedNodes_Sum_Greedy=0;    
    CorrectSearch_A_star=0;
    ProcessedNodes_Sum_A_star=0;
    CorrectSearch_T_star=0;
    ProcessedNodes_Sum_T_star=0;    
    for c=1:Cycle        

        %%% Initialization
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % Location
        for i=1:TheNumberOfNodes(n)
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
        for i=1:TheNumberOfNodes(n)
            for j=1:TheNumberOfNodes(n)
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

        %%% Simulation and comparison
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    

        % Find the real shortest path
        DG=sparse(W);
        [Length,Path]=graphshortestpath(DG,Start,Target);    

        % Find the shortest path with Greedy
        [Path_Greedy ProcessedNodes_Count_Greedy Length_Greedy]=Greedy(TheNumberOfNodes(n),W,Location,Start,Target);
        
        % Find the shortest path with A*
        [Path_A_star ProcessedNodes_Count_A_star Length_A_star]=A_star(TheNumberOfNodes(n),W,Location,Start,Target);
        
        % Find the shortest path with T*
        [Path_T_star ProcessedNodes_Count_T_star Length_T_star]=T_star(TheNumberOfNodes(n),W,Location,Start,Target);        

        ProcessedNodes_Sum_Greedy=ProcessedNodes_Sum_Greedy+ProcessedNodes_Count_Greedy;
        ProcessedNodes_Sum_A_star=ProcessedNodes_Sum_A_star+ProcessedNodes_Count_A_star;
        ProcessedNodes_Sum_T_star=ProcessedNodes_Sum_T_star+ProcessedNodes_Count_T_star;
        
        %/ Comparison between results
        if isequal(Path,Path_Greedy)==1
            CorrectSearch_Greedy=CorrectSearch_Greedy+1;
        end;        
        if isequal(Path,Path_A_star)==1
            CorrectSearch_A_star=CorrectSearch_A_star+1;
        end;
        if isequal(Path,Path_T_star)==1
            CorrectSearch_T_star=CorrectSearch_T_star+1;
        end;        
    end;
    
    HitRate_Greedy=[HitRate_Greedy CorrectSearch_Greedy/Cycle*100];
    ProcessedNodes_Greedy=[ProcessedNodes_Greedy round(ProcessedNodes_Sum_Greedy/Cycle)];    
    HitRate_A_star=[HitRate_A_star CorrectSearch_A_star/Cycle*100];
    ProcessedNodes_A_star=[ProcessedNodes_A_star round(ProcessedNodes_Sum_A_star/Cycle)];
    HitRate_T_star=[HitRate_T_star CorrectSearch_T_star/Cycle*100];
    ProcessedNodes_T_star=[ProcessedNodes_T_star round(ProcessedNodes_Sum_T_star/Cycle)];    
end;

disp(' ');
disp('The simulation process is finished.');

figure(1);
hold on;
plot(TheNumberOfNodes,HitRate_Greedy,':r','LineWidth',2.5);
plot(TheNumberOfNodes,HitRate_A_star,'-.b','LineWidth',2.5);
plot(TheNumberOfNodes,HitRate_T_star,'-k','LineWidth',2.5);
hold off;
xlabel('The number of nodes');
xlim([TheNumberOfNodes(1) TheNumberOfNodes(length(TheNumberOfNodes))]);
ylabel('Hit rate (%)');
legend('Greedy','A*','T*');
grid on;

figure(2);
hold on;
plot(TheNumberOfNodes,ProcessedNodes_Greedy,':r','LineWidth',2.5);
plot(TheNumberOfNodes,ProcessedNodes_A_star,'-.b','LineWidth',2.5);
plot(TheNumberOfNodes,ProcessedNodes_T_star,'-k','LineWidth',2.5);
hold off;
xlabel('The number of nodes');
xlim([TheNumberOfNodes(1) TheNumberOfNodes(length(TheNumberOfNodes))]);
ylabel('The number of processed nodes');
legend('Greedy','A*','T*',2);
grid on;
