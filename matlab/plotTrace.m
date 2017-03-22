%% plotTrace plots the trace of distributed mapper
%  arguments: 
%  filename = tracefilename used in the code for saving trace
%  nrRobots = number of robots

close all; clc;

%% Config
nrRobots = 4; % Number of robots
filename = '/tmp/testdistributedEstimation_4robots'; % Trace filename


%% Plot
legendmatrix=cell(nrRobots,1);
color = jet(nrRobots);

set(0,'defaultLineLineWidth',2);   % set the default line width to lw
set(0,'DefaultFigureWindowStyle','normal');
set(0,'DefaultAxesFontSize',15)

%% Iterate over robots and plot the errors
for robot = 0:nrRobots -1    
    
    %% Robot_i trace file
    trace_file = sprintf('%s_%d.txt',filename, robot);
    
    %% Read the file
    trace = dlmread(trace_file, ' ');
    fprintf('Robot: %d, Distributed Error: %f, Centralized Error: %f\n', robot, trace(3,1), trace(4,1));    
    residualTrace = trace(1:2,:);
    [rows, cols] = size(trace);    

    %% Linear Rotation Graph Error
    end_index = find(trace(1,:) == -1);
    iterations = 0:end_index-2;
    figure(1); plot(iterations, log10(residualTrace(1,1:end_index-1)), 'Color', color(robot+1,:));
    le=ylabel('Linear Rotation Graph: $\chi^2$ Error (log)');
    set(le,'Interpreter','Latex')
    le = xlabel('\#Iteration');
    set(le,'Interpreter','Latex') 
    axis tight;
    box off;
    xlim([-5, 200]);
    hold on;    
    
    %% Linear Pose Graph Error
    end_index = find(trace(2,:) == -1);
    iterations = 0:end_index-2;
    figure(2); plot(iterations, log10(residualTrace(2,1:end_index-1)), 'Color', color(robot+1,:));
    le=ylabel('Linear Pose Graph: $\chi^2$ Error (log)');
    set(le,'Interpreter','Latex')
    le = xlabel('\#Iteration');
    set(le,'Interpreter','Latex')    
    axis tight;
    box off;
    xlim([-5,200]);
    ylim([-0.6,0.6]);
    hold on;    
    
    %% Change in Rotation Estimate
    estimateChangeTrace = trace(5:6,:);
    end_index = find(estimateChangeTrace(1,:) == -1);
    iterations = 1:end_index-1;
    figure(3); plot(iterations,log10(estimateChangeTrace(1,1:end_index-1)), 'Color', color(robot+1,:));
    le = ylabel('Linear Rotation Estimate Change (log)');
    box off;
    set(le,'Interpreter','Latex')        
    le = xlabel('\#Iteration');
    set(le,'Interpreter','Latex')    
        xlim([-5,200]);
    hold on;    
    
    %% Change in Pose Estimate
    end_index = find(estimateChangeTrace(2,:) == -1);
    iterations = 1:end_index-1;
    figure(4); plot(iterations, log10(estimateChangeTrace(2,1:end_index-1)), 'Color', color(robot+1,:));
    le = ylabel('Linear Pose Estimate Change (log)');
    box off;
    set(le,'Interpreter','Latex')            
    le = xlabel('\#Iteration');
    set(le,'Interpreter','Latex')        
    xlim([-5,200]);
    hold on;       
        
    legendmatrix{robot+1}=strcat('Robot ',num2str(robot+1));    
end

%% Plot overall graph residuals    
trace_file = sprintf('%s_overall_error.txt',filename)
trace = dlmread(trace_file, ' ');
centralized_error = trace(3,1);
trace = trace(1:2,:);
[rows, cols] = size(trace);

%% Centralized vs Distributed Rotation Error
figure(5); 
centralized_trace_file = sprintf('%s_centralizedRotation.txt',filename);
centralized_rotation = csvread(centralized_trace_file)

end_index = find(trace(1,:) == -1);
iterations = 1:end_index-1;
plot(iterations, (trace(1,1:end_index-1)), 'r'); hold on;
plot(iterations, ones(1,numel(iterations)).*(centralized_rotation), 'g');
le=ylabel('$\chi^2$ Error', 'FontSize', 30, 'FontWeight', 'bold');
set(le,'Interpreter','Latex')
le = xlabel('\#Iteration', 'FontSize', 30, 'FontWeight', 'bold');
set(le,'Interpreter','Latex')
legend('Distributed Rotation Error', 'Centralized Rotation Error');
hold on;
box off;
legend boxoff;

%% Centralized vs Distributed Pose Error
figure(6); 
end_index = find(trace(2,:) == -1);
iterations = 0:end_index-2;
plot(iterations, log10(trace(2,1:end_index-1)), 'r'); hold on;
iterations = 0:size(trace(2,:),2)-1;
plot(iterations, ones(1,size(trace(2,:),2)).*log10(centralized_error), 'g');
le=ylabel('$\chi^2$ Error (log)', 'FontSize', 30, 'FontWeight', 'bold');
set(le,'Interpreter','Latex')
le = xlabel('\#Iteration', 'FontSize', 30, 'FontWeight', 'bold');
set(le,'Interpreter','Latex')
lh = legend('Distributed Pose Error', 'Centralized Pose Error');
set(lh, 'FontSize', 18)
hold on;
box off;
legend boxoff;

