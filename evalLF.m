
function evalLF()
a = 3;
numRobots = 11;
steps =3000;
popSize = 20;
eListSize = popSize *  0.1;

env = MultiRobotEnv(numRobots);
env.robotRadius = 0.15;
env.showTrajectory = false;
env.showRobotIds = false;
env.plotSensorLines = false; % So the sensor lines don't dominate the visuals
% numTeams = numRobots;
% env.robotColors = repmat(hsv(numTeams),[ceil(numRobots/numTeams) 1]);
% env.robotColors = env.robotColors(1:numRobots,:); % Truncate colors in case there are unequal teams
% load exampleMap
% map = binaryOccupancyMap(50,50,2);
% x = [1.2; 2.3; 3.4; 4.5; 5.6];
% y = [5.0; 4.0; 3.0; 2.0; 1.0];
% setOccupancy(map, [x y], ones(5,1));
% env.mapName = 'map';

%% Create robot detectors for all robots
detectors = cell(1,numRobots);

%% Create robot parameter vectors
parameters = cell(1,numRobots);
ranges = cell(1,numRobots);

%% Initialization
for rIdx = 1:numRobots
    detector = RobotDetector(env,rIdx);
    detector.maxDetections = numRobots;
    detector.maxRange = 200;
    detector.fieldOfView =2*pi;
    detectors{rIdx} = detector;
    
    param.idealPose=[];
    param.sideRole = -1;
    param.dTheta = 0;
    param.xek11 = 0.0; param.xek12 = 0.0;
    param.yek11 = 0.0; param.yek12 = 0.0;
    param.aek11 = 0.0; param.aek12 = 0.0;
    param.ukx11 = 0.0;
    param.uky11 = 0.0;
    param.uka11 = 0.0;
    param.uv11 = 0.0;
    param.uw11 = 0.0;
    
    param.kBest = ones(1,9);
    param.sumEX = 0;
    param.sumEY = 0;
    param.sumETheta = 0;
    
    param.kPopList = [0.1+1.9*rand(popSize,9) 1e3*ones(popSize,1)] ;
    %     param.lidar = LidarSensor;
    %     param.lidar.scanAngles = linspace(-pi,pi,18);
    %     param.lidar.maxRange = 5;
    %     attachLidarSensor(env,rIdx,param.lidar);
    parameters{rIdx} = param;
end

env.robotColors(1,:) = [1 0 0];
env.robotColors(2:numRobots,:) = zeros(numRobots-1,3);
sampleTime = 0.1;              % Sample time [s]
tVec = 0:sampleTime:steps;        % Time array
poses = initPose(numRobots, env.robotRadius, 5);
% poses = [10*(rand(2,numRobots) - 0.5); ...
%          pi*rand(1,numRobots)];

for rIdx = 2:numRobots
    pose = poses(:,rIdx);
    if pose(1) > poses(1,1)
        parameters{rIdx}.sideRole = 1;
    else
        parameters{rIdx}.sideRole = 0;
    end
end

%% Simulation loop
vel = zeros(3,numRobots);
e = zeros(numel(tVec),numRobots);
% totalKBest =  zeros(numel(tVec),numRobots,9);

for idx = 2:numel(tVec)
    parameters{1}.sideRole = 2;
    env(1:numRobots, poses);
    
    % Read the sensor and execute the controller for each robot
    for rIdx = 1:numRobots
        detections = step(detectors{rIdx});
        %         scans =parameters{rIdx}.lidar(poses(:,rIdx));
        %         ranges{rIdx} = scans;
        [e(idx,rIdx) , vel(:,rIdx), parameters{rIdx}] = swarmController(poses,rIdx,detections,parameters);
        %         totalKBest(idx,rIdx,:) = parameters{rIdx}.kBest;
    end
    % Discrete integration of pose
    poses = poses + vel*sampleTime;
    % Update the environment
    %    env(1:numRobots, poses, ranges);
    grid on;
    hold on;
    
    xmin = min(poses(1,:))-10;
    xmax = max(poses(1,:))+10;
    xlim([xmin xmax]);
    %     xlim([0 1.2*numRobots]);   % Without this, axis resizing can slow things down
    
    ymin = min(poses(2,:))-10;
    ymax = max(poses(2,:))+10;
    ylim([0 ymax]);
    % ylim([0 30]);
    % xlim([-10 20]);
    % axis([0 200 0 500]);
    xlabel('X (m)')
    ylabel('Y (m)')
    if idx == 2 || idx == numel(tVec)
        saveas(figure(1), strcat(num2str(idx),'_trace.pdf'));
    end
    % Set the motion of the global leader
end
%

% for rIdx = 2:numRobots
%     figure(rIdx);
%     for k= 1:9
%         plot(1:numel(tVec),totalKBest(:,rIdx,k));
%         hold on;
%     end
%     grid on;
%     xlabel('Simulation Steps')
%     ylabel('Online Optimized Controller Parameters')
%     str = {'$k_{xp}$','$k_{xi}$','$k_{xd}$','$k_{yp}$','$k_{yi}$','$k_{yd}$','$k_{\theta p}$','$k_{\theta i}$','$k_{\theta d}$'};
%     lgd = legend(str, 'Interpreter','latex', 'FontSize',14, 'Location','east');
%     lgd.NumColumns = 1;
%     saveas(figure(rIdx), strcat(num2str(rIdx),'_K.pdf'));
% end

% grid on;
% axis([0 numel(tVec)-1 0 inf]);
% xlabel('Simulation Steps')
% ylabel('Errors (m)')

% saveas(figure(numRobots+1), 'Errors.pdf');

mean( e(end,2:numRobots))
end