function ITAE = FormationEval(numRobots, safe_dist, detectorRange, weights, sampleTime, evalTime)
env = MultiRobotEnv(numRobots);
env.robotRadius = 0.15;
env.showTrajectory = true;
env.showRobotIds = false;
env.plotSensorLines = false;
%% Create robot detectors for all robots
detectors = cell(1,numRobots);

%% Create robot parameter vectors
parameters = cell(1,numRobots);
ranges = cell(1,numRobots);
  k1=1;
  k2=-1;

%% Initialization
for rIdx = 1:numRobots
    detector = RobotDetector(env,rIdx);
    detector.maxDetections = numRobots;
    detector.maxRange = detectorRange;
    detector.fieldOfView =1.5*pi;
    detectors{rIdx} = detector;
    
    param.targetPose=[];
    param.eLast = 1e6;
    param.sideRole = -1;
    param.dTheta = 0;
    %%%% 0930
    param.wiv_1 = [weights(1:5)' weights(6:10)' weights(11:15)' weights(16:20)'];
    param.wov_1 = [weights(21:25); weights(26:30);weights(31:35)];
    param.wiv_2 = param.wiv_1;
    param.wov_2 = param.wov_1;
    
    param.wiw_1 = [weights(36:45)' weights(46:55)' weights(56:65)' weights(66:75)' weights(76:85)' weights(86:95)' weights(96:105)'];
    param.wow_1 = [weights(106:115); weights(116:125);weights(126:135);weights(136:145);weights(146:155);weights(156:165)];
    
    param.wiw_2 = param.wiw_1;
    param.wow_2 = param.wow_1;
    
    param.dux_1 = 0;
    param.duyTheta_1 = 0;
    
    param.v1 = 0;
    param.w1 = 0;

%     param.wix_1 = [weights(1:5)' weights(6:10)' weights(11:15)' weights(16:20)'];
%     param.wox_1 = [weights(21:25); weights(26:30);weights(31:35)];
%     param.wix_2 = param.wix_1;
%     param.wox_2 = param.wox_1;
%     
%     param.wiy_1 = [weights(36:40)' weights(41:45)' weights(46:50)' weights(51:55)' ];
%     param.woy_1 = [weights(56:60); weights(61:65);weights(66:70)];
%     param.wiy_2 = param.wiy_1;
%     param.woy_2 = param.woy_1;
%     
%     param.wiTheta_1 = [weights(71:75)' weights(76:80)' weights(81:85)' weights(86:90)' ];
%     param.woTheta_1 = [weights(91:95); weights(96:100);weights(101:105)];
%     param.wiTheta_2 = param.wiTheta_1;
%     param.woTheta_2 = param.woTheta_1;
%     
%     param.dux_1 = 0;
%     param.duy_1 = 0;
%     param.duTheta_1 = 0;
    
    
    param.xek11 = 0.0; param.xek12 = 0.0;
    param.yek11 = 0.0; param.yek12 = 0.0;
    param.aek11 = 0.0; param.aek12 = 0.0;
    param.ukx11 = 0.0;
    param.uky11 = 0.0;
    param.uka11 = 0.0;
    
    param.kBest = ones(1,9);
    parameters{rIdx} = param;
end

env.robotColors(1,:) = [1 0 0];
env.robotColors(2:numRobots,:) = zeros(numRobots-1,3);
tVec = 0:sampleTime:evalTime;        % Time array
poses = initPose(numRobots, env.robotRadius, 5);
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
e = cell(numel(tVec),numRobots);
ITAE = 0;
totalKBest =  zeros(numel(tVec),numRobots,9);

for idx = 2:numel(tVec)
    parameters{1}.sideRole = 2;
    env(1:numRobots, poses); 
    box on;
    hold on;
    leaderPose =  poses(:,1);
    x0=leaderPose(1);
    y0=leaderPose(2);
    xlabel('X','fontsize',14);
    ylabel('Y','fontsize',14);
%     axis([round(min(poses(1,:))-2) round(max(poses(1,:))+2) round(min(poses(2,:))-2) round(max(poses(2,:))+2)]);

if (idx == 2 || mod(idx,1000) == 0)
          name = sprintf('./NNF1/R_%d_%d.png',numRobots, idx);
%           saveas(1,name);
end


%         name = sprintf('./NNAgg/Image%d.png',idx);
%         saveas(1,name);
    
    b1=y0-k1*x0;
    b2 = y0-k2*x0;
    % Read the sensor and execute the controller for each robot
    for rIdx = 1:numRobots
        detections = step(detectors{rIdx});
        [e{idx,rIdx} , vel(:,rIdx), parameters{rIdx}] = FormationNNC(poses, rIdx, safe_dist, detections, parameters, sampleTime);
        totalKBest(idx,rIdx,:) = parameters{rIdx}.kBest;
%         ITAE = ITAE + sampleTime * idx * abs(norm(e{idx,rIdx}));

% k1*x - y1 + b1 = 0; 领航者左边
%     y2=k2*x+b2;
% k2*x - y2 + b2 = 0; 领航者右边
    end
    % Discrete integration of pose
    poses = poses + vel*sampleTime;
    if rIdx ~= 1
        if poses(1,rIdx) < x0
            ITAE = ITAE + sampleTime * idx * abs(k1*poses(1,rIdx) - poses(2,rIdx) + b1)/sqrt(k1^2+1);
        else
            ITAE = ITAE + sampleTime * idx * abs(k2*poses(1,rIdx) - poses(2,rIdx) + b2)/sqrt(k2^2+1);
        end
    end
     % Update the environment
%    env(1:numRobots, poses, ranges);
%     grid on;
%     hold on;
    
    
%      x=-20:20;
%      y1=k1*x+b1;
    % k1*x - y1 + b1 = 0; 领航者左边
%     y2=k2*x+b2;
     % k2*x - y2 + b2 = 0; 领航者右边

%     xmin = min(poses(1,:))-10;
%     xmax = max(poses(1,:))+10;
%     xlim([xmin xmax]);
%     xlim([-20 30]);   % Without this, axis resizing can slow things down
%     ylim([0 50]);   % Without this, axis resizing can slow things down
%     ymin = min(poses(2,:))-10;
%     ymax = max(poses(2,:))+10;
%     ylim([0 ymax]);
% ylim([0 30]);
% xlim([-10 20]);
% axis([0 200 0 500]);


 for rIdx = 1:numRobots
     % [wi_1, wi_2, wo_1, wo_2, du_1] = UpdateBPWeights(xi, E, Epid, K, I, H, Out, wi_1, wi_2, wo_1, wo_2, du, du_1)
 end


 if idx == 2 || idx == numel(tVec)
        saveas(figure(1), strcat(num2str(idx),'_trace.pdf'));
 end
% Set the motion of the global leader
end

for rIdx = 2:numRobots
    figure(rIdx);
    for k= 1:9
        plot(1:numel(tVec),totalKBest(:,rIdx,k));
        hold on;
    end
    grid on;
    xlabel('Simulation Steps')
    ylabel('Online Optimized Controller Parameters')
    str = {'$k_{xp}$','$k_{xi}$','$k_{xd}$','$k_{yp}$','$k_{yi}$','$k_{yd}$','$k_{\theta p}$','$k_{\theta i}$','$k_{\theta d}$'};
    lgd = legend(str, 'Interpreter','latex', 'FontSize',14, 'Location','east');
    lgd.NumColumns = 1;
    saveas(figure(rIdx), strcat(num2str(rIdx),'_K.pdf'));
end

end