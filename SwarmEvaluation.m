% SwarmEvaluation
%% Fuzzy Controller
% function meanConvHullAera = SwarmEvaluation(isFuzzy, numRobots, Safe_Dist, detectorRange, fis, sampleTime, evalTime)

%% NN Controller
 function meanConvHullAera = SwarmEvaluation(FuzzyOrNN, numRobots, Safe_Dist, detectorRange, controller, sampleTime, evalTime)
    %% Set up Enviroments
    env = MultiRobotEnv(numRobots);
    env.robotRadius = 0.15;
    env.showTrajectory = false;
    env.showRobotIds = false;
    env.plotSensorLines = false;
    env.robotColors = repmat(hsv(10),[ceil(numRobots/10) 1]);
    %% Create robot detectors for all robots
    detectors = cell(1,numRobots);

    for rIdx = 1:numRobots
        detector = RobotDetector(env,rIdx);
        detector.maxDetections = numRobots;
        detector.maxRange = detectorRange;
        detector.fieldOfView =3;
        detectors{rIdx} = detector;
    end

    tVec = 0:sampleTime:evalTime;        % Time array
    poses = initPose(numRobots, env.robotRadius, 5);
   aera2 = 0;
   comp = zeros(size(tVec,2)-1,1);
    %% Simulation loop
    vel = zeros(3,numRobots);
    for idx = 2:numel(tVec)
        % Update the environment
        env(1:numRobots, poses); 
        xlabel('X','fontsize',14);
        ylabel('Y','fontsize',14);
        a = numRobots + 5;
        axis([0 a 0 a]);
        box on;
        
%         k = convhull(poses(1:2,:)');
%         %         
%         aera2 = polyarea(poses(1,k),poses(2,k));
%         axis([0 25 0 25]);
%         hold on;
%        
%         plot(poses(1,k),poses(2,k),'k-');
%         name = sprintf('./NNAgg/Image%d.png',idx);
%         saveas(1,name);
%         children = get(gca, 'children');
%         delete(children(1));
        
%         pause;
        % Read the sensor and execute the controller for each robot
        for rIdx = 1:numRobots
            detections = step(detectors{rIdx});
            if ~isempty(detections)
                [closestRange,closestInds] = min(detections(:,1));
                if closestRange < Safe_Dist
                    w = - detections(closestInds,2);
                    v  =  closestRange/30;
                else
                    range = mean(detections(:,1));
                    dDis = range - Safe_Dist;
                    angle = mean(detections(:, 2));
                    
                    %% Fuzzy Controller:
                    if FuzzyOrNN == 0
                        v  =  range/10;
                        w = -evalfis(controller,[dDis angle]);
                    else
                        %% NN Controller:
                        v  =  dDis;
                        p = [dDis; angle];
                        w = 1.5*sim(controller,p);
                    end
                end
            else
                v =  0.1;
                w = rand;
            end
            pose = poses(:,rIdx);
            vel(:,rIdx) = bodyToWorld([v;0;w],pose);
        end
        % Discrete integration of pose
        poses = poses + vel*sampleTime;
         k = convhull(poses(1:2,:)');
        aera2 = aera2 + polyarea(poses(1,k),poses(2,k));
%         idx
        polyarea(poses(1,k),poses(2,k));
        %Calculate the minimal aera of convex hull
%         X = poses(1:2,:);
%         mD = min(pdist(X'));
        %         if mD < 0.3
        %             meanConvHullAera = 1e6;
        %             return
        %         end
        
%         figure(2);
%         plot(poses(1,k),poses(2,k),'r-',poses(1,:),poses(2,:),'b*');
%         axis([0 20 0 20]);
%         if mod(idx,10)==0
%         name = sprintf('Image%d.png',idx);
%         saveas(2,name);
%         end
        %    Axis limits
        %      xmin = round(min(poses(1,:))-10);
        %     xmax = round(max(poses(1,:))+10);
        %     xlim([xmin xmax]);
        % %     xlim([0 1.2*numRobots]);   % Without this, axis resizing can slow things down
        %
        %     ymin = round(min(poses(2,:))-10);
        %     ymax = round(max(poses(2,:))+10);
        %     ylim([0 ymax]);
        
        %     grid on;
%             hold on;

    end
        %         aera2 = aera2 + polyarea(poses(1,k),poses(2,k));
%        meanConvHullAera = polyarea(poses(1,k),poses(2,k));
    meanConvHullAera = aera2/(numel(tVec)-1);
   [minIND,minValue] = mink(comp, 1);
end