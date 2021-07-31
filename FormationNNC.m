function [e,vel,param] = FormationNNC(poses,rIdx, safe_dist, detections, parameters, sampleTime)
    % Unpack the robot's pose
param = parameters{rIdx};
parameters{rIdx}.dTheta = poses(3,1);
pose = poses(:,rIdx);
wiv_1 = parameters{rIdx}.wiv_1;
wiw_1 = parameters{rIdx}.wiw_1;
wov_1 = parameters{rIdx}.wov_1;
wow_1 = parameters{rIdx}.wow_1;

wiv_2 = parameters{rIdx}.wiv_2;
wiw_2 = parameters{rIdx}.wiw_2;
wov_2 = parameters{rIdx}.wov_2;
wow_2 = parameters{rIdx}.wow_2;
dux_1 = parameters{rIdx}.dux_1;
duyTheta_1= parameters{rIdx}.duyTheta_1;
v1 = parameters{rIdx}.v1;
w1 = parameters{rIdx}.w1;

% 
% wix_1 = parameters{rIdx}.wix_1;
% wox_1 = parameters{rIdx}.wox_1;
% wiy_1 = parameters{rIdx}.wiy_1;
% woy_1 = parameters{rIdx}.woy_1;
% wiTheta_1 = parameters{rIdx}.wiTheta_1;
% woTheta_1 = parameters{rIdx}.woTheta_1;
% 
% wix_2 = parameters{rIdx}.wix_2;
% wox_2 = parameters{rIdx}.wox_2;
% wiy_2 = parameters{rIdx}.wiy_2;
% woy_2 = parameters{rIdx}.woy_2;
% wiTheta_2 = parameters{rIdx}.wiTheta_2;
% woTheta_2 = parameters{rIdx}.woTheta_2;
% 
% dux_1 = parameters{rIdx}.dux_1;
% duy_1 = parameters{rIdx}.duy_1;
% duTheta_1 = parameters{rIdx}.duTheta_1;



SAFE_DIST = safe_dist;
vmax = 0.5;
wmax = pi;
dL = 1;
dPhi = pi/4;

leaderPose =  poses(:,1);
dTheta = leaderPose(3);
if parameters{rIdx}.sideRole == 2 %Leader
    if pose(2) < 5000
        vel = bodyToWorld([0.1;0;0],pose);
        e = 0;
        return
    else
        vel = bodyToWorld([0.1;0;0.1],pose);
        e =0;
        return
    end
else
    % Determine the sideRole
%         pose = poses(:,rIdx);
%         if pose(1) > poses(1,1)
%             parameters{rIdx}.sideRole = 1;
%         else
%             parameters{rIdx}.sideRole = 0;
%         end
end

if ~isempty(detections) % See someone
    % Find the closest one
    [closestRange,closestInds] = min(detections(:,1));
    closestMateRange = 100;
    mateInd = 0;
    
    %         leaderInds = find(detections(:,3) ==1);
    if parameters{rIdx}.sideRole == 1
        dPhi = pi/4;
        for ii = 1:size(detections,1)
            if detections(ii,2) > -0.1
                if parameters{detections(ii,3)}.sideRole == parameters{rIdx}.sideRole || parameters{detections(ii,3)}.sideRole == 2
                    if closestMateRange > detections(ii,1)
                        closestMateRange = detections(ii,1);
                        mateInd = ii;
                    end
                end
            end
        end
    end
    
    if parameters{rIdx}.sideRole == 0
        dPhi = -pi/4;
        for ii = 1:size(detections,1)
            if detections(ii,2) < 0.1
                if parameters{detections(ii,3)}.sideRole == parameters{rIdx}.sideRole || parameters{detections(ii,3)}.sideRole == 2
                    if closestMateRange > detections(ii,1)
                        closestMateRange = detections(ii,1);
                        mateInd = ii;
                    end
                end
            end
        end
    end
    
    if mateInd ~= 0
        L = detections(mateInd,1);
        phi = detections(mateInd,2);
    else
        L = detections(closestInds,1);
        phi = detections(closestInds,2);
    end
    
    if closestRange < SAFE_DIST
        phi = detections(closestInds,2);
        xE = -1/closestRange*cos(phi);
        yE = -1/closestRange*sin(phi);
        thetaE = -phi;
    else
        xE = L*cos(phi) - dL * cos(dPhi);
        yE = L*sin(phi) - dL * sin(dPhi);
        thetaE =  dTheta - pose(3);
    end
   
    targetPose = [xE+pose(1) yE+pose(2) dTheta]';
    param.targetPose = targetPose;
    
    dxek = xE - parameters{rIdx}.xek11;
    dxek2 = dxek - (parameters{rIdx}.xek11 - parameters{rIdx}.xek12);
    dyek = yE - parameters{rIdx}.yek11;
    dyek2 = dyek - (parameters{rIdx}.yek11 - parameters{rIdx}.yek12);
    daek = thetaE - parameters{rIdx}.aek11;
    daek2 = daek - (parameters{rIdx}.aek11 - parameters{rIdx}.aek12);
%     xiX = [targetPose(1), pose(1), xE, 1];
%     xiY = [targetPose(2), pose(2), yE, 1];
%     xiTheta = [targetPose(3), pose(3), thetaE, 1];
    xiX = [targetPose(1), pose(1), xE, 1];
    xiYTheta = [targetPose(2), pose(2), yE, targetPose(3), pose(3), thetaE,1];
       [Kx, Ohx] = GetKByWeights(xiX, 5, 3, wiv_1, wov_1);
%     [Kx, Ohx] = GetKByWeights(xiX, 5, 3, wix_1, wox_1);
%     [Ky, Ohy] = GetKByWeights(xiY, 5, 3, wiy_1, woy_1);
%     [KTheta, OhTheta] = GetKByWeights(xiTheta, 5, 3, wiTheta_1, woTheta_1);
    [KyTheta, OhYTheta] = GetKByWeights(xiYTheta, 10, 6, wiw_1, wow_1);
    H = [dxek xE dxek2 0 0 0 0 0 0; 0 0 0 dyek yE dyek2 daek thetaE daek2];
%     K = [Kx; Ky; KTheta];
  
    K = [Kx; KyTheta];
%     K(isnan(K))=1;
    %         [K,kPopList] = bsoIteration(H,pose,targetPose,parameters,rIdx);
    %
    %         %     figure(2)
    %         %     plot(K)
    %         %     hold on
    %
   U = H*K;
   v = v1+U(1);
   w =  w1+U(2);
    if v > 0
        v = min(v,vmax);
    else
%         v= max(v,-vmax);
        v= 0;
    end
    
    if w > 0
        w = min(w,wmax);
    else
        w = max(w,-wmax);
    end
    
    %Parameters Update
    vel = bodyToWorld([v;0;w],pose);
    pose = pose + vel*sampleTime;
    e = targetPose - pose;
%     if norm(e) < norm(param.eLast)
    EpidX = [dxek xE dxek2];
%     EpidY = [dyek yE dyek2];
%     EpidTheta = [daek thetaE daek2];
    
%      [param.wix_1, param.wix_2, param.wox_1,param. wox_2, param.dux_1] = UpdateBPWeights(xiX, e(1), EpidX, Kx, Ohx, 5, 3, wix_1, wix_2, wox_1, wox_2, EpidX*Kx, dux_1);
%      [param.wiy_1, param.wiy_2, param.woy_1,param. woy_2, param.duy_1] = UpdateBPWeights(xiY, e(2), EpidY, Ky, Ohy, 5, 3, wiy_1, wiy_2, woy_1, woy_2, EpidY*Ky, duy_1);
%      [param.wiTheta_1, param.wiTheta_2, param.woTheta_1,param. woTheta_2, param.duTheta_1] = UpdateBPWeights(xiTheta, e(2), EpidTheta, KTheta, OhTheta, 5, 3, wiTheta_1, wiTheta_2, woTheta_1, woTheta_2, EpidTheta*KTheta, duTheta_1);
%      
    EpidYTheta = [dyek yE dyek2 daek thetaE daek2];
    [param.wiv_1, param.wiv_2, param.wov_1,param. wov_2, param.dux_1] = UpdateBPWeights(xiX, e(1), EpidX, Kx, Ohx, 5, 3, wiv_1, wiv_2, wov_1, wov_2, U(1), dux_1);
    [param.wiw_1, param.wiw_2, param.wow_1,param. wow_2, param.duyTheta_1] = UpdateBPWeights(xiYTheta, [e(2), e(3)], EpidYTheta, KyTheta, OhYTheta, 10, 6, wiw_1, wiw_2, wow_1, wow_2, U(2), duyTheta_1);

    %     
%  0930
% 
%     param.wiv_2 = wiv_1;
%     param.wov_2 = wov_1;
%     param.wiw_2 = wiw_1;
%     param.wow_2 = wow_1;
%     param.dux_1 = U(1);
%     param.duyTheta_1 = U(2);

    %     ukx11 = ukx; uky11 = uky; uka11 = uka;
    param.xek12 =  parameters{rIdx}.xek11;
    param.yek12 = parameters{rIdx}.yek11;
    param.aek12 = parameters{rIdx}.aek11;
    param.xek11 = xE;
    param.yek11 = yE;
    param.aek11 = thetaE;
    param.v1 = v;
    param.w1 = w;
    
    param.kBest = K;
%         param.kPopList = kPopList;
%     end
else
    v = 0;
    w = parameters{rIdx}.sideRole - 0.5;
    e = [1e6, 1e6, 1e6];
end

vel = bodyToWorld([v;0;w],pose);
% pose = pose + vel*sampleTime;
 
end