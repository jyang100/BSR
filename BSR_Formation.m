% BSR_Main
clc;
clear;
close all;

%% Env. Parameters
numRobots = 51; 
IterSteps = 1500;           % IterSteps for optimizaton.
sampleTime = 0.1;         % Sample time for eval [s].
evalTime =500;              % evalTime for each iteration.
Safe_Dist =0.5;
% detectorRange = 30;
detectorRange = round(numRobots/2);

%% Parameters for optimization
popSize = numRobots;
eListsPerc = 0.2;
eListSize = round(popSize  *  eListsPerc);
pE = 0.2;
pOne = 0.2; 

%% Manual Params
numParams = 165;
% params = ones(1,numParams);


%% Design the Rulebase Automatically
ideas = randCustom(popSize,numParams);
% meanITAE = 1e6;
meanITAEConv = 1e6*ones(IterSteps+popSize,2);
ideasEval = [ideas 1e6*ones(popSize,1)];
for j=1:popSize
    fprintf('Evaluation %d idea in %d generation of %d  \n', j, i, IterSteps);
    netWeights = ideas(j,1:numParams);
%     net = formationNNC(params);
    ideasEval(j,numParams+1) = FormationEval(numRobots, Safe_Dist, detectorRange, netWeights, sampleTime, evalTime);
    meanITAEConv(j,1) =  ideasEval(j,numParams+1) ;
    [meanITAEConv(j,2), ~] = mink(ideasEval(:,numParams+1), 1);
end
for i = 1:IterSteps
    fprintf('No. %d generation of %d.  \n', i, IterSteps);
    [~, maxInd] = mink(ideasEval(:,numParams+1), eListSize);
    [~, minInd] = maxk(ideasEval(:,numParams+1), popSize - eListSize);
    eList = ideasEval(maxInd,1:numParams);
    normalList = ideasEval(minInd,1:numParams);
    if rand() < pE
        if rand() < pOne
            oneEListIdx = randsample(size(eList,1), 1);
            oneElite = eList(oneEListIdx,:);
            randLogical = randi([0, 1], [1, numParams]);
            newGenIdea = oneElite.*randLogical + randCustom(1,numParams).*~randLogical;
            netWeights = newGenIdea;
            meanITAE = FormationEval(numRobots, Safe_Dist, detectorRange, netWeights, sampleTime, evalTime);
        else
            twoElistIdx = randsample(size(eList,1), 2);
            randLogical = randi([0, 1], [1, numParams]);
            newGenIdea = eList(twoElistIdx(1),:).* randLogical + eList(twoElistIdx(2),:).*~randLogical;
            netWeights = newGenIdea;
            meanITAE = FormationEval(numRobots, Safe_Dist, detectorRange, netWeights, sampleTime, evalTime);
        end
    else
        if rand() < pOne
            oneNormalIdx = randsample(size(normalList,1), 1);
            oneNormal = normalList(oneNormalIdx,:);
            randLogical = randi([0, 1], [1, numParams]);
            newGenIdea = oneNormal.*randLogical + randCustom(1,numParams).*~randLogical;
            netWeights = newGenIdea;
            meanITAE = FormationEval(numRobots, Safe_Dist, detectorRange, netWeights, sampleTime, evalTime);
        else
            twoNormalIdx = randsample(size(normalList,1), 2);
            randLogical = randi([0, 1], [1, numParams]);
            newGenIdea = normalList(twoNormalIdx(1),:).* randLogical + normalList(twoNormalIdx(2),:).*~randLogical;
            netWeights = newGenIdea;
            meanITAE = FormationEval(numRobots, Safe_Dist, detectorRange, netWeights, sampleTime, evalTime);
        end
    end
    
    ideasEval2 = [ideasEval;[newGenIdea meanITAE]];
    [~, maxInd] = mink(ideasEval2(:,numParams+1), popSize);
     meanITAEConv(i+popSize,1) =  meanITAE ;
    [meanITAEConv(i+popSize,2), ~] = mink(ideasEval2(:,numParams+1), 1);
    ideasEval = ideasEval2(maxInd,1:numParams+1);
end

[~, ind]=min(ideasEval(:,numParams+1));
bestParam = ideasEval(ind,1:numParams);
%bestParam = ideasEval(1,1:numParams);
netWeights = bestParam;
meanITAE = FormationEval(numRobots, Safe_Dist, detectorRange, netWeights, sampleTime, evalTime);
% net = NNController(Safe_Dist,detectorRange,netWeights);
% meanConvHullAera = SwarmEvaluation(1, numRobots, Safe_Dist, detectorRange, net, sampleTime, 100);

save('BSR_F_1007.mat');

function randNum = randCustom(size,num)
        randNum = -1 + 2* rand(size,num);
end
