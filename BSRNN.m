% BSR_Main
clc;
clear;
close all;

%% Env. Parameters
numRobots = 20;
IterSteps = 1500;           % IterSteps for optimizaton.
sampleTime = 0.1;         % Sample time for eval [s].
evalTime =20;              % evalTime for each iteration.
Safe_Dist =0.8;
detectorRange = round(numRobots/2);

%% Parameters for optimization
popSize = numRobots;
eListsPerc = 0.2;
eListSize = popSize  *  eListsPerc;
pE = 0.2;
pOne = 0.2;

%% Manual Params
numParams = 33;
% params = ones(1,numParams);


%% Design the NN Automatically
ideas = randCustom(popSize,numParams);
meanConvHullAeras = 1000*ones(popSize,1);
meanITAEConv = 1e6*ones(IterSteps+popSize,1);
ideasEval = [ideas meanConvHullAeras];
for j=1:popSize
    fprintf('Evaluation %d idea in %d generation of %d  \n', j, i, IterSteps);
    params = ideas(j,1:numParams);
    net = NNController(Safe_Dist,detectorRange,params);
    ideasEval(j,numParams+1) = SwarmEvaluation(1, numRobots, Safe_Dist, detectorRange, net, sampleTime, evalTime);
    [meanITAEConv(j,1), ~] = mink(ideasEval(:,numParams+1), 1);
    meanITAEConv(j,2) = ideasEval(j,numParams+1);
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
            params = newGenIdea;
            net = NNController(Safe_Dist,detectorRange,params);
            meanConvHullAera = SwarmEvaluation(1, numRobots, Safe_Dist, detectorRange, net, sampleTime, evalTime);
        else
            twoElistIdx = randsample(size(eList,1), 2);
            randLogical = randi([0, 1], [1, numParams]);
            newGenIdea = eList(twoElistIdx(1),:).* randLogical + eList(twoElistIdx(2),:).*~randLogical;
            params = newGenIdea;
            net = NNController(Safe_Dist,detectorRange,params);
            meanConvHullAera = SwarmEvaluation(1, numRobots, Safe_Dist, detectorRange, net, sampleTime, evalTime);
        end
    else
        if rand() < pOne
            oneNormalIdx = randsample(size(normalList,1), 1);
            oneNormal = normalList(oneNormalIdx,:);
            randLogical = randi([0, 1], [1, numParams]);
            newGenIdea = oneNormal.*randLogical + randCustom(1,numParams).*~randLogical;
            params = newGenIdea;
            net = NNController(Safe_Dist,detectorRange,params);
            meanConvHullAera = SwarmEvaluation(1, numRobots, Safe_Dist, detectorRange, net, sampleTime, evalTime);
        else
            twoNormalIdx = randsample(size(normalList,1), 2);
            randLogical = randi([0, 1], [1, numParams]);
            newGenIdea = normalList(twoNormalIdx(1),:).* randLogical + normalList(twoNormalIdx(2),:).*~randLogical;
            params = newGenIdea;
            net = NNController(Safe_Dist,detectorRange,params);
            meanConvHullAera = SwarmEvaluation(1, numRobots, Safe_Dist, detectorRange, net, sampleTime, evalTime);
        end
    end
    
    ideasEval2 = [ideasEval;[newGenIdea meanConvHullAera]];
    [~, maxInd] = mink(ideasEval2(:,numParams+1), popSize);
    ideasEval = ideasEval2(maxInd,1:numParams+1);
    [meanITAEConv(i+popSize,1), ~] = mink(ideasEval(:,numParams+1), 1);
    meanITAEConv(i+popSize, 2) = meanConvHullAera;
end

[~, ind]=min(ideasEval(:,numParams+1));
bestParam = ideasEval(ind,1:numParams);
%bestParam = ideasEval(1,1:numParams);
params = bestParam;
net = NNController(Safe_Dist,detectorRange,params);
meanConvHullAera = SwarmEvaluation(1, numRobots, Safe_Dist, detectorRange, net, sampleTime, evalTime);

save('BSR_NN_0625.mat');

function randNum = randCustom(size,num)
        randNum = -1 + 2* rand(size,num);
end
