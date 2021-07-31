%% BSR_Main
clc;
clear;
close all;

%% Env. Parameters
numRobots = 20;
IterSteps = 1500;           % IterSteps for optimizaton.
sampleTime = 0.1;         % Sample time for eval [s].
evalTime =60;              % evalTime for each iteration.
Safe_Dist =0.8;
detectorRange = round(numRobots/2);

%% Parameters for optimization
popSize = numRobots;
eListsPerc = 0.2;
eListSize = popSize  *  eListsPerc;
pE = 0.2;
pOne = 0.8;
numParams = 28;
%% Manual Rulebase
ruleList = [1 1 5 1 1; 1 2 5 1 1; 1 3 5 1 1; 1 4 5 1 1; 1 5 5 1 1; 1 6 5 1 1; 1 7 5 1 1;
    2 1 7 1 1; 2 2 6 1 1; 2 3 6 1 1; 2 4 5 1 1; 2 5 4 1 1; 2 6 4 1 1; 2 7 3 1 1;
    3 1 8 1 1; 3 2 7 1 1; 3 3 7 1 1; 3 4 5 1 1; 3 5 3 1 1; 3 6 3 1 1; 3 7 2 1 1;
    4 1 9 1 1; 4 2 8 1 1; 4 3 8 1 1; 4 4 5 1 1; 4 5 2 1 1; 4 6 2 1 1; 4 7 1 1 1;];
%     5 1 8 1 1; 5 2 7 1 1; 5 3 7 1 1; 5 4 5 1 1; 5 5 3 1 1; 5 6 3 1 1; 5 7 2 1 1;
%     6 1 9 1 1; 6 2 8 1 1; 6 3 8 1 1; 6 4 5 1 1; 6 5 2 1 1; 6 6 2 1 1; 6 7 1 1 1];
% fis = FuzzyController(Safe_Dist,detectorRange,ruleList);
% SwarmEvaluation(numRobots, Safe_Dist, detectorRange, fis, sampleTime, evalTime);

%% Design the Rulebase Automatically
ideas = randi(9,popSize,28);
meanConvHullAeras = 1000*ones(popSize,1);
ideasEval = [ideas meanConvHullAeras];
meanITAEConv = 1e6*ones(IterSteps+popSize,2);

for j=1:popSize
    fprintf('Evaluation %d idea in %d generation of %d  \n', j, i, IterSteps);
    ruleList(:,3) = ideas(j,1:numParams);
    fis = FuzzyController(Safe_Dist,detectorRange,ruleList);
    ideasEval(j,numParams+1) = SwarmEvaluation(0, numRobots, Safe_Dist, detectorRange, fis, sampleTime, evalTime);
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
            newGenIdea = oneElite.*randLogical + randi(9,1,numParams).*~randLogical;
            ruleList(:,3) = newGenIdea;
            fis = FuzzyController(Safe_Dist,detectorRange,ruleList);
            meanConvHullAera = SwarmEvaluation(0, numRobots, Safe_Dist, detectorRange, fis, sampleTime, evalTime);
        else
            twoElistIdx = randsample(size(eList,1), 2);
            randLogical = randi([0, 1], [1, numParams]);
            newGenIdea = eList(twoElistIdx(1),:).* randLogical + eList(twoElistIdx(2),:).*~randLogical;
            ruleList(:,3) = newGenIdea;
            fis = FuzzyController(Safe_Dist,detectorRange,ruleList);
            meanConvHullAera = SwarmEvaluation(0, numRobots, Safe_Dist, detectorRange, fis, sampleTime, evalTime);
        end
    else
        if rand() < pOne
            oneNormalIdx = randsample(size(normalList,1), 1);
            oneNormal = normalList(oneNormalIdx,:);
            randLogical = randi([0, 1], [1, numParams]);
            newGenIdea = oneNormal.*randLogical + randi(9,1,numParams).*~randLogical;
            ruleList(:,3) = newGenIdea;
            fis = FuzzyController(Safe_Dist,detectorRange,ruleList);
            meanConvHullAera = SwarmEvaluation(0, numRobots, Safe_Dist, detectorRange, fis, sampleTime, evalTime);
        else
            twoNormalIdx = randsample(size(normalList,1), 2);
            randLogical = randi([0, 1], [1, numParams]);
            newGenIdea = normalList(twoNormalIdx(1),:).* randLogical + normalList(twoNormalIdx(2),:).*~randLogical;
            ruleList(:,3) = newGenIdea;
            fis = FuzzyController(Safe_Dist,detectorRange,ruleList);
            meanConvHullAera = SwarmEvaluation(0, numRobots, Safe_Dist, detectorRange, fis, sampleTime, evalTime);
        end
    end
    
    ideasEval2 = [ideasEval;[newGenIdea meanConvHullAera]];
    [~, maxInd] = mink(ideasEval2(:,numParams+1), popSize);
    ideasEval = ideasEval2(maxInd,1:numParams+1);
     [meanITAEConv(i+popSize,1), ~] = mink(ideasEval(:,numParams+1), 1);
    meanITAEConv(i+popSize, 2) = meanConvHullAera;
end

[~, ind]=min(ideasEval(:,29));
bestRule = ideasEval(ind,1:28);
ruleList(:,3) = bestRule;
fis = FuzzyController(Safe_Dist,detectorRange,ruleList);
fis.Rules;
%save('BSR_FZ_1006.mat');
figure(2); 
gensurf(fis);
view(135,45)
