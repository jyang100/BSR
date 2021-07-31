figure(1)
plot(meanITAEConv(1:11), 'LineWidth',2);
xlabel('Initial Ideas', 'fontsize', 14);
ylabel('Minimal Fitness Value', 'fontsize', 14);
box on;

figure(2)
plot(meanITAEConv(12:1511), 'LineWidth',2);
xlabel('Iteration Steps', 'fontsize', 14);
ylabel('Minimal Fitness Value', 'fontsize', 14);
box on;

vecMag = 0:0.5:10;
vecAngle  = -1.57:0.1:1.57;
omega = zeros(size(vecMag,2), size(vecAngle,2));
[~, ind]=min(ideasEval(:,numParams+1));
bestParam = ideasEval(ind,1:numParams);
params = bestParam;
net = NNController(Safe_Dist,detectorRange,params);
for i = 1:size(vecMag,2)
    for j = 1:size(vecAngle,2)
        omega(i,j) = 1.5*sim(net, [vecMag(i);vecAngle(j)]);
    end
end
figure(3)
surf(vecAngle, vecMag, omega);
xlabel('Bearing', 'fontsize', 14);
ylabel('Magnitude', 'fontsize', 14);
zlabel('Turning Speed', 'fontsize', 14);
view(45,45)
