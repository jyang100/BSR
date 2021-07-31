% https://www.mathworks.com/help/deeplearning/ref/network.html#buv67s9-1
function net = NNController(Safe_Dist, detectorRange,params)
% Define the Network
% net = network(numInputs,numLayers,biasConnect,inputConnect,layerConnect,outputConnect)
net = network(2,3,[0;1;0],[1 1;0 0;0 0],[0 0 0;1 0 0;0 1 0],[0 0 1]);

%Define Inputs
net.inputs{1}.size = 1;
net.inputs{2}.size = 1;
net.inputs{1}.range = [0 detectorRange-Safe_Dist];
net.inputs{2}.range = [-1.5 1.5];

%Define Layers
net.layers{1}.dimensions = 2;
net.layers{2}.dimensions =10;
net.layers{3}.dimensions =1;

%Input Weights
net.IW{1,1} = [params(31);0];
net.IW{1,2} = [0;params(32)];

%Biases
net.b{2,1} = [zeros(9,1);params(33)];
% net.b{2,1} = params(4);

%Layer Weights
net.LW{2,1} = [params(1:10)' params(11:20)'];
net.LW{3,2} = params(21:30);
net.layers{3}.transferFcn = 'tansig';
%net.layers{1}.transferFcn = 'tansig';

end
