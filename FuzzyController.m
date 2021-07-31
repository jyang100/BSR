% FuzzyController
function fis = FuzzyController(Safe_Dist,detectorRange,ruleList)
SD = Safe_Dist;
RD = detectorRange;
% Fuzzy Controller
fis = mamfis('Name',"SwarmController");
fis = addInput(fis,[-0.4*SD RD],'Name',"Magnitude");
fis = addInput(fis,[-1.5 1.5],'Name',"Bearing");
fis = addOutput(fis,[-3 3],'Name',"Turning Speed");

% MF for dDis
% fis = addMF(fis,"dDis","trapmf",[-2*SD -2*SD -SD -SD+0.2*SD],'Name',"Very Close");
% fis = addMF(fis,"dDis","trapmf",[-SD -SD+0.2*SD -0.4*SD -0.2*SD],'Name',"Close");
fis = addMF(fis,"Magnitude","trapmf",[-0.4*SD -0.2*SD 0.2*SD 0.4*SD],'Name',"Zero");
fis = addMF(fis,"Magnitude","trapmf",[0.2*SD 0.4*SD 0.2*RD 0.4*RD],'Name',"Far");
fis = addMF(fis,"Magnitude","trapmf",[0.2*RD 0.4*RD 0.6*RD 0.8*RD],'Name',"Medium Far");
fis = addMF(fis,"Magnitude","trapmf",[0.6*RD 0.8*RD RD RD],'Name',"Very Far");

%MF for dAngle
fis = addMF(fis,"Bearing","trapmf",[-1.5 -1.5 -1 -0.7],'Name',"Very Left");
fis = addMF(fis,"Bearing","trapmf",[-1 -0.7 -0.5 -0.3],'Name',"Medium Left");
fis = addMF(fis,"Bearing","trapmf",[-0.5 -0.3 -0.2 -0.1],'Name',"Left");
fis = addMF(fis,"Bearing","trapmf",[-0.2 -0.1 0.1 0.2],'Name',"Zero");
fis = addMF(fis,"Bearing","trapmf",[0.1 0.2 0.3 0.5],'Name',"Right");
fis = addMF(fis,"Bearing","trapmf",[0.3 0.5 0.7 1],'Name',"Medium Right");
fis = addMF(fis,"Bearing","trapmf",[0.7 1 1.5 1.5],'Name',"Very Right");

%MF for omega
fis = addMF(fis,"Turning Speed","trapmf",[-3 -2.5 -2.5 -2],'Name',"Very Right");
fis = addMF(fis,"Turning Speed","trapmf",[-2 -1 -1 -0.5],'Name',"Medium Right");
fis = addMF(fis,"Turning Speed","trapmf",[-1 -0.5 -0.5 -0.2],'Name',"Right");
fis = addMF(fis,"Turning Speed","trapmf",[-0.5 -0.2 -0.2 0],'Name',"ZR");
fis = addMF(fis,"Turning Speed","trapmf",[-0.2 0 0 0.2],'Name',"ZO");
fis = addMF(fis,"Turning Speed","trapmf",[0 0.2 0.2 0.5],'Name',"ZL");
fis = addMF(fis,"Turning Speed","trapmf",[0.2 0.5 0.5 1],'Name',"Left");
fis = addMF(fis,"Turning Speed","trapmf",[0.5 1 1 2],'Name',"Medium Left");
fis = addMF(fis,"Turning Speed","trapmf",[2 2.5 2.5 3],'Name',"Very Left");

fis = addRule(fis,ruleList);
end