function [K, Oh] = GetKByWeights(xi, H, Out, wi, wo)
    Oh=zeros(H,1);    %Output from NN middle layer
    %I=Oh;             %Input to NN middle layer
    I=0.0001*xi*wi';
    for j=1:1:H
        Oh(j)=(exp(I(j))-exp(-I(j)))/(exp(I(j))+exp(-I(j))); %Middle Layer
    end
    K=wo*Oh;             %Output Layer
    for l=1:1:Out
        K(l)=exp(K(l))/(exp(K(l))+exp(-K(l)));        %Getting kp,ki,kd
    end
end