function [wi_1, wi_2, wo_1, wo_2, du_1] = UpdateBPWeights(xi, E, Epid, K, Oh, H, Out, wi_1, wi_2, wo_1, wo_2, du, du_1)
xite=0.20;
alfa=0.05;
I=xi*wi_1';
dyu = sign(prod(E)./(du-du_1+0.0001));
%Output layer
for j=1:1:Out
    dK(j)=2/(exp(K(j))+exp(-K(j)))^2;
end
for l=1:1:Out
    delta3(l)=sum(E)*dyu*Epid(l)*dK(l);
end

for l=1:1:Out
   for i=1:1:H
       d_wo=xite*delta3(l)*Oh(i)+alfa*(wo_1-wo_2);
   end
end
    wo =wo_1 + d_wo+alfa*(wo_1-wo_2);
  %Hidden layer
for i=1:1:H
    dO(i)=4/(exp(I(i))+exp(-I(i)))^2;
end
    segma=delta3*wo;
for i=1:1:H
   delta2(i)=dO(i)*segma(i);
end

d_wi=xite*delta2'*xi;
wi = wi_1+d_wi+alfa*(wi_1-wi_2); 

%Parameters Update
du_1=du;

wo_2=wo_1;
wo_1=wo;

wi_2=wi_1;
wi_1=wi;

end