Gamma = 20;
pi = 3.14;
r_c = 0.1;
b = 1;
a = pi./4;
beta = 0.7;
delta = 4;

[x,y] = meshgrid(-5:.1:5);
first_item =log(((x-a./2 + b./2).^2 + r_c.^2)./((x-a./2 - b./2).^2 + r_c.^2));
second_item = log(((x + a./2 + b./2).^2 + r_c.^2)./((x + a./2 - b./2).^2 + r_c.^2));
third_item = (1+ y./sqrt((b./2).^2 + y.^2)).*exp(-((y-beta).^2)./(2*delta));
f0 = (Gamma./(4*pi*b)).* (first_item - second_item).*third_item;

mesh(x,y,f0);