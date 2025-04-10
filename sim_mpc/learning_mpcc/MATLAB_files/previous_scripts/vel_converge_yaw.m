%%
x = linspace(0,20,1000);

y = 1./(x+0.1./(1+x))
y_true = 1./x;
plot(x,y)
hold on
plot(x,y_true)
%%
mu = 1.0;
g = 9.81;
q_r_max = 7.5;
q_r = 5.0;
% vx = linspace(0,20,1000);
% approx_vx = 1./(vx+1./(1+vx));
% r_max = 0.85.*mu.*g./approx_vx;
r = linspace(0,1.05,1000);
r_max = 1.0;
cost = q_r*exp(q_r_max.*(r.^2-r_max.^2));
hold on
plot(r,cost)

%%
r = linspace(-1.20,1.20,1000);
r_max = 1.0;
q_r_max = 1;
q_r = 5.0;
% penalty = sqrt(r.^2 + (r_max / q_r_max)^2) - r_max / q_r_max;
% penalty = r.^2 + (r_max / q_r_max)^2 - r_max / q_r_max;
% penalty = q_r*r.^2 + ((r_max / q_r_max)^2 - r_max / q_r_max);
% penalty = q_r*r.^2 ;
penalty = q_r* (sqrt(r.^2 + (r_max / q_r_max)^2) - r_max / q_r_max);

hold on
plot(r,penalty)

%%
r = linspace(-1.20,1.20,1000);
r_max = 1.0;
q_r_max = 5.0;
q_r = 5.0;
penalty = q_r*exp(q_r_max*(abs(r)-r_max));
hold on
plot(r,penalty)

%%
r = linspace(-1.20,1.20,1000);
r_max = 1.0;
q_r_max = 5.0;
q_r = 5.0;
penalty = q_r*r.^4;
hold on
plot(r,penalty)
%%
r = linspace(-1.20,1.20,1000);
r_max = 0.5;
q_r_max = 7.5;
q_r = 5.0;
penalty = q_r*exp(q_r_max*(r.^2-r_max^2));
hold on
plot(r,penalty)