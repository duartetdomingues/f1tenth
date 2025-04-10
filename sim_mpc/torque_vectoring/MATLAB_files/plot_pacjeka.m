clear
clc
close all
% Initialization Parameters
% Car
m = 228;
l_f = 0.88;
l_r = 0.66;

%Tyre
D = 1.0279;
B = 10;
C = 2;
E = 0.5;

% Compute normal force on a wheel
Fz_tire = m/2*9.81*l_f/(l_f+l_r);

% Complete Magic Formula
compute_Fy = @(x) D*Fz_tire*sin(C*atan(B*x-E*(B*x-atan(B*x))));
%Simplified Magic Formula
compute_Fy_simplified = @(x) D*Fz_tire*sin(C*atan(B*x));

% Create alpha vector and compute Fy for each alpha
alpha = -1:0.0001:1;
alpha_deg = linspace(-57.2958,57.2958,length(alpha));
Fy = compute_Fy_simplified(alpha);

% Plot Force Curve and get peaks (of simplified version)
subplot(1,2,1)
plot(alpha_deg,Fy)
[min_force, min_alpha] = min(Fy);
title("F_y")
fprintf("Min Foce: %f; Min Slip Angle: %f\n",min_force,alpha(min_alpha))

% Plot derivative (of simplified version)
subplot(1,2,2)
string = sprintf("%f*%f*sin(%f*atan(%f*x))",D,Fz_tire,C,B);
fprime = diff(str2sym(string));
fprime = matlabFunction(fprime);
Fy_prime = fprime(alpha);
plot(alpha_deg,Fy_prime)
title("F_y^'")

% Compare derivative by syms and by wolfram
wolfram_Fy_prime = @(x) (B.*C.*D.*Fz_tire.*cos(C.*atan(B.*x)))./(B^2.*x.^2+1);
Fy_wolfram = wolfram_Fy_prime(alpha);

figure()
subplot(1,2,1)
plot(alpha_deg,Fy_prime)
title("F_y^' syms")

subplot(1,2,2)
plot(alpha,Fy_wolfram)
title("F_y^' Wolfram")

