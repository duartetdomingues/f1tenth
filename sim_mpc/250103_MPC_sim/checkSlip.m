function checkSlip(ref_path, sp)

g = 9.8; Lwb =2.5;

for i = 1:length(sp)-1
    accSP(i) = sp(i)-sp(i+1);
end

X1(:, 1) = ref_path(:,1);
X1(:, 2) = ref_path(:,2);
[L, R, K] = curvature(X1);
 
for i = 1:length(sp)-1
    slipSP(i) = (1/g)*sqrt( accSP(i)^2 + ((sp(i+1)^4)/(R(i+1)^2)));
end

steerAng = atan(Lwb./R);
steerAng = rad2deg(steerAng);
steerIDX = find(steerAng>6.039);
ISP = find(slipSP>0.7);
sz=50;

figure(10)
plot(ref_path(:,1), ref_path(:,2), 'LineWidth', 1.5)
grid on
hold on
scatter(ref_path(ISP,1), ref_path(ISP,2),'r', 'LineWidth', 4)
legend('Reference Path','Problematic Points')
xlabel('X [m]')
ylabel('Y [m]')

figure(11)
plot(sp, 'LineWidth', 1.5)
grid on
hold on
scatter(ISP, sp(ISP), sz, 'filled', 'red')
legend('Speed Profile','Problematic Points')
xlabel('X [m]')
ylabel('Speed [m/s]')
