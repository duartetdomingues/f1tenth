function [s_mesh, fval] = genMesh(N_nodes, track_length, init_mesh)

Aeq = zeros(2, N_nodes);
Aeq(1,1) = 1;
Aeq(2, end) = 1;

beq = [0; track_length];

A = zeros(N_nodes-1, N_nodes);

for i = 1:N_nodes-1
    A(i, i:(i+1)) = [1 -1];
end

b = -1*ones(N_nodes-1, 1);

lb = zeros(N_nodes, 1);
ub = track_length*ones(N_nodes, 1);

if isempty(init_mesh)
    x0 = linspace(0, track_length, N_nodes);
else
    x0 = init_mesh(:).';
end

options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
options.MaxFunctionEvaluations = 7000;
options.OptimalityTolerance = 10^-6;
options.StepTolerance = 10^-10;
[s_mesh, fval] = fmincon(@(x) costFunc(x, track_length),x0,A,b,Aeq,beq,lb,ub, [], options);
s_mesh = s_mesh.';