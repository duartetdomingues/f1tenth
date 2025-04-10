function [sp, t_sp, E] = sp_24_06_21(ref_path, v_init, v_final, t_aval)

global xmax N SPEED_MAX ACC_MAX tmax x_v

t_start = SPEED_MAX/ACC_MAX;
dx_start = 0.5*ACC_MAX*t_start^2;
dx_uniform = xmax - 2*dx_start;
t_uniform = dx_uniform/SPEED_MAX;
tmax = (2*t_start+t_uniform)*1; % escolher aqui o tempo máximo de percurso

if tmax > t_aval
    tmax = t_aval;
end

x=[ref_path(1,4) dx_start dx_start+dx_uniform xmax];
sp=[v_init SPEED_MAX SPEED_MAX v_final];

v_ind = zeros(1,length(x_v));
for i=1:length(x_v)
    v_ind(i)=interp1(x,sp,x_v(i),'pchip');
end

v0=v_ind;
v0(:) = 0.1;
v0(end) = v_final;
% figure(103)
% plot(v0)

% limits of variables 
lb = ones(1,length(x_v))*0.1;
ub = ones(1,length(x_v))*SPEED_MAX;
% ub(N*0.4:N*0.45) = SPEED_MAX*0.5; % <-------------   curva - velocidade baixa aqui.
v0 = min(v0,ub);

Aeq = zeros(N,N);
beq = zeros(N,1);
Aeq(1,1) = 1;
Aeq(end,end) = 1;
beq(1) = v_init;
beq(end) = v_final;

options = optimoptions('fmincon','MaxFunctionEvaluations',500000,...
    'Display','iter','Algorithm','sqp','MaxIterations', 10000);

%t_start = tic;
[sp, fval]=fmincon(@objfun_v1, v0,[],[],Aeq,beq,lb,ub,@confun_v1,options);
%t_end = toc(t_start);
E=fval/3600; % valor de energia em Wh

t_sp = zeros(1,length(sp));
t1 = 0;
for i=1:length(x_v)-1
    v1=sp(i);

    delta_x = x_v(i+1) - x_v(i);
    delta_v = sp(i+1) - sp(i);
        
    % acc
     a=1/(2*delta_x)*(delta_v^2 + 2*v1*delta_v);
     
    if a==0
        delta_t=delta_x/v1;
        
    else
        a1=0.5*a;
        b1=v1;
        c1=-delta_x;
        delta_t=(-b1+sqrt(b1^2-4*a1*c1))/(2*a1);

    end

    % increase time
    t1=t1+delta_t;
    t_sp(i) = t1;
end

t_sp(end) = t_sp(end-1) + delta_t;


end