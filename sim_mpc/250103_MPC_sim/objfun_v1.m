function f = objfun_v1(x)

global M Cd Crr1 A theta rho g x_v tmax

% generated v
v_v=x;
% v_v(1)=0.1;
% v_v(end)=0.1;
E_cons=zeros(1,length(x_v));
t1=0;

for i=1:length(x_v)-1
    v1=v_v(i);

    delta_x = x_v(i+1) - x_v(i);
    delta_v = v_v(i+1) - v_v(i);
    
    % coeficientes de atrito
    theta_i=(theta(i)+theta(i+1))/2;
    beta=0.5*rho*Cd*A;
    %beta = 0;
    gama=Crr1*M*g*cosd(theta_i)+M*g*sind(theta_i);
    
    % acc
     a=1/(2*delta_x)*(delta_v^2 + 2*v1*delta_v);
     
    if a==0
        delta_t=delta_x/v1;
        
        % energia
        E_cons(i+1)=(beta*v1^3+gama*v1)*delta_t;
        E_cons(i+1)=max(0,E_cons(i+1)); % no regenerative braking
    else
        a1=0.5*a;
        b1=v1;
        c1=-delta_x;
        delta_t=(-b1+sqrt(b1^2-4*a1*c1))/(2*a1);
        
        %energia
        E_cons(i+1)=M*a^2*delta_t^2/2 +  M*a*v1*delta_t + (beta*(a*delta_t+v1)^4/(4*a)-beta*v1^4/(4*a))+gama*a*delta_t^2/2+gama*v1*delta_t;
        E_cons(i+1)=max(0,E_cons(i+1)); % no regenerative braking
    end
%     t=0:delta_t/100:delta_t;
%     acc=ones(1,length(t))*a;
%     v=a*t+v1;
%     x=0.5*a*t.^2+v1*t+x1;

    % increase tiMe
    t1=t1+delta_t;
end

E_total=sum(E_cons) + 1000*abs((tmax - t1));% alpha =1000

f=E_total;

if isnan(f)==1
    erro=1;
end
if f<0
    erro=1;
end

end