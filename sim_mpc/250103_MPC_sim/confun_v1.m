function [c, ceq]=confun_v1(x)

global ACC_MAX tmax x_v g caminho L

v_v=x;
% v_v(1)=0.1;
% v_v(end)=0.1;
a_v=zeros(1,length(v_v));
delta_t_v = zeros(1,length(v_v));

t1=0;

for i=1:length(x_v)-1
    
    v1 = v_v(i);
    
    delta_x = x_v(i+1) - x_v(i);
    delta_v = v_v(i+1) - v_v(i);
    
    % acc
     a_v(i)=1/(2*delta_x)*(delta_v^2 + 2*v1*delta_v);
     
    if a_v(i)==0
        delta_t=delta_x/v1;
    else
        a1=0.5*a_v(i);
        b1=v1;
        c1=-delta_x;
        delta_t=(-b1+sqrt(b1^2-4*a1*c1))/(2*a1);
    end
    t1=t1+delta_t;
    delta_t_v(i) = delta_t;
end
const=abs(a_v)-ACC_MAX; 
c=const;

caminhoX(:, 1) = caminho(:,1);
caminhoX(:, 2) = caminho(:,2);
[~, R, ~] = curvature(caminhoX);
slip = (1/g)*sqrt((a_v.^2) + ((v_v.^4)./(R'.^2)))-0.7;
slip(isnan(slip)) = 0;
c = [c; slip];


ceq = 0;
end