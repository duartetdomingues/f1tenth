function [sp_interp,t_sp_interp,E] = sp_20_06_21(ref_path, v_init, v_final, t_aval)

global N x_v xmax theta caminho

L_target = 200;
x = ref_path(1:end,4);

ref_path = undersample_ref_path(ref_path, L_target);
caminho = ref_path;
% track
N = size(ref_path,1);
x_v = ref_path(1:end,4);
xmax = x_v(end); %m


% perfil da estrada
theta = ref_path(1:end,6);
%theta(1:length(ref_path)) = 3;

[sp,t_sp,E] = sp_24_06_21(ref_path, v_init, v_final, t_aval);

sp_interp = interp1(x_v,sp,x);
t_sp_interp = interp1(x_v,t_sp,x);

for i = length(sp_interp):-1:1
    if isnan(sp_interp(i))
        sp_interp(i) = 0.1;
    else
        continue;
    end
end


if isnan(sum(sp))
    disp('Error - Nans');
    return;
end

end


function path_out = undersample_ref_path(ref_path, L_target)

L = length(ref_path);

if L <= L_target
    path_out = ref_path;
else
    n = round(L/L_target);
    path_out = downsample(ref_path,n);
end

end
