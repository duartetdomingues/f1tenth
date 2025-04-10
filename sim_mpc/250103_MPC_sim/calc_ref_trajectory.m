function [xref, dref, ind] = calc_ref_trajectory(state, ref_path, speed_profile, pind)

T = 6;
N_X = 4;
t_step = 0.2;
dl = 1;
min_speed = 1;

xref = zeros(N_X, T);
dref = zeros(1, T);
ncourse = length(ref_path);

ind = calc_nearest_path_index(state, ref_path, pind);

if pind >= ind
    ind = pind;
end

if ind > length(speed_profile)
    speed = speed_profile(end);
else
    speed = speed_profile(ind);
end

xref(1,1) = ref_path(ind,1);
xref(2,1) = ref_path(ind,2);
xref(3,1) = speed;
xref(4,1) = ref_path(ind,3);
dref(1,1) = 0;

travel = 0;

for i=2:T
    travel = travel + abs(state.v)*t_step;
    dind = round(travel/dl);
    
    if ind + dind < ncourse
        xref(1,i) = ref_path(ind + dind,1);
        xref(2,i) = ref_path(ind + dind,2);
        xref(4,i) = ref_path(ind + dind,3);
        
        %speed correction mechanism
        cte = calc_cross_track_error(state, ref_path, ind+dind);
        kspeed = 1 - 0.7 * abs(cte);
        kspeed = max(kspeed, 0); %to make sure kspeed is positive
        speed = kspeed * speed_profile(ind + dind);
        speed = max(speed, min_speed);
        
        xref(3,i) = speed;
        dref(1,i) = 0;
        
        if abs(cte) > 2 % large deviation from reference path
            speed = min(speed, min_speed); % reduce speed
        end
        
    else
        xref(1,i) = ref_path(ncourse,1);
        xref(2,i) = ref_path(ncourse,2);
        xref(3,i) = speed;
        xref(4,i) = ref_path(ncourse,3);
        dref(1,i) = 0;
    end
        
end

end

function cte = calc_cross_track_error(state, ref_path, ind)
x = state.x;
y = state.y;
theta = state.yaw;
xr = ref_path(ind, 1);
yr = ref_path(ind, 2);

dx = x - xr;
dy = y - yr;
c = cos(theta);
s = sin(theta);

cte = dx*s - dy*c;
end
