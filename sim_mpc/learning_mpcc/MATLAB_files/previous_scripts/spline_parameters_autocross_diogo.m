n_points = 5;
n_splines = 10;
params = zeros(1,4);

all_progress_splines = zeros(1,n_splines);
all_params = zeros(n_splines,n_points,4);
all_progress_in_spline = zeros(n_splines,n_points,1);

for i = 1:n_splines
    all_progress_splines(1,i) = (i-1)*1.5;
    for j = 1:n_points
        all_params(i,j,1) = 1*j*i;
        all_params(i,j,2) = 2*i*j;
        all_params(i,j,3) = 3*i*j;
        all_params(i,j,4) = 4*i*j;
        all_progress_in_spline(i,j,1) = (j-1)/(10/3) +(i-1)*(n_points)/(10/3);
    end
end
% we select the best spline
for s = 0:0.1:n_splines*1.475
    flag_spline = zeros(n_splines,1);
    for m = 1:n_splines
        if m == n_splines
            flag_spline(m,1) = s >= all_progress_splines(1,m);
        else
            flag_spline(m,1) = s >= all_progress_splines(1,m) & s < all_progress_splines(1,m+1);
        end
        if flag_spline(m,1) == 1
            break;
        end
    end
    flag_points = zeros(n_points,1);
    for n = 1:n_points
        if n == n_points
            flag_points(n,1) = s >= all_progress_in_spline(m,n,1);
        else
            flag_points(n,1) = s >= all_progress_in_spline(m,n,1) & s < all_progress_in_spline(m,n+1,1);
        end
    end
    X = flag_points(1,1)*(all_params(m,1,1)*s^3 + all_params(m,1,2)*s^2 + all_params(m,1,3)*s + all_params(m,1,4)) + flag_points(2,1)*(all_params(m,2,1)*s^3 + all_params(m,2,2)*s^2 + all_params(m,2,3)*s + all_params(m,2,4)) + flag_points(3,1)*(all_params(m,3,1)*s^3 + all_params(m,3,2)*s^2 + all_params(m,3,3)*s + all_params(m,3,4)) + flag_points(4,1)*(all_params(m,4,1)*s^3 + all_params(m,4,2)*s^2 + all_params(m,4,3)*s + all_params(m,4,4)) + flag_points(5,1)*(all_params(m,5,1)*s^3 + all_params(m,5,2)*s^2 + all_params(m,5,3)*s + all_params(m,5,4));
    X_deriv = flag_points(1,1)*(3*all_params(m,1,1)*s^2 + 2*all_params(m,1,2)*s + all_params(m,1,3)) + flag_points(2,1)*(3*all_params(m,2,1)*s^2 + 2*all_params(m,2,2)*s + all_params(m,2,3)) + flag_points(3,1)*(3*all_params(m,3,1)*s^2 + 2*all_params(m,3,2)*s + all_params(m,3,3)) + flag_points(4,1)*(3*all_params(m,4,1)*s^2 + 2*all_params(m,4,2)*s + all_params(m,4,3)) + flag_points(5,1)*(3*all_params(m,5,1)*s^2 + 2*all_params(m,5,2)*s + all_params(m,5,3));
%     params = all_params(lowest_spline,lowest_index,:);
%     params_deriv = polyder(X);
end


