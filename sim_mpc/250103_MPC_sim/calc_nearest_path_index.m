function ref_idx = calc_nearest_path_index(state, ref_path, last_idx)

SEARCH_RANGE = 30;

% The initial point is either the 1st point or the last closest point of
% the vehicle to the path
initial_index = max(1, last_idx);
% The final point is either the last point of the path or 30 points ahead
% of the last closest point of the vehicle to the path.
final_index = min(length(ref_path), last_idx + SEARCH_RANGE);

dx = ref_path(initial_index:final_index,1) - state.x;
dy = ref_path(initial_index:final_index,2) - state.y;

dist_array = dx.*dx + dy.*dy;

[~,index] = min(dist_array);

last_path_idx = index + initial_index - 1;

ref_idx = min(length(ref_path),last_path_idx);

% ref_pose = ref_path(ref_index,1:3);

end