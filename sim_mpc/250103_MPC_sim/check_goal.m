function [goal_flag,dist] = check_goal(state, goal, tind, nind)

GOAL_DIST = 1;
STOP_SPEED = 1;

goal_flag = false;

dx = state.x - goal(1);
dy = state.y - goal(2);
dist = sqrt(dx*dx + dy*dy);

if dist <= GOAL_DIST
    z = 1;
end

is_goal = dist <= GOAL_DIST;


if abs(tind - nind) >= 50
    is_goal = false;
end

% goal_flag = is_goal;

is_stop = (abs(state.v)) <= STOP_SPEED;

if is_goal && is_stop
    goal_flag = true;
end

end