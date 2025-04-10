function [poseError, vError] = computeError(state, xref)


poseError = norm([state.x state.y] - [xref(1,1) xref(2,1)]);
vError = norm(state.v - xref(3,1));