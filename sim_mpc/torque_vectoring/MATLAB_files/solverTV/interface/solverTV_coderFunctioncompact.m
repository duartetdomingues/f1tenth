% solverTV : A fast customized optimization solver.
% 
% Copyright (C) 2013-2023 EMBOTECH AG [info@embotech.com]. All rights reserved.
% 
% 
% This program is distributed in the hope that it will be useful.
% EMBOTECH makes NO WARRANTIES with respect to the use of the software 
% without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
% PARTICULAR PURPOSE. 
% 
% EMBOTECH shall not have any liability for any damage arising from the use
% of the software.
% 
% This Agreement shall exclusively be governed by and interpreted in 
% accordance with the laws of Switzerland, excluding its principles
% of conflict of laws. The Courts of Zurich-City shall have exclusive 
% jurisdiction in case of any dispute.
% 
% [OUTPUTS] = solverTV(INPUTS) solves an optimization problem where:
% Inputs:
% - xinit - matrix of size [4x1]
% - x0 - matrix of size [145x1]
% - all_parameters - matrix of size [630x1]
% - solver_timeout - scalar
% - num_of_threads - scalar
% Outputs:
% - outputs - column vector of length 145
function [outputs] = solverTV(xinit, x0, all_parameters, solver_timeout, num_of_threads)
    
    [output, ~, ~] = solverTVBuildable.forcesCall(xinit, x0, all_parameters, solver_timeout, num_of_threads);
    outputs = coder.nullcopy(zeros(145,1));
    outputs(1:10) = output.zI;
    outputs(11:140) = output.z;
    outputs(141:145) = output.zN;
end
