% solverMPCC : A fast customized optimization solver.
% 
% Copyright (C) 2013-2023 EMBOTECH AG [info@embotech.com]. All rights reserved.
% 
% 
% This software is intended for simulation and testing purposes only. 
% Use of this software for any commercial purpose is prohibited.
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
% [OUTPUTS] = solverMPCC(INPUTS) solves an optimization problem where:
% Inputs:
% - x0 - matrix of size [859x1]
% - xinit - matrix of size [14x1]
% - all_parameters - matrix of size [21471x1]
% - solver_timeout - scalar
% - num_of_threads - scalar
% Outputs:
% - outputs - column vector of length 859
function [outputs] = solverMPCC(x0, xinit, all_parameters, solver_timeout, num_of_threads)
    
    [output, ~, ~] = solverMPCCBuildable.forcesCall(x0, xinit, all_parameters, solver_timeout, num_of_threads);
    outputs = coder.nullcopy(zeros(859,1));
    outputs(1:833) = output.z;
    outputs(834:848) = output.zN_1;
    outputs(849:859) = output.zN;
end
