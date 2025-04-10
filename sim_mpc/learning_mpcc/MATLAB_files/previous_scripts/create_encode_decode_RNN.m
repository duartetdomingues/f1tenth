clear;clc
% path_weights = '/home/david/fst/autonomous-systems/src/control/learning_mpcc/AI/saved_weights/model_14_T_1.h5';
path_weights = '/home/david/fst/autonomous-systems/src/control/learning_mpcc/AI/saved_weights/model_17_T_1.h5';

info = h5info(path_weights);

%% Parameters
% fc1
fc1_kernel = double(h5read(path_weights,'/fc1/fc1/kernel:0'));
fc1_bias = double(h5read(path_weights,'/fc1/fc1/bias:0'));

% fc2
fc2_kernel = double(h5read(path_weights,'/fc2/fc2/kernel:0'));
fc2_bias = double(h5read(path_weights,'/fc2/fc2/bias:0'));

% fc1_reversed
fc1_rev_kernel = double(h5read(path_weights,'/fc1_reversed/fc1_reversed/kernel:0'));
fc1_rev_bias = double(h5read(path_weights,'/fc1_reversed/fc1_reversed/bias:0'));

% fc2_reversed
fc2_rev_kernel = double(h5read(path_weights,'/fc2_reversed/fc2_reversed/kernel:0'));
fc2_rev_bias = double(h5read(path_weights,'/fc2_reversed/fc2_reversed/bias:0'));

% output layer
fc_out_kernel = double(h5read(path_weights,'/output/output/kernel:0'));
fc_out_bias = double(h5read(path_weights,'/output/output/bias:0'));

%rnn weights
cell_name = info.Groups(9).Groups.Groups.Name;
rnn_kernel = double(h5read(path_weights,strcat(cell_name,'/kernel:0')));
rnn_recurrent_kernel = double(h5read(path_weights,strcat(cell_name,'/recurrent_kernel:0')));
rnn_bias = double(h5read(path_weights,strcat(cell_name,'/bias:0')));

%% NN
in0 = [1;1;1;1;1];
in1 = [2;2;2;2;2];

% Expand space
x0 = GELU(fc1_kernel*in0 + fc1_bias);
x0 = GELU(fc2_kernel*x0 + fc2_bias);

x1 = GELU(fc1_kernel*in1 + fc1_bias);
x1 = GELU(fc2_kernel*x1 + fc2_bias);

% RNN
h0 = [0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0];
h1 = tanh(rnn_kernel*x0 + rnn_recurrent_kernel*h0 + rnn_bias);
h2 = tanh(rnn_kernel*x1 + rnn_recurrent_kernel*h1 + rnn_bias);

% Compress and output
out = GELU(fc2_rev_kernel*h2 + fc2_rev_bias);
out = GELU(fc1_rev_kernel*out + fc1_rev_bias);

out = fc_out_kernel*out + fc_out_bias

function Y=GELU(X)
%     Y = 0.5*(1+tanh(sqrt(2/pi)*(x+0.044715*x^3)));
    Y = X.*0.5.*(1+erf(X./sqrt(2)));
end