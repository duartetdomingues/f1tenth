% clear;clc
path_weights = '/home/david/fst/autonomous-systems/src/control/learning_mpcc/AI/saved_weights/model_18_T_1.mat';

weights = load(path_weights);
weights = weights.weights;

%% Parameters
% fc1
fc1_kernel = double(getfield(weights,'fc1.weight'));
fc1_bias = double(getfield(weights,'fc1.bias'))';

% fc2
fc2_kernel = double(getfield(weights,'fc2.weight'));
fc2_bias = double(getfield(weights,'fc2.bias'))';

% fc1_reversed
fc1_rev_kernel = double(getfield(weights,'fc1_reversed.weight'));
fc1_rev_bias = double(getfield(weights,'fc1_reversed.bias'))';

% fc2_reversed
fc2_rev_kernel = double(getfield(weights,'fc2_reversed.weight'));
fc2_rev_bias = double(getfield(weights,'fc2_reversed.bias'))';

% output layer
fc_out_kernel = double(getfield(weights,'output.weight'));
fc_out_bias = double(getfield(weights,'output.bias'))';

%rnn weights
rnn_kernel = double(getfield(weights,'rnn_layer.weight_ih_l0'));
rnn_recurrent_kernel = double(getfield(weights,'rnn_layer.weight_hh_l0'));
rnn_bias = double(getfield(weights,'rnn_layer.bias_ih_l0'))';
rnn_recurrent_bias = double(getfield(weights,'rnn_layer.bias_hh_l0'))';

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
h1 = tanh(rnn_kernel*x0 + rnn_recurrent_kernel*h0 + rnn_bias + rnn_recurrent_bias);
h2 = tanh(rnn_kernel*x1 + rnn_recurrent_kernel*h1 + rnn_bias + rnn_recurrent_bias);

% Compress and output
out = GELU(fc2_rev_kernel*h2 + fc2_rev_bias);
out = GELU(fc1_rev_kernel*out + fc1_rev_bias);

out = fc_out_kernel*out + fc_out_bias

function Y=GELU(X)
%     Y = 0.5*(1+tanh(sqrt(2/pi)*(x+0.044715*x^3)));
    Y = X.*0.5.*(1+erf(X./sqrt(2)));
end