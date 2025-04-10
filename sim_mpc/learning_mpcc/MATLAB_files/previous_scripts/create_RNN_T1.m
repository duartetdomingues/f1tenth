path_weights = '/home/david/fst/autonomous-systems/src/control/learning_mpcc/AI/saved_weights/model_6_T_1.h5';
info = h5info(path_weights);

%rnn weights
cell_name = info.Groups(2).Groups.Groups.Name;
rnn_kernel = double(h5read(path_weights,strcat(cell_name,'/kernel:0')));
rnn_recurrent_kernel = double(h5read(path_weights,strcat(cell_name,'/recurrent_kernel:0')));
rnn_bias = double(h5read(path_weights,strcat(cell_name,'/bias:0')));
%dense weights
dense_kernel = double(h5read(path_weights,'/Dense/Dense/kernel:0'));
dense_bias = double(h5read(path_weights,'/Dense/Dense/bias:0'));

x0 = [1;1;1;1;1];
x1 = [2;2;2;2;2];

h0 = [0;0;0;0;0];
h1 = tanh(rnn_kernel*x0 + rnn_recurrent_kernel*h0 + rnn_bias);
h2 = tanh(rnn_kernel*x1 + rnn_recurrent_kernel*h1 + rnn_bias);
% out = sigmoid(dense_kernel*h2 + dense_bias)
out = dense_kernel*h2 + dense_bias

function Y=sigmoid(X)
    Y = 1 ./ (1 + exp(-X));
end