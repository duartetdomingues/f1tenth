
clear;clc;close
T = 1;
modelID = 20; % 6 and 14 are the one working

modelName = strcat('model_',int2str(modelID));
NNstruct = get_NN_params_torch(modelName,T);
% NNstruct = get_NN_params(modelName,T);
NNparams_ = [NNstruct.fc1_kernel(:)' NNstruct.fc1_bias(:)' NNstruct.fc2_kernel(:)' NNstruct.fc2_bias(:)' NNstruct.rnn_kernel(:)' NNstruct.rnn_recurrent_kernel(:)' NNstruct.rnn_bias(:)' NNstruct.rnn_recurrent_bias(:)' NNstruct.fc2_rev_kernel(:)' NNstruct.fc2_rev_bias(:)' NNstruct.fc1_rev_kernel(:)' NNstruct.fc1_rev_bias(:)' NNstruct.fc_out_kernel(:)' NNstruct.fc_out_bias(:)'];

p = [zeros(1,50) NNparams_]';


% in0 = [1,1,1,1,1]';
% in1 = [2,2,2,2,2]';
in0 = [0.5, 0.1, 5, 1, 0.5]';
in1 = [0.55, 0.1, 5.2, 1.1, 0.55]';

ind = 54 + 1;
% FC1
NNparams.fc1_kernel = reshape(p(ind:ind+(32*5)-1),[32,5]);
ind = ind + 32*5;
NNparams.fc1_bias = reshape(p(ind:ind+32-1),[32,1]);
ind = ind + 32;
% FC2
NNparams.fc2_kernel = reshape(p(ind:ind+(16*32)-1),[16,32]);
ind = ind + 16*32;
NNparams.fc2_bias = reshape(p(ind:ind+16-1),[16,1]);
ind = ind + 16;
% RNN
NNparams.rnn_kernel = reshape(p(ind:ind+(16*16)-1),[16,16]);
ind = ind + 16*16;
NNparams.rnn_recurrent_kernel = reshape(p(ind:ind+(16*16)-1),[16,16]);
ind = ind + 16*16;
NNparams.rnn_bias = reshape(p(ind:ind+16-1),[16,1]);
ind = ind + 16;
NNparams.rnn_recurrent_bias = reshape(p(ind:ind+16-1),[16,1]);
ind = ind + 16;
% FC2_REV
NNparams.fc2_rev_kernel = reshape(p(ind:ind+(32*16)-1),[32,16]);
ind = ind + 32*16;
NNparams.fc2_rev_bias = reshape(p(ind:ind+32-1),[32,1]);
ind = ind + 32;
% FC1_REV
NNparams.fc1_rev_kernel = reshape(p(ind:ind+(5*32)-1),[5,32]);
ind = ind + 5*32;
NNparams.fc1_rev_bias = reshape(p(ind:ind+5-1),[5,1]);
ind = ind + 5;
% FC_OUT
NNparams.fc_out_kernel = reshape(p(ind:ind+(3*5)-1),[3,5]);
ind = ind + 3*5;
NNparams.fc_out_bias = reshape(p(ind:ind+3-1),[3,1]);

% Expand space
x0 = GELU(NNparams.fc1_kernel*in0 + NNparams.fc1_bias);
x0 = GELU(NNparams.fc2_kernel*x0 + NNparams.fc2_bias);

x1 = GELU(NNparams.fc1_kernel*in1 + NNparams.fc1_bias);
x1 = GELU(NNparams.fc2_kernel*x1 + NNparams.fc2_bias);

% RNN
h0 = [0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0];
h1 = tanh(NNparams.rnn_kernel*x0 + NNparams.rnn_recurrent_kernel*h0 + NNparams.rnn_bias + NNparams.rnn_recurrent_bias);
h2 = tanh(NNparams.rnn_kernel*x1 + NNparams.rnn_recurrent_kernel*h1 + NNparams.rnn_bias + NNparams.rnn_recurrent_bias);

% Compress and output
out = GELU(NNparams.fc2_rev_kernel*h2 + NNparams.fc2_rev_bias);
out = GELU(NNparams.fc1_rev_kernel*out + NNparams.fc1_rev_bias);

out = NNparams.fc_out_kernel*out + NNparams.fc_out_bias

function Y=GELU(X)
    Y = X.*0.5.*(1+erf(X./sqrt(2)));
end