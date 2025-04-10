
clear;clc;close

modelName = 'model_22_google_40_T1';
NNparams_ = get_NN_params_torch(modelName,1);

p = [zeros(1,53) NNparams_]';


% in0 = [1,1,1,1,1]';
% in1 = [2,2,2,2,2]';
in0 = [0.5, 0.1, 5, 1, 0.5]';
in1 = [0.55, 0.1, 5.2, 1.1, 0.55]';

ind = 53 + 1; % CHANGE THIS PARAMETER
    % FC1
    NNparams.fc1_kernel = reshape(p(ind:ind+(8*5)-1),[8,5]);
    ind = ind + 8*5;
    NNparams.fc1_bias = reshape(p(ind:ind+8-1),[8,1]);
    ind = ind + 8;
    % RNN
    NNparams.rnn_kernel = reshape(p(ind:ind+(8*8)-1),[8,8]);
    ind = ind + 8*8;
    NNparams.rnn_recurrent_kernel = reshape(p(ind:ind+(8*8)-1),[8,8]);
    ind = ind + 8*8;
    NNparams.rnn_bias = reshape(p(ind:ind+8-1),[8,1]);
    ind = ind + 8;
    NNparams.rnn_recurrent_bias = reshape(p(ind:ind+8-1),[8,1]);
    ind = ind + 8;
    % FC1_REV
    NNparams.fc1_rev_kernel = reshape(p(ind:ind+(5*8)-1),[5,8]);
    ind = ind + 5*8;
    NNparams.fc1_rev_bias = reshape(p(ind:ind+5-1),[5,1]);
    ind = ind + 5;
    % FC_OUT
    NNparams.fc_out_kernel = reshape(p(ind:ind+(3*5)-1),[3,5]);
    ind = ind + 3*5;
    NNparams.fc_out_bias = reshape(p(ind:ind+3-1),[3,1]);

    % Expand space
    x0 = GELU(NNparams.fc1_kernel*in0 + NNparams.fc1_bias);

    x1 = GELU(NNparams.fc1_kernel*in1 + NNparams.fc1_bias);
    
    % RNN
    h0 = [0;0;0;0;0;0;0;0];
    h1 = tanh(NNparams.rnn_kernel*x0 + NNparams.rnn_recurrent_kernel*h0 + NNparams.rnn_bias + NNparams.rnn_recurrent_bias);
    h2 = tanh(NNparams.rnn_kernel*x1 + NNparams.rnn_recurrent_kernel*h1 + NNparams.rnn_bias + NNparams.rnn_recurrent_bias);
    
    % Compress and output
    out = GELU(NNparams.fc1_rev_kernel*h2 + NNparams.fc1_rev_bias);
    
    out = NNparams.fc_out_kernel*out + NNparams.fc_out_bias

function Y=GELU(X)
    Y = X.*0.5.*(1+erf(X./sqrt(2)));
end