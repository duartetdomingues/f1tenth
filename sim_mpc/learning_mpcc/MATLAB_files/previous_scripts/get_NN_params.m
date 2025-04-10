function params = get_NN_params(modelName,T)

    path_weights = strcat('/home/david/fst/autonomous-systems/src/control/learning_mpcc/AI/saved_weights/', modelName,'_T_', int2str(T),'.h5');
    info = h5info(path_weights);

    if strcmp(modelName, 'model_6')
%         %rnn weights
%         params.rnn_kernel = double(h5read(path_weights,'/SimpleRNN/SimpleRNN/simple_rnn_cell_10/kernel:0'));
%         params.rnn_recurrent_kernel = double(h5read(path_weights,'/SimpleRNN/SimpleRNN/simple_rnn_cell_10/recurrent_kernel:0'));
%         params.rnn_bias = double(h5read(path_weights,'/SimpleRNN/SimpleRNN/simple_rnn_cell_10/bias:0'));
        %dense weights
        params.dense_kernel = double(h5read(path_weights,'/Dense/Dense/kernel:0'));
        params.dense_bias = double(h5read(path_weights,'/Dense/Dense/bias:0'));

        %rnn weights
        cell_name = info.Groups(2).Groups.Groups.Name;
        params.rnn_kernel = double(h5read(path_weights,strcat(cell_name,'/kernel:0')));
        params.rnn_recurrent_kernel = double(h5read(path_weights,strcat(cell_name,'/recurrent_kernel:0')));
        params.rnn_bias = double(h5read(path_weights,strcat(cell_name,'/bias:0')));
    elseif strcmp(modelName, 'model_14')
        % fc1
        params.fc1_kernel = double(h5read(path_weights,'/fc1/fc1/kernel:0'));
        params.fc1_bias = double(h5read(path_weights,'/fc1/fc1/bias:0'));
        
        % fc2
        params.fc2_kernel = double(h5read(path_weights,'/fc2/fc2/kernel:0'));
        params.fc2_bias = double(h5read(path_weights,'/fc2/fc2/bias:0'));
        
        % fc1_reversed
        params.fc1_rev_kernel = double(h5read(path_weights,'/fc1_reversed/fc1_reversed/kernel:0'));
        params.fc1_rev_bias = double(h5read(path_weights,'/fc1_reversed/fc1_reversed/bias:0'));
        
        % fc2_reversed
        params.fc2_rev_kernel = double(h5read(path_weights,'/fc2_reversed/fc2_reversed/kernel:0'));
        params.fc2_rev_bias = double(h5read(path_weights,'/fc2_reversed/fc2_reversed/bias:0'));
        
        % output layer
        params.fc_out_kernel = double(h5read(path_weights,'/output/output/kernel:0'));
        params.fc_out_bias = double(h5read(path_weights,'/output/output/bias:0'));
        
        %rnn weights
        cell_name = info.Groups(9).Groups.Groups.Name;
        params.rnn_kernel = double(h5read(path_weights,strcat(cell_name,'/kernel:0')));
        params.rnn_recurrent_kernel = double(h5read(path_weights,strcat(cell_name,'/recurrent_kernel:0')));
        params.rnn_bias = double(h5read(path_weights,strcat(cell_name,'/bias:0')));
    elseif strcmp(modelName, 'model_16')
        % fc1
        params.fc1_kernel = double(h5read(path_weights,'/fc1/fc1/kernel:0'));
        params.fc1_bias = double(h5read(path_weights,'/fc1/fc1/bias:0'));
        
        % fc2
        params.fc2_kernel = double(h5read(path_weights,'/fc2/fc2/kernel:0'));
        params.fc2_bias = double(h5read(path_weights,'/fc2/fc2/bias:0'));
        
        % fc1_reversed
        params.fc1_rev_kernel = double(h5read(path_weights,'/fc1_reversed/fc1_reversed/kernel:0'));
        params.fc1_rev_bias = double(h5read(path_weights,'/fc1_reversed/fc1_reversed/bias:0'));
        
        % fc2_reversed
        params.fc2_rev_kernel = double(h5read(path_weights,'/fc2_reversed/fc2_reversed/kernel:0'));
        params.fc2_rev_bias = double(h5read(path_weights,'/fc2_reversed/fc2_reversed/bias:0'));
        
        % output layer
        params.fc_out_kernel = double(h5read(path_weights,'/output/output/kernel:0'));
        params.fc_out_bias = double(h5read(path_weights,'/output/output/bias:0'));
        
        %rnn weights
        cell_name = info.Groups(9).Groups.Groups.Name;
        params.rnn_kernel = double(h5read(path_weights,strcat(cell_name,'/kernel:0')));
        params.rnn_recurrent_kernel = double(h5read(path_weights,strcat(cell_name,'/recurrent_kernel:0')));
        params.rnn_bias = double(h5read(path_weights,strcat(cell_name,'/bias:0')));
    elseif strcmp(modelName, 'model_17')
        % fc1
        params.fc1_kernel = double(h5read(path_weights,'/fc1/fc1/kernel:0'));
        params.fc1_bias = double(h5read(path_weights,'/fc1/fc1/bias:0'));
        
        % fc2
        params.fc2_kernel = double(h5read(path_weights,'/fc2/fc2/kernel:0'));
        params.fc2_bias = double(h5read(path_weights,'/fc2/fc2/bias:0'));
        
        % fc1_reversed
        params.fc1_rev_kernel = double(h5read(path_weights,'/fc1_reversed/fc1_reversed/kernel:0'));
        params.fc1_rev_bias = double(h5read(path_weights,'/fc1_reversed/fc1_reversed/bias:0'));
        
        % fc2_reversed
        params.fc2_rev_kernel = double(h5read(path_weights,'/fc2_reversed/fc2_reversed/kernel:0'));
        params.fc2_rev_bias = double(h5read(path_weights,'/fc2_reversed/fc2_reversed/bias:0'));
        
        % output layer
        params.fc_out_kernel = double(h5read(path_weights,'/output/output/kernel:0'));
        params.fc_out_bias = double(h5read(path_weights,'/output/output/bias:0'));
        
        %rnn weights
        params.rnn_kernel = double(h5read(path_weights,strcat('/rnn_layer/rnn_layer/rnn_cell/kernel:0')));
        params.rnn_recurrent_kernel = double(h5read(path_weights,strcat('/rnn_layer/rnn_layer/rnn_cell/recurrent_kernel:0')));
        params.rnn_bias = double(h5read(path_weights,strcat('/rnn_layer/rnn_layer/rnn_cell/bias:0')));
    else
        error('No model detected')
    end