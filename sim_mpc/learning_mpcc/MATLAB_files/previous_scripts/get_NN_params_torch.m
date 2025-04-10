function NNparams = get_NN_params_torch(modelName, layers)

    path_weights = strcat('../AI/saved_weights/', modelName,'.mat');

    weights = load(path_weights);
    weights = weights.weights;
    
    %% Parameters

    if layers == 2
        % fc1
        params.fc1_kernel = double(getfield(weights,'fc1.weight'));
        params.fc1_bias = double(getfield(weights,'fc1.bias'))';
        
        % fc2
        params.fc2_kernel = double(getfield(weights,'fc2.weight'));
        params.fc2_bias = double(getfield(weights,'fc2.bias'))';
        
        % fc1_reversed
        params.fc1_rev_kernel = double(getfield(weights,'fc1_reversed.weight'));
        params.fc1_rev_bias = double(getfield(weights,'fc1_reversed.bias'))';
        
        % fc2_reversed
        params.fc2_rev_kernel = double(getfield(weights,'fc2_reversed.weight'));
        params.fc2_rev_bias = double(getfield(weights,'fc2_reversed.bias'))';
        
        % output layer
        params.fc_out_kernel = double(getfield(weights,'output.weight'));
        params.fc_out_bias = double(getfield(weights,'output.bias'))';
        
        %rnn weights
        params.rnn_kernel = double(getfield(weights,'rnn_layer.weight_ih_l0'));
        params.rnn_recurrent_kernel = double(getfield(weights,'rnn_layer.weight_hh_l0'));
        params.rnn_bias = double(getfield(weights,'rnn_layer.bias_ih_l0'))';
        params.rnn_recurrent_bias = double(getfield(weights,'rnn_layer.bias_hh_l0'))';
    
        NNparams = [params.fc1_kernel(:)' params.fc1_bias(:)' params.fc2_kernel(:)' params.fc2_bias(:)' params.rnn_kernel(:)' params.rnn_recurrent_kernel(:)' params.rnn_bias(:)' params.rnn_recurrent_bias(:)' params.fc2_rev_kernel(:)' params.fc2_rev_bias(:)' params.fc1_rev_kernel(:)' params.fc1_rev_bias(:)' params.fc_out_kernel(:)' params.fc_out_bias(:)'];
    elseif layers == 1
        % fc1
        params.fc1_kernel = double(getfield(weights,'fc1.weight'));
        params.fc1_bias = double(getfield(weights,'fc1.bias'))';
        
        % fc1_reversed
        params.fc1_rev_kernel = double(getfield(weights,'fc1_reversed.weight'));
        params.fc1_rev_bias = double(getfield(weights,'fc1_reversed.bias'))';
                
        % output layer
        params.fc_out_kernel = double(getfield(weights,'output.weight'));
        params.fc_out_bias = double(getfield(weights,'output.bias'))';
        
        %rnn weights
        params.rnn_kernel = double(getfield(weights,'rnn_layer.weight_ih_l0'));
        params.rnn_recurrent_kernel = double(getfield(weights,'rnn_layer.weight_hh_l0'));
        params.rnn_bias = double(getfield(weights,'rnn_layer.bias_ih_l0'))';
        params.rnn_recurrent_bias = double(getfield(weights,'rnn_layer.bias_hh_l0'))';
    
        NNparams = [params.fc1_kernel(:)' params.fc1_bias(:)' params.rnn_kernel(:)' params.rnn_recurrent_kernel(:)' params.rnn_bias(:)' params.rnn_recurrent_bias(:)' params.fc1_rev_kernel(:)' params.fc1_rev_bias(:)' params.fc_out_kernel(:)' params.fc_out_bias(:)'];
    elseif layers == 0
        % output layer
        params.fc_out_kernel = double(getfield(weights,'output.weight'));
        params.fc_out_bias = double(getfield(weights,'output.bias'))';
        
        %rnn weights
        params.rnn_kernel = double(getfield(weights,'rnn_layer.weight_ih_l0'));
        params.rnn_recurrent_kernel = double(getfield(weights,'rnn_layer.weight_hh_l0'));
        params.rnn_bias = double(getfield(weights,'rnn_layer.bias_ih_l0'))';
        params.rnn_recurrent_bias = double(getfield(weights,'rnn_layer.bias_hh_l0'))';
    
        NNparams = [params.rnn_kernel(:)' params.rnn_recurrent_kernel(:)' params.rnn_bias(:)' params.rnn_recurrent_bias(:)' params.fc_out_kernel(:)' params.fc_out_bias(:)'];


    end
