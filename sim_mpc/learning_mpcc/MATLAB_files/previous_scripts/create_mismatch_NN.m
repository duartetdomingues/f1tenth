function model = create_mismatch_NN(modelName,T)
    % Load the saved weights file
    path = strcat('../AI/saved_models/' , modelName , '_T_' , int2str(T) , '.h5');
%     
%     if modelName == "model_4"
%         numFeatures = 5;
%         numHiddenUnits = 32;
%         numResponses = 3;
%         
%         layers = [ ...
%             sequenceInputLayer(numFeatures)
%             lstmLayer(numHiddenUnits,'OutputMode','last')
%             fullyConnectedLayer(numResponses)
%             regressionLayer];
% 
%         
%     end
%     
%     model.load_weights(path);
    model = importKerasNetwork(path);

end