
function readSDF(filename, plot_opt)

fid = fopen([filename, '.sdf'], 'r');
left_cone_counter = 0;
right_cone_counter = 0;
while ~feof(fid)
    line = fgetl(fid);
    if strcmp(line ,'    <include>')
        fgetl(fid);
        line = fgetl(fid); % line with pose
        
        splitted_str = split(line, {'<pose>',' '});
        
        x = str2double(splitted_str{8});
        y = str2double(splitted_str{9});
        
        line = fgetl(fid); % line with cone type
        
        splitted_str = split(line, {'<name>','</name>'});
        
        if strcmp(splitted_str{2}, 'cone_right')
            right_cone_counter = right_cone_counter + 1;
            data.X_i(right_cone_counter) = x;
            data.Y_i(right_cone_counter) = y;
        elseif strcmp(splitted_str{2}, 'cone_left')
            left_cone_counter = left_cone_counter + 1;
            data.X_o(left_cone_counter) = x;
            data.Y_o(left_cone_counter) = y;
        end
    end
end
fclose(fid);

%% Find center line points
if  left_cone_counter < right_cone_counter
    % match right cones with cones in the left
    matching_idxs = NaN(right_cone_counter, 2);
    for i = 1:right_cone_counter
        cone = [data.X_i(i), data.Y_i(i)];
        
        auxMat = cone - [data.X_o.', data.Y_o.'];
        
        distances = auxMat(:,1).^2 + auxMat(:,2).^2;
        
        [~, idx] = min(distances);
        
        matching_idxs(i,:) = [i, idx];
    end
    for i = 1:right_cone_counter
        data.X(i) = (data.X_i(matching_idxs(i, 1)) + data.X_o(matching_idxs(i, 2)))*0.5;
        data.Y(i) = (data.Y_i(matching_idxs(i, 1)) + data.Y_o(matching_idxs(i, 2)))*0.5;
    end
else
    % match left cones with cones in the right
    matching_idxs = NaN(left_cone_counter, 2);
    for i = 1:left_cone_counter
        cone = [data.X_o(i), data.Y_o(i)];
        
        auxMat = cone - [data.X_i.', data.Y_i.'];
        
        distances = auxMat(:,1).^2 + auxMat(:,2).^2;
        
        [~, idx] = min(distances);
        
        matching_idxs(i,:) = [i, idx];
    end
    for i = 1:left_cone_counter
        data.X(i) = (data.X_o(matching_idxs(i, 1)) + data.X_i(matching_idxs(i, 2)))*0.5;
        data.Y(i) = (data.Y_o(matching_idxs(i, 1)) + data.Y_i(matching_idxs(i, 2)))*0.5;
    end
end

% plots (optional)

data.X(10) = [];
data.Y(10) = [];

if  plot_opt > 0
    plot(data.X_i, data.Y_i); hold on;
    plot(data.X_o, data.Y_o); hold on;
    plot(data.X, data.Y); axis equal;
    legend('right', 'left', 'centerline');
    grid minor;
end

data = jsonencode(data); % encode to json

fid = fopen([filename, '.json'], 'w');

fprintf(fid, data);

fclose(fid);