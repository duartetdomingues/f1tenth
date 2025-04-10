% The way this ordering is done considers that the X grows positively from 
% the first cone to the second one
% and also that the second cone is at least more than 
% 0.2m away from the the first one. This might need to have some changes
% but works for now

function track_data = orderCones(track_data)

    Xi_vec = zeros(size(track_data.X_b));
    Yi_vec = zeros(size(track_data.Y_b));

    min_distance = inf;
    Xi_min = NaN;
    Yi_min = NaN;
    for i = 1: size(track_data.X_b,1)

        distance = sqrt(track_data.X_b(i)^2 + track_data.Y_b(i)^2);
        if distance < min_distance 
            min_distance = distance;
            Xi_min = track_data.X_b(i);
            Yi_min = track_data.Y_b(i);
        end    
    end

    Xi_vec(1) = Xi_min;
    Yi_vec(1) = Yi_min;

    % second point (in the direction of movement)
    min_distance = inf;
    for i = 1: size(track_data.X_b,1)

        distance = sqrt( (track_data.X_b(i) - Xi_min)^2 + (track_data.Y_b(i) - Yi_min)^2);
        if distance < min_distance && distance > 0.2 && track_data.X_b(i) > Xi_min
            min_distance = distance;
            Xi_vec(2) = track_data.X_b(i);
            Yi_vec(2) = track_data.Y_b(i);
        end    
    end

    %all the other points
    for i = 3:size(track_data.X_b,1)
        min_distance = inf;
        for j = 1:size(track_data.X_b,1) 

            distance = sqrt( (track_data.X_b(j) - Xi_vec(i-1))^2 + (track_data.Y_b(j) - Yi_vec(i-1))^2);
            if distance < min_distance && distance > 0.2 &&  isempty(find(track_data.X_b(j) == Xi_vec)) &&  isempty(find(track_data.Y_b(j) == Yi_vec))
                min_distance = distance;
                Xi_vec(i) = track_data.X_b(j);
                Yi_vec(i) = track_data.Y_b(j);
            end 

        end
    end

    if Xi_vec(1) ~= Xi_vec(end) && Yi_vec(1) ~= Yi_vec(end)
        Xi_vec = [Xi_vec; Xi_vec(1)];
        Yi_vec = [Yi_vec; Yi_vec(1)];
    end

    data_new = track_data;
    track_data.X_b = Xi_vec;
    track_data.Y_b = Yi_vec;


    % Order outter cones
    Xo_vec = zeros(size(track_data.X_y));
    Yo_vec = zeros(size(track_data.Y_y));

    min_distance = inf;
    Xo_min = NaN;
    Yo_min = NaN;
    for i = 1: size(track_data.X_y,1)

        distance = sqrt(track_data.X_y(i)^2 + track_data.Y_y(i)^2);
        if distance < min_distance 
            min_distance = distance;
            Xo_min = track_data.X_y(i);
            Yo_min = track_data.Y_y(i);
        end    
    end

    Xo_vec(1) = Xo_min;
    Yo_vec(1) = Yo_min;

    % second point (in the direction of movement)
    min_distance = inf;
    for i = 1: size(track_data.X_y,1)

        distance = sqrt( (track_data.X_y(i) - Xo_min)^2 + (track_data.Y_y(i) - Yo_min)^2);
        if distance < min_distance && distance > 0.2 && track_data.X_y(i) > Xo_min
            min_distance = distance;
            Xo_vec(2) = track_data.X_y(i);
            Yo_vec(2) = track_data.Y_y(i);
        end    
    end

    %all the other points
    for i = 3:size(track_data.X_y,1)
        min_distance = inf;
        for j = 1:size(track_data.X_y,1) 

            distance = sqrt( (track_data.X_y(j) - Xo_vec(i-1))^2 + (track_data.Y_y(j) - Yo_vec(i-1))^2);
            if distance < min_distance && distance > 0.2 &&  isempty(find(track_data.X_y(j) == Xo_vec)) &&  isempty(find(track_data.Y_y(j) == Yo_vec))
                min_distance = distance;
                Xo_vec(i) = track_data.X_y(j);
                Yo_vec(i) = track_data.Y_y(j);
            end 

        end
    end

    if Xo_vec(1) ~= Xo_vec(end) && Yo_vec(1) ~= Yo_vec(end)
        Xo_vec = [Xo_vec; Xo_vec(1)];
        Yo_vec = [Yo_vec; Yo_vec(1)];
    end

    track_data.X_y = Xo_vec;
    track_data.Y_y = Yo_vec;

end