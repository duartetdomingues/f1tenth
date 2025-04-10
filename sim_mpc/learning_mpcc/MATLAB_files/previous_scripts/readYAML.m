function data = readYAML(file_name)

    yaml_data = yaml.loadFile(file_name);

    number_cones_y = length(yaml_data.map.yellow_cones);
    number_cones_b = length(yaml_data.map.blue_cones);
    number_centerline = length(yaml_data.map.centerline);

    data = struct('X_b',zeros(number_cones_b,1),'Y_b',zeros(number_cones_b,1),'X_y',zeros(number_cones_y,1),'Y_y',zeros(number_cones_y,1),'X',zeros(number_centerline,1),'Y',zeros(number_centerline,1));

    for i=1:number_cones_b
        data.X_b(i) = yaml_data.map.blue_cones{1,i}{1,1};
        data.Y_b(i) = yaml_data.map.blue_cones{1,i}{1,2};
    end
    for i=1:number_cones_y
        data.X_y(i) = yaml_data.map.yellow_cones{1,i}{1,1};
        data.Y_y(i) = yaml_data.map.yellow_cones{1,i}{1,2};
    end
    for i=1:number_centerline
        data.X(i) = yaml_data.map.centerline{1,i}{1,1};
        data.Y(i) = yaml_data.map.centerline{1,i}{1,2};
    end
        
end