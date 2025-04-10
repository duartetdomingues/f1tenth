function encodeSDFtrack(data, name)

fid = fopen([name, '.sdf'], 'w');

if fid == -1
    error('Could not open SDF file.');
end

fprintf(fid, '<?xml version=''1.0'' encoding=''UTF-8''?>\n');
fprintf(fid, '<sdf version="1.4">\n');
fprintf(fid, '  <model name="some track">\n');

% outter cones
for i = 1:length(data.X_o)
    fprintf(fid, '    <include>\n');
    fprintf(fid, '      <uri>model://fssim_gazebo/models/cone_blue</uri>\n');
    fprintf(fid, '      <pose>%f %f 0 0 0 0</pose>\n', data.X_o(i), data.Y_o(i));
    fprintf(fid, '      <name>cone_right</name>\n');
    fprintf(fid, '    </include>\n');
end

% inner cones
for i = 1:length(data.X_i)
    fprintf(fid, '    <include>\n');
    fprintf(fid, '      <uri>model://fssim_gazebo/models/cone_yellow</uri>\n');
    fprintf(fid, '      <pose>%f %f 0 0 0 0</pose>\n', data.X_i(i), data.Y_i(i));
    fprintf(fid, '      <name>cone_left</name>\n');
    fprintf(fid, '    </include>\n');
end

% orange cones
orange = [4.7 2.5; 4.7 -2.5; 7.3 2.5; 7.3 -2.5];
for i = 1:4
    fprintf(fid, '    <include>\n');
    fprintf(fid, '      <uri>model://fssim_gazebo/models/cone_orange_big</uri>\n');
    fprintf(fid, '      <pose>%f %f 0 0 0 0</pose>\n', orange(i, 1), orange(i, 2));
    fprintf(fid, '      <name>cone_orange_big</name>\n');
    fprintf(fid, '    </include>\n');
end

% tk device
tk_device = [6 3; 6 -3];
for i = 1:2
    fprintf(fid, '    <include>\n');
    fprintf(fid, '      <uri>model://fssim_gazebo/models/time_keeping</uri>\n');
    fprintf(fid, '      <pose>%f %f 0 0 0 0</pose>\n', tk_device(i, 1), tk_device(i, 2));
    fprintf(fid, '      <name>tk_device_%d</name>\n', i-1);
    fprintf(fid, '    </include>\n');
end

fprintf(fid, '  </model>\n');
fprintf(fid, '</sdf>');

fclose(fid);