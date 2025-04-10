function data = readJSON(file_name)

fid = fopen(file_name);
raw = fread(fid,inf);
str = char(raw');
fclose(fid);
data = jsondecode(str);
