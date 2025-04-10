function matlabFuncToCpp(x, p, n_output)

n_params = length(p);

fid = fopen('NN_jac_p.m', 'r');
while true
    line = fgetl(fid);
    if ~isempty(line)
        if line(1) == 't'
            break;
        end
    end
end

% p operations
p_names = cellstr(string(p));
[~,idx_p] = sort(cellfun(@length, p_names),'descend');
p_names = p_names(idx_p);

% x operations
x_names = cellstr(string(x));
[~,idx_x] = sort(cellfun(@length, x_names),'descend');
x_names = x_names(idx_x);


k = 1;
while line(1) == 't'
    temp = split(line);
    
    for i = 1:length(p_names)
        line = strrep(line, p_names{i}, ['p(', num2str(idx_p(i)-1), ')'] );
    end
    
    for i = 1:length(x_names)
       line = strrep(line, x_names{i}, ['x(', num2str(idx_x(i)-1), ')'] );
    end
    
    line = strrep(line, '.*', '*');
    line = strrep(line, '2.0', '2');
    
    if any(strfind(line, '.^2'))
        dot_idx = strfind(line, '.^2');
        for char_idx = (dot_idx-1):-1:1
            if isletter(line(char_idx))
                break;
            end
        end
        item = line(char_idx:(dot_idx-1));
        
        line = strrep(line, '.^2', ['*', item]);
    end
    
    line = ['float ', line];
    code{k} = line;
    
    line = fgetl(fid);
    k = k+1;
end

line = strrep(line, '.*', '*');
line = strrep(line, '2.0', '2');

Jac_cell_original = split(line, {'Jac_p = reshape([', ',', ['],[', num2str(n_params),',', num2str(n_output), ']);']});

Jac_cell_original(1) = [];
Jac_cell_original(end) = [];

counter = 0;
counter2 = 0;
for j = 0:(n_output-1)
    for i = 0:(n_params - 1)
        counter = counter + 1;
        
        line = Jac_cell_original{counter};
        
        if strcmp(line, '0.0')
           line = '0.0f';
        end
        
        for k = 1:length(p_names)
            line = strrep(line, p_names{k}, ['p(', num2str(idx_p(k)-1), ')'] );
        end
        
        for k = 1:length(x_names)
            line = strrep(line, x_names{k}, ['x(', num2str(idx_x(k)-1), ')'] );
        end
        
        line = strrep(line, '.*', '*');
        line = strrep(line, '2.0', '2');
        
        if any(strfind(line, '.^2'))
            dot_idx = strfind(line, '.^2');
            for char_idx = (dot_idx-1):-1:1
                if isletter(line(char_idx))
                    break;
                end
            end
            item = line(char_idx:(dot_idx-1));
            
            line = strrep(line, '.^2', ['*', item]);
        end
        
        counter2 = counter2 + 1;
        Jac_cell{counter2} = ['Jac_p(', num2str(i), ', column_idx) = ', line];
    end
    counter2 = counter2 + 1;
    Jac_cell{counter2} = 'column_idx++';
end
Jac_cell = Jac_cell(1:(counter2-1));

fclose(fid);


% write cpp file
fid = fopen('Jac_p.cpp','w');

includes = '#include <cmath>\n#include <eigen3/Eigen/Eigen>\n\n';
fprintf(fid, includes);

header = 'void NN_jacobian(Eigen::MatrixXf &Jac_p, unsigned int &column_idx, const Eigen::VectorXf &x, const Eigen::VectorXf &p)\n{\n';
fprintf(fid, header);

comment = '\t// auxiliar variables\n\n';
fprintf(fid, comment);

for i = 1:length(code)
    fprintf(fid, ['\t', code{i}, '\n']);
end
fprintf(fid, '\n');

comment = '\t// Jacobian assembly\n\n';
fprintf(fid, comment);

for i = 1:length(Jac_cell)
    fprintf(fid, ['\t', Jac_cell{i}, ';\n']);
end

fprintf(fid, '}');

fclose(fid);