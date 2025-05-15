close all

% 1. Ler e binarizar o mapa
filename_base ='map_2025_05_15-16_48';
pgm_file = [filename_base, '.pgm'];
yaml_file = [filename_base, '.yaml'];
map = imread(pgm_file);
yaml_text = fileread(yaml_file);

walls = map < 100;                % Identificar as paredes

% Depois extrai contornos
[B, ~] = bwboundaries(walls, 'holes');

for k = 1:length(B)
    b = B{k};
    % interpolar para 500 pontos com spline (contorno fechado ou aberto)
    %n_interp = 100;
    n_interp = 80;
    t = linspace(0, 1, length(b));
    t_interp = linspace(0, 1, n_interp);
    x_smooth = interp1(t, b(:,2), t_interp, 'spline');
    y_smooth = interp1(t, b(:,1), t_interp, 'spline');
    smoothContours{k} = [y_smooth(:), x_smooth(:)];

    % calcular perímetro suavizado
    dx = diff(x_smooth);
    dy = diff(y_smooth);
    smoothPerimeters(k) = sum(sqrt(dx.^2 + dy.^2));
end

% 4. Ordenar os contornos por perímetro (decrescente)
[~, idx] = sort(smoothPerimeters, 'descend');

figure;
imshow(map); hold on;
for k = 1:4
    c = smoothContours{idx(k)};
    plot(c(:,2), c(:,1), 'LineWidth', 2);
end
title("Contours map smooth")

figure
imshow(map); hold on;
for k = 2:3
    c = B{idx(k)};
    plot(c(:,2), c(:,1), 'LineWidth', 2);
end
title("Contours track")


% Supondo que:
% c1 = contorno exterior (Nx2)
% c2 = contorno interior (Nx2)
c1 = B{idx(2)};
c2 = B{idx(3)};

% Garantir mesma direção (sentido horário/anti-horário)
% if norm(c1(1,:) - c2(end,:)) < norm(c1(1,:) - c2(1,:))
%     c2 = flipud(c2);
% end

mask = poly2mask([c1(:,2); flipud(c2(:,2))], ...
                 [c1(:,1); flipud(c2(:,1))], ...
                 size(map,1), size(map,2));

skel = bwmorph(mask, 'skel', Inf);  % linha fina central

skel = bwmorph(skel, 'spur', 100);  % corta "espinhos"

[y, x] = find(skel);
pts = [y, x];


% Construir matriz de distâncias
D = squareform(pdist(pts));

% Ignorar a diagonal (autodistância)
D(D == 0) = Inf;

% Escolher ponto inicial arbitrário
idx_ordered = zeros(size(pts,1),1);
idx_ordered(1) = 1;
visited = false(size(pts,1),1);
visited(1) = true;

for i = 2:size(pts,1)
    last = idx_ordered(i-1);
    
    % Encontrar o ponto mais próximo não visitado
    D(last, visited) = Inf;
    [~, next] = min(D(last, :));
    
    idx_ordered(i) = next;
    visited(next) = true;
end

% Aplicar ordenação
centerline = pts(idx_ordered, :);

centerline = flipud(centerline);


% Visualizar
imshow(map); hold on;
plot(c1(:,2), c1(:,1), 'r', 'LineWidth', 1.5);
plot(c2(:,2), c2(:,1), 'b', 'LineWidth', 1.5);
plot(centerline(:,1), centerline(:,2), 'g-', 'MarkerSize', 5);
title('Centerline via skeletonization');

janela_media_movel=10; % ajusta janela

% Replicar bordas
% centerline_ext = [ ...
%     centerline(end-janela_media_movel+1:end, :);  % antes
%     centerline;
%     centerline(1:janela_media_movel, :)           % depois
% ];

N = janela_media_movel;  % metade do tamanho da janela

% 1. Estender a curva nas extremidades
centerline_ext = [ ...
    centerline(end-N+1:end, :);  % antes do início
    centerline;
    centerline(1:N, :)           % depois do fim
];

% 2. Aplicar suavização
x_smooth_ext = smoothdata(centerline_ext(:,1), 'gaussian', 2*N+1);
y_smooth_ext = smoothdata(centerline_ext(:,2), 'gaussian', 2*N+1);

% 3. Cortar de volta ao comprimento original
x_smooth2 = x_smooth_ext(N+1:end-N);
y_smooth2 = y_smooth_ext(N+1:end-N);
centerline_smooth2 = [x_smooth2, y_smooth2];

% Plot
figure
imshow(map); hold on;
plot(c1(:,2), c1(:,1), 'r', 'LineWidth', 1.5);
plot(c2(:,2), c2(:,1), 'g', 'LineWidth', 1.5);
plot(centerline(:,2), centerline(:,1), 'b', 'LineWidth', 2);
plot(centerline_smooth2(:,2), centerline_smooth2(:,1), 'm', 'LineWidth', 2)
legend('Contorno exterior', 'Contorno interior', 'Centerline','Centerline smooth1','Centerline smooth2')


% Extrair resolution
res_expr = regexp(yaml_text, 'resolution:\s*([0-9.]+)', 'tokens');
resolution = str2double(res_expr{1});

% Extrair origin
origin_expr = regexp(yaml_text, 'origin:\s*\[([^\]]+)\]', 'tokens');
origin = str2num(origin_expr{1}{1});

x0 = origin(1);
y0 = origin(2);

% 4. Calcular limites reais
[img_height, img_width] = size(map);
x_real = x0 + (0:img_width-1) * resolution;
y_real = y0 + (img_height-1:-1:0) * resolution;  % invertido (imagem → mundo)

% 5. Mostrar imagem com eixos reais
figure; hold on
imagesc(x_real, y_real, map);
colormap(gray); axis equal;
xlabel('x [m]'); ylabel('y [m]');
title('Mapa SLAM em coordenadas reais');

% Converter coordenadas (linha, coluna) -> (x_real, y_real)
x_real = (centerline_smooth2(:,2)) * resolution + origin(1);  % coluna -> x
y_real = (img_height - centerline_smooth2(:,1)) * resolution + origin(2);  % linha invertida -> y

centerline_real = [x_real, y_real];

centerline_real = flipud(centerline_real); % inverter o sentido da traj

plot(centerline_real(:,1), centerline_real(:,2), 'b-', 'LineWidth', 2);
axis equal;
xlabel('x [m]');
ylabel('y [m]');
title('Centerline em coordenadas reais');

%calcular yaw
dx = diff(centerline_real(:,1));
dy = diff(centerline_real(:,2));

yaw = atan2(dy, dx);  % ângulo da tangente em cada segmento

% 2. Padding para manter o mesmo número de pontos (repete último valor)
yaw_full = [yaw; yaw(end)];

x_mid = (centerline_real(1:end-1,1) + centerline_real(2:end,1)) / 2;
y_mid = (centerline_real(1:end-1,2) + centerline_real(2:end,2)) / 2;

figure;
plot(centerline_real(:,1), centerline_real(:,2), 'b-', 'LineWidth', 1.5); hold on;
step = 20;
quiver(x_mid(1:step:end), y_mid(1:step:end), ...
       cos(yaw(1:step:end)), sin(yaw(1:step:end)), 0.5, 'r');
axis equal;
xlabel('x [m]');
ylabel('y [m]');
title('Centerline com orientação (yaw)');
legend('Centerline', 'Yaw');

%velocidade de ref
vref=ones(length(yaw_full),1)*2;

data = table(centerline_real(:,1), centerline_real(:,2), yaw_full, vref,'VariableNames', {'x', 'y', 'yaw', 'v'});

% 4. Guardar em CSV
csv_filename = ['../traj/pts/centerline2_', filename_base, '.csv'];
writetable(data, csv_filename);


function c_interp = interpolate_by_arclength(c, n_interp)
    % Calcula o comprimento acumulado
    dists = sqrt(sum(diff(c).^2, 2));
    arc_len = [0; cumsum(dists)];
    arc_len = arc_len / arc_len(end);  % normaliza para [0,1]

    % Interpola ao longo do comprimento do arco
    tq = linspace(0, 1, n_interp);
    c_interp = [interp1(arc_len, c(:,1), tq, 'spline')', ...
                interp1(arc_len, c(:,2), tq, 'spline')'];
end




