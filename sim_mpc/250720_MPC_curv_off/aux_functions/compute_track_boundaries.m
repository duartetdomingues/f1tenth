function [left_x, left_y, right_x, right_y] = compute_track_boundaries(track)
    % Função que calcula as bordas esquerda e direita de uma pista
    % usando a curvatura e a trajetória.
    %
    % Parâmetros de entrada:
    % track - Estrutura com os dados da trajetória, contendo:
    %         track.x_traj - Coordenadas x da trajetória
    %         track.y_traj - Coordenadas y da trajetória
    %         track.kappa_traj - Curvatura ao longo da trajetória
    % nl_traj - Largura da pista à esquerda (em unidades correspondentes à trajetória)
    % nr_traj - Largura da pista à direita (em unidades correspondentes à trajetória)
    %
    % Parâmetros de saída:
    % left_x, left_y - Coordenadas das bordas esquerda
    % right_x, right_y - Coordenadas das bordas direita

    % Calcular as direções tangenciais à trajetória
    dx = diff(track.x_traj); % Diferença nas coordenadas x
    dy = diff(track.y_traj); % Diferença nas coordenadas y
    d = sqrt(dx.^2 + dy.^2); % Distância entre os pontos consecutivos
    
    % Direções normais à trajetória (perpendicular à tangente)
    theta = atan2(dy, dx); % Ângulo tangencial da trajetória (em radianos)
    norm_x = -sin(theta); % Componente normal em x
    norm_y = cos(theta);  % Componente normal em y
    
    % Normalizar as direções normais
    norm_x = [norm_x; norm_x(end)]; % Adiciona o último valor para manter o tamanho
    norm_y = [norm_y; norm_y(end)]; % Adiciona o último valor para manter o tamanho

    % Larguras à esquerda e à direita, ajustadas pela curvatura
    left_x = track.x_traj + track.nl_traj .* norm_x;
    left_y = track.y_traj + track.nl_traj .* norm_y;
    right_x = track.x_traj - track.nr_traj .* norm_x;
    right_y = track.y_traj - track.nr_traj .* norm_y;
end