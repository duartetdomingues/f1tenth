function traj2 = compute_raceline_trajectory(history_xy, history)
   

    % Critério: ponto mais próximo do ponto inicial após um certo número de passos
    % Critério: encontrar dois pontos sobrepostos (distância < threshold) e separados por uma distância mínima de índices
    threshold = 0.05; % tolerância de sobreposição [m]
    min_index_gap = 20; % distância mínima entre índices

    min_idx = 1;
    min_dist = inf;
    idx1 = 1;
    idx2 = 1;

    exit_flag = false;
    for lap =1:2

        min_idx=idx2;
        for i = min_idx:length(history_xy)-min_index_gap
            for j = i+min_index_gap:length(history_xy)
                dist = norm(history_xy(i,1:2) - history_xy(j,1:2));
                if dist < threshold && dist < min_dist
                    min_dist = dist;
                    idx1 = i;
                    idx2 = j;
                    exit_flag = true;
                    break;
                end
            end
            if exit_flag
                break;
            end
        end
       fprintf('Loop fechado lap %d entre índices: %d e %d, distância: %.4f on (x,y / x,y) %.2f,%.2f / %.2f,%.2f\n',lap , idx1, idx2, min_dist, history_xy(idx1,1),history_xy(idx1,2),history_xy(idx2,1),history_xy(idx2,2));
       min_dist = inf;
    end

    
    
    traj2.s = history(idx1:idx2,1)-history(idx1,1);
    traj2.x = history_xy(idx1:idx2,1);
    traj2.y = history_xy(idx1:idx2,2);


    
    % Calculate curvature (kappa) from trajectory points
    dx = diff(traj2.x);
    dy = diff(traj2.y);
    ddx = diff(dx);
    ddy = diff(dy);

    % Pad to match length
    dx = [dx; dx(end)];
    dy = [dy; dy(end)];
    ddx = [ddx; ddx(end); ddx(end)];
    ddy = [ddy; ddy(end); ddy(end)];

    traj2.kappa = (dx .* ddy - dy .* ddx) ./ ((dx.^2 + dy.^2).^(3/2));

    traj2.vx = history(idx1:idx2,4); % velocidade longitudinal


end
