% Dados experimentais de tempo e posição
t = [0, 0.1, 0.2, 0.3, 0.4, 0.5];  % Tempo
delta = [0, 10, 25, 40, 50, 55];  % Posição angular medida

% Definir o tipo de modelo exponencial
ft = fittype('a*(1 - exp(-x/b))', 'independent', 'x', 'coefficients', {'a', 'b'});

% Ajustar a curva aos dados
[curve, gof] = fit(t', delta', ft);

% Exibir os parâmetros ajustados
disp(['Constante de tempo (tau): ', num2str(curve.b)]);

% Plotar a curva ajustada
figure;
plot(curve, t, delta);
xlabel('Tempo (s)');
ylabel('Posição Angular (°)');
title('Ajuste Exponencial para a Resposta do Servo');
