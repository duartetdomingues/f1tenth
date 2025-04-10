function createVParam

Vb = 76;
fb = 76;
wb = 2*pi*fb;
Pp = 2;
Ib = 160;
Imb = 120/sqrt(2);
% Equivalent Circuit Parameters
Rs = 8.56e-3;
ls = 0.06292e-3;
LM = 1.0122e-3;
Rr = 2*5.10e-3;
lr = 0.06709e-3;
% stator and rotor self inductances for dq model
LS = ls + LM;
LR = lr + LM;
% dq model Matrices
LMatrix = [LS 0 LM 0; 0 LS 0 LM; LM 0 LR 0; 0 LM 0 LR];
RMatrix = [Rs 0 0 0; 0 Rs 0 0; 0 0 Rr 0; 0 0 0 Rr];
RLInv = RMatrix/(LMatrix);

ratedFluxPeak = sqrt(2)*Imb*LM;
maxTorque = 65;
maxID = sqrt(2)*Imb;
maxIQ = sqrt(2*Ib^2 - Imb^2);


save('vehicleParameters.mat', 'Vb', 'fb', 'wb', 'Pp', 'Ib', 'Imb', 'Rs',...
    'ls', 'LM', 'Rr', 'lr', 'LS', 'LR', 'LMatrix', 'RMatrix', 'RLInv', ...
    'ratedFluxPeak', 'maxTorque', 'maxID', 'maxIQ')