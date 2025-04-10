function [x,y,dx,dy] = splineTrack(z,p)
    s = z();

    x = a ;
    y = ;
    dx = ;
    dy = ;
end
% Como a data é regular é descobrir o intervalo entre pontos (unwrapInput), com base no
% intervalo e no s dado     descobre-se o index (getIndex) depois é ver,
% com o index é usar o ponto para a spline depois juntamente com o s
% (GetPoint)