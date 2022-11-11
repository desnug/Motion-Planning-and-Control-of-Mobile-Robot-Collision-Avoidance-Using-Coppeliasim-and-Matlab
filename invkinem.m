function [pR,pL] = invkinem(v,w)
    wheelradius = 0.195/2;
    bodywidth = 0.381;
    R2 = wheelradius/2;
    RB = wheelradius/bodywidth;
    vw = [v;w];
    kinemat = [R2 R2;RB -RB];
    pLR = kinemat\vw;
    pR = pLR(1);
    pL = pLR(2);
end      