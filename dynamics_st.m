function Xdot = dynamics_st(t,X,P)
    %% decompose
    z = X(1);
    dz = X(2);
    
    k = P.k;
    g = P.g;
    M = P.M;
    z0 = P.z0;
    dz0 = P.dz0;
    Tst = P.Tst;
    coeff_Fz = P.coeff_Fz;
    
    %% desired velocity
    coeff_ddz = coeff_Fz/M - g;
    coeff_dz = bz_int(coeff_ddz,dz0,Tst);
    
    s = t/Tst;
    dz_d = polyval_bz(coeff_dz,s);
    
    %% stance dynamics: with feedback on velocity
    Fz = polyval_bz(coeff_Fz,s);
    
    ddz = Fz/M - g + k/M*(dz_d - dz);
    Xdot = [dz;ddz];
    
end