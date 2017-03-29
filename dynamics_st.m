function Xdot = dynamics_st(t,X,params)
    %% decompose
    z = X(1);
    dz = X(2);
    
    k = params.k;
    g = params.g;
    M = params.M;
    z0 = params.z0;
    dz0 = params.dz0;
    Tst = params.Tst;
    coeff_Fz = params.coeffFz;
    
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