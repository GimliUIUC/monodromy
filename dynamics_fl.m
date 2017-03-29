function Xdot = dynamics_fl(t,X,params)

    z = X(1);
    dz = X(2);
    
    g = params.g;
    M = params.M;
    
    ddz = - g;
    Xdot = [dz;ddz];
    
end