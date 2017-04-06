function Xdot = dynamics_fl(t,X,params)

    dz = X(2);
    
    g = params.g;
    
    ddz = - g;
    Xdot = [dz;ddz];
    
end