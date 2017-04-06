function X_f = VecField(P)
% X_f final state
% P has parameters and initial conditions
%   .k
%   .M
%   .g
%   .Tst
%   .Tfl
%   .z0
%   .dz0
%   .coeff_Fz

k = P.k;
M = P.M;
g = P.g;
Tst = P.Tst;
Tfl = P.Tfl;
z0 = P.z0;
dz0 = P.dz0;
coeff_Fz = P.coeff_Fz;


%% stance phase
X0 = [z0;dz0];
[t,X] = ode45(@(t,X)dynamics_st(t,X,P),[0,Tst],X0);


%% final state
X_f = X(end,:);



