% main: finite difference method for calculating Monodromy matrix

% - integrate the dynamics with initial condition/input and get the final 
%   state 
% - perturb the initial condition by a small amount and integrate again to
% get the Monodromy matrix
% - get the eigenvalues and eigenvectors for Monodromy matrix

function Monodromy_finite_diff

%% Constants and parameters
M = 0.625;
g = 9.81;
Tst = 0.18;
z0 = 0.08;
dz0 = 0;
delta_z0 = 1e-2;
delta_dz0 = 1e-2;
coeff_Fz = 3.7*[0 0 0.8834 18.6884 0.5801 29.8481 0];

params.k = 0;
params.M = M;
params.g = g;
params.z0 = z0;
params.dz0 = dz0;
params.Tst = Tst;
params.coeffFz = coeff_Fz;

params_pos_z = params;
params_pos_dz = params;
params_neg_z = params;
params_neg_dz = params;
params_pos_z.z0 =     z0 + delta_z0;
params_pos_dz.dz0 =    dz0 + delta_dz0;
params_neg_z.z0 =     z0 - delta_z0;
params_neg_dz.dz0 =    dz0 - delta_dz0;


%% stance phase
X0 = [z0;dz0];
X0_pos_z =  [z0 + delta_z0;dz0];                % perturbed i.c.
X0_pos_dz = [z0;dz0 + delta_dz0];               % perturbed i.c.
X0_neg_z =  [z0 - delta_z0;dz0];                % perturbed i.c.
X0_neg_dz = [z0;dz0 - delta_dz0];               % perturbed i.c.

[t,X] =         ode45(@(t,X)dynamics_st(t,X,params),[0,Tst],X0);
[t_pos_z,X_pos_z] = ode45(@(t,X)dynamics_st(t,X,params_pos_z),[0,Tst],X0_pos_z);
[t_pos_dz,X_pos_dz] = ode45(@(t,X)dynamics_st(t,X,params_pos_dz),[0,Tst],X0_pos_dz);
[t_neg_z,X_neg_z] = ode45(@(t,X)dynamics_st(t,X,params_neg_z),[0,Tst],X0_neg_z);
[t_neg_dz,X_neg_dz] = ode45(@(t,X)dynamics_st(t,X,params_neg_dz),[0,Tst],X0_neg_dz);

tList = t;                  XList = X;
tList_pos_z = t_pos_z;      XList_pos_z = X_pos_z;
tList_pos_dz = t_pos_dz;    XList_pos_dz = X_pos_dz;
tList_neg_z = t_neg_z;      XList_neg_z = X_neg_z;
tList_neg_dz = t_neg_dz;    XList_neg_dz = X_neg_dz;

%% flight phase
X0 = X(end,:);
X0_pos_z = X_pos_z(end,:);
X0_pos_dz = X_pos_dz(end,:);
X0_neg_z = X_neg_z(end,:);
X0_neg_dz = X_neg_dz(end,:);

options = odeset('Events',@(t,X)apexEvent(t,X));

[t,X] =         ode45(@(t,X)dynamics_fl(t,X,params),[Tst,100],X0,options);
[t_pos_z,X_pos_z] = ode45(@(t,X)dynamics_fl(t,X,params_pos_z),[Tst,100],X0_pos_z,options);
[t_pos_dz,X_pos_dz] = ode45(@(t,X)dynamics_fl(t,X,params_pos_dz),[Tst,100],X0_pos_dz,options);
[t_neg_z,X_neg_z] = ode45(@(t,X)dynamics_fl(t,X,params_neg_z),[Tst,100],X0_neg_z,options);
[t_neg_dz,X_neg_dz] = ode45(@(t,X)dynamics_fl(t,X,params_neg_dz),[Tst,100],X0_neg_dz,options);

tList = [tList;t];                      XList = [XList;X];
tList_pos_z = [tList_pos_z;t_pos_z];    XList_pos_z = [XList_pos_z;X_pos_z];
tList_pos_dz = [tList_pos_dz;t_pos_dz]; XList_pos_dz = [XList_pos_dz;X_pos_dz];
tList_neg_z = [tList_neg_z;t_neg_z];    XList_neg_z = [XList_neg_z;X_neg_z];
tList_neg_dz = [tList_neg_dz;t_neg_dz]; XList_neg_dz = [XList_neg_dz;X_neg_dz];

list.tList = tList;                     list.XList = XList;
list.tList_pos_z = tList_pos_z;         list.XList_pos_z = XList_pos_z;
list.tList_pos_dz = tList_pos_dz;       list.XList_pos_dz = XList_pos_dz;
list.tList_neg_z = tList_neg_z;         list.XList_neg_z = XList_neg_z;
list.tList_neg_dz = tList_neg_dz;       list.XList_neg_dz = XList_neg_dz;

plotStates(list)

%% Monodromy matrix
Q = zeros(2,2);
Q(1,1) = (XList_pos_z(end,1) - XList_neg_z(end,1))/(2*delta_z0);
Q(1,2) = (XList_pos_z(end,1) - XList_neg_z(end,1))/(2*delta_dz0);
Q(2,1) = (XList_pos_dz(end,2) - XList_neg_dz(end,2))/(2*delta_z0);
Q(2,2) = (XList_pos_dz(end,2) - XList_neg_dz(end,2))/(2*delta_dz0);


Q = Q
[eigVec,eigVal] = eig(Q)



function plotStates(list)

tList = list.tList;                      XList = list.XList;
tList_pos_z = list.tList_pos_z;          XList_pos_z = list.XList_pos_z;
tList_pos_dz = list.tList_pos_dz;        XList_pos_dz = list.XList_pos_dz;
tList_neg_z = list.tList_neg_z;          XList_neg_z = list.XList_neg_z;
tList_neg_dz = list.tList_neg_dz;        XList_neg_dz = list.XList_neg_dz;

%% plots
subplot(1,2,1)
plot(tList,XList(:,1))
hold on
plot(tList_pos_z,XList_pos_z(:,1))
plot(tList_neg_z,XList_neg_z(:,1))

title('z')
subplot(1,2,2)
plot(tList,XList(:,2))
hold on
plot(tList_pos_dz,XList_pos_dz(:,2))
plot(tList_neg_dz,XList_neg_dz(:,2))
title('dz')














