% MAIN_exe

%% Constants and parameters
k = 0;
M = 0.675;
g = 9.81;
Tst = 0.18;
Tfl = 0.6;             % flight time
z0 = 0.08;
dz0 = 0;
delta_z0 = 1e-6;
delta_dz0 = 1e-6;
coeff_Fz = 3.7*[0 0 0.8834 18.6884 0.5801 29.8481 0];

P.k = k;
P.M = M;
P.g = g;
P.Tst = Tst;
P.Tfl = Tfl;
P.z0 = z0;
P.dz0 = dz0;
P.coeff_Fz = coeff_Fz;

P_pos_z = P; P_pos_z.z0 = z0 + delta_z0;
Xf_pos_z = VecField(P_pos_z);

P_neg_z = P; P_neg_z.z0 = z0 - delta_z0;
Xf_neg_z = VecField(P_neg_z);

P_pos_dz = P; P_pos_dz.dz0 = dz0+delta_dz0;
Xf_pos_dz = VecField(P_pos_dz);

P_neg_dz = P; P_neg_dz.dz0 = dz0-delta_dz0;
Xf_neg_dz = VecField(P_neg_dz);


%% Monodromy Matrix
% Q(1,2): the final *z* value with dz0 perturbed divided by 2*delta_dz0
% Q(2,1): the final *dz* value with z0 perturbed divided by 2*delta_z0

Q = zeros(2,2);
Q(1,1) = (Xf_pos_z(1)-Xf_neg_z(1))/(2*delta_z0);
Q(1,2) = (Xf_pos_dz(1)-Xf_neg_dz(1))/(2*delta_dz0);
Q(2,1) = (Xf_pos_z(2)-Xf_neg_z(2))/(2*delta_z0);
Q(2,2) = (Xf_pos_dz(2)-Xf_neg_dz(2))/(2*delta_dz0);

Q = Q
[eigVec, eigVal] = eig(Q)


