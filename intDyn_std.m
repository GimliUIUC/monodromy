function [zz,dz,Fz,xx,dx,Fx,tau,J_st] = intDyn_std(x)
%% function for explaining x
[N,L1,L2,M,Nq] = getParams();
g = 9.81;

%% --- decompose x ---
[z0, T_st, delta_x, alpha_Fx, alpha_Fz, q, dq] = decompose_x(x);
q1 = q(1:N);
q2 = q(N+1:end);
dq1 = dq(1:N);
dq2 = dq(N+1:end);

%% integrate Bezier polynomial
alpha_ddz = [0 alpha_Fz]/M - g;
alpha_dz = bz_int(alpha_ddz,0,T_st);          % bezier coefficients for dz
alpha_z = bz_int2(alpha_ddz,z0,0,T_st);       % bezier coefficients for dz

alpha_ddx = [0 alpha_Fx]/M;
alpha_dx = bz_int(alpha_ddx,0,T_st);          % bezier coefficients for dz
alpha_x = bz_int2(alpha_ddx,0,0,T_st);       % bezier coefficients for dz

%%
zz = zeros(N,1);
dz = zeros(N,1);
Fz = zeros(N,1);
xx = zeros(N,1);
dx = zeros(N,1);
Fx = zeros(N,1);
J_st = zeros(2,2,N);
tau = zeros(2,N);

t = linspace(0,T_st,N);
s = t/T_st;

for ii = 1:N
    zz(ii) = polyval_bz(alpha_z,s(ii));
    dz(ii) = polyval_bz(alpha_dz,s(ii));
    Fz(ii) = polyval_bz([0 alpha_Fz],s(ii));
    
    xx(ii) = polyval_bz(alpha_x,s(ii));
    dx(ii) = polyval_bz(alpha_dx,s(ii));
    Fx(ii) = polyval_bz([0 alpha_Fx],s(ii));

    J_st(:,:,ii) = fcn_J_st([q1(ii),q2(ii)],[L1,L2]);
    tau(:,ii) = J_st(:,:,ii)'*[Fx(ii);Fz(ii)];
end