function [z0, T_st, delta_x, alpha_x, alpha_z, q, dq] = decompose_x(x)

[N,L1,L2,M,Nq] = getParams();

z0 = x(1);
T_st = x(2);
delta_x = x(3);

alpha_x = x(4:8);
alpha_z = x(9:13);

q = x(Nq:Nq+2*N-1);
dq = x(Nq+2*N:end);



