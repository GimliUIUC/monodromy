function alpha_int2 = bz_int2(alpha,x0,dx0,s_max)

if nargin == 3
    s_max = 1;
end

[n,m] = size(alpha);
if n > m
    alpha = alpha';
end

M = length(alpha)+1;
AA = zeros(M+1,M+1);

for ii = 1:M-1
    AA(ii,ii:ii+2) = [1 -2 1];
end

AA = M*(M-1)/(s_max)^2 * AA;

AA(M,1:2) = [-M/s_max M/s_max];
AA(M+1,1) = 1;

alpha_int2 = (AA\[alpha dx0 x0]')';


