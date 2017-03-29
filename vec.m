function vy = vec(y)

[m,n] = size(y);
vy = reshape(y,[m*n,1]);
