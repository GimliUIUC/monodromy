%% function for visualization
function visualization(x)
[N,s,M,Nq] = getParams();
g = 9.81;

%% --- decompose optimization variables ---
z0 = x(1);
alpha = x(2:6);       % 5th order bezier polynomial
T_stance = x(7);

param.alpha = alpha;
param.T_stance = T_stance;
q1 = x(Nq:Nq+N-1);
q2 = x(Nq+N:Nq+2*N-1);
dq1 = x(Nq+2*N:Nq+3*N-1);
dq2 = x(Nq+3*N:Nq+4*N-1);

%% ---  simulate dynamics ---
ic = [z0;0];        % z, dz
t = linspace(0,T_stance,N);
[t, X] = ode45(@(t,X)my_dynamics(t,X,param),t,ic);

v_tf = X(end,2);
% parameters = [s,M];
% J = fcn_J([q1(end),q2(end)],parameters);
% test = [0;X(end,2)] + J*[dq1(end);dq2(end)]
t_ = v_tf/g;
h_max = 0.5*g*t_^2;
fprintf('\n h_max = %.3f\n',h_max)
fprintf('z0 = %.3f\n',z0)
fprintf('T_stance = %.3f\n',T_stance)
impulse = mean([0 alpha])*T_stance;
fprintf('impulse = %.3f\n',impulse);
fprintf('coeff = [0 %.3f %.3f %.3f %.3f %.3f]\n',alpha(1),alpha(2),...
    alpha(3),alpha(4),alpha(5));

Fz = zeros(0,1);
q = zeros(2,0);
dq = zeros(2,0);
u = zeros(2,0);
ss = t/param.T_stance;
parameters = [s,M];
for ii = 1:length(ss)
    Fz(ii) = polyval_bz([0, alpha],ss(ii));
    q(:,ii) = fcn_inv([0;X(ii,1)]);
    J = fcn_J(q(:,ii),parameters);
    dq(:,ii) = J\[0;-X(ii,2)];
    u(:,ii) = J'*[0;Fz(ii)];
end

%%
subplot(2,2,1)
plot(t,X(:,1))
hold on 
plot(t,X(:,2))
xlabel('Time [s]')
ylabel('z-position [m]')
legend('z [m]','dz [m/s]')

subplot(2,2,2)
plot(t,Fz)
hold on
xlabel('Time [s]')
ylabel('Fz [N]')


subplot(2,2,3)
plot(t,u)
hold on
xlabel('Time [s]')
ylabel('Joint torques')
legend('tau1','tau2')

subplot(2,2,4)
plot(-u(1,:),dq1)
hold on
plot(-u(2,:),dq2)
xlabel('torque [Nm]')
ylabel('dq')
legend('hip','knee','location','northwest')

