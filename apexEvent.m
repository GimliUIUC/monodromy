function [zeroCrossing,isterminal,direction] = apexEvent(t,X)


dz = X(2);
zeroCrossing = dz;
isterminal = 1;
direction = -1;