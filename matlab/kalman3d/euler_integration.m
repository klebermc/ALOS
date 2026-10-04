function [ X ] = euler_integration( X , U, dt )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% The states are:
% X = [Vx, Vy, Px, Py, psi]'

X(5) = X(5) + U(3)*dt;
psi=X(5);
X(1:2) = X(1:2,1) + ([cos(psi) -sin(psi); sin(psi) cos(psi)]*[U(1);U(2)])*dt;
X(3:4) = X(3:4) + [X(1);X(2)] * dt;

end