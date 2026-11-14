function [x, phi, theta, b, sb] = kf5_outputs(xhat, P)
%KF5_OUTPUTS roll, pitch and bias from the state of the section 5 filter
%   xhat (N x 6) = [x b], x = gravity in the body frame (g), b = bias (rad/s), P (6 x 6 x N)
%   x = [-sin(theta); sin(phi)cos(theta); cos(phi)cos(theta)]
%   phi, theta in deg; b and sb = sqrt(diag(P)) of the bias in deg/s

% angles
x     = xhat(:,1:3);
phi   = atan2d( x(:,2), x(:,3));
theta = atan2d(-x(:,1), sqrt(x(:,2).^2 + x(:,3).^2));

% bias and standard deviation
b     = xhat(:,4:6) * 180/pi;
N     = size(xhat, 1);
sb    = zeros(N, 3);
for i = 1:3
    sb(:,i) = sqrt(squeeze(P(3+i, 3+i, :))) * 180/pi;
end
end
