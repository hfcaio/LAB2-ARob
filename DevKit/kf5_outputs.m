function [x, phi, theta, b, sb] = kf5_outputs(xhat, P)
%KF5_OUTPUTS  Roll, pitch e bias a partir do estado do filtro da secção 5.
%   xhat (N×6) = [x b], x = gravidade no corpo (g), b = bias (rad/s); P (6×6×N).
%   x = [-sin(theta); sin(phi)cos(theta); cos(phi)cos(theta)]
%   Devolve phi, theta em deg, b e sb = sqrt(diag(P)) do bias em deg/s.
x     = xhat(:,1:3);
phi   = atan2d( x(:,2), x(:,3));
theta = atan2d(-x(:,1), sqrt(x(:,2).^2 + x(:,3).^2));
b     = xhat(:,4:6) * 180/pi;
N     = size(xhat, 1);
sb    = zeros(N, 3);
for i = 1:3
    sb(:,i) = sqrt(squeeze(P(3+i, 3+i, :))) * 180/pi;
end
end
