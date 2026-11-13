%% Covariâncias dos sensores usadas em Q e R (secção 3, exp. C em hover, 9-30 s)
%  Valores impressos por sec3_stats.m. Convenção do grupo:
%    R = incerteza do gyro (ruído de processo)
%    Q = incerteza do inclinómetro (ruído de medida)

R_p     = 6.38e-4;   % (rad/s)^2  var(gyro x)   (std 1.447 deg/s)
R_q     = 7.63e-4;   % (rad/s)^2  var(gyro y)   (std 1.583 deg/s)
Q_phi   = 4.06e-5;   % rad^2      var(phi_m)    (std 0.365 deg)
Q_theta = 5.81e-5;   % rad^2      var(theta_m)  (std 0.437 deg)
