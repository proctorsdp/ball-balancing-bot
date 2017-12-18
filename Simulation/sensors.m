% sensors.m
%   Compute the output of rate gyros, accelerometers, and pressure sensors
%
%  Revised:
%   3/5/2010  - RB 
%   5/14/2010 - RB

function y = sensors(uu, P)

    % relabel the inputs
%    pn      = uu(1);
%    pe      = uu(2);
    pd      = uu(3);
%    u       = uu(4);
%    v       = uu(5);
%    w       = uu(6);
    phi     = uu(7);
    theta   = uu(8);
%    psi     = uu(9);
    p       = uu(10);
    q       = uu(11);
    r       = uu(12);
    F_x     = uu(13);
    F_y     = uu(14);
    F_z     = uu(15);
%    M_l     = uu(16);
%    M_m     = uu(17);
%    M_n     = uu(18);
    Va      = uu(19);
%    alpha   = uu(20);
%    beta    = uu(21);
%    wn      = uu(22);
%    we      = uu(23);
%    wd      = uu(24);
   

    % simulate rate gyros (units are rad/sec)
	sigma_gyro = 0.13 * pi/180; % for the ADXRS540
	
    y_gyro_x = p + random(); %*sigma_gyro;
    y_gyro_y = q + random(); %*sigma_gyro;
    y_gyro_z = r + random(); %*sigma_gyro;

    % simulate accelerometers (units of g)
	sigma_accel = 0.0025; % for the ADXL325
	
    y_accel_x = F_x/P.mass+P.gravity*sin(theta); %+ random()*sigma_accel;
    y_accel_y = F_y/P.mass-P.gravity*cos(theta)*sin(phi); % + random()*sigma_accel;
    y_accel_z = F_z/P.mass-P.gravity*cos(theta)*sin(phi); % + random()*sigma_accel;

    % simulate pressure sensors
	beta_diff_pres = 0.020; % kP for the MPXV5004G
	sigma_pres = 0.002; % kP for the MPXV5004G
	
    y_static_pres = P.rho*P.gravity*-pd;
    y_diff_pres = P.rho*Va^2/2 + beta_diff_pres + random()*sigma_pres;

    % construct total output
    y = [...
        y_gyro_x;...
        y_gyro_y;...
        y_gyro_z;...
        y_accel_x;...
        y_accel_y;...
        y_accel_z;...
        y_static_pres;...
        y_diff_pres;...
    ];

end

function out = random()
% Returs random number between -1 and 1;
out = rand()*2 -1;
end

