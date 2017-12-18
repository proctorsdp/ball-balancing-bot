% gps.m
%   Compute the output of gps sensor
%
%  Revised:
%   3/5/2010 - RB 
%   5/14/2010 - RB

function y = gps(uu, P)

    % relabel the inputs
    Va      = uu(1);
%    alpha   = uu(2);
%    beta    = uu(3);
    wn      = uu(4);
    we      = uu(5);
%    wd      = uu(6);
    pn      = uu(7);
    pe      = uu(8);
    pd      = uu(9);
%    u       = uu(10);
%    v       = uu(11);
%    w       = uu(12);
%    phi     = uu(13);
%    theta   = uu(14);
    psi     = uu(15);
%    p       = uu(16);
%    q       = uu(17);
%    r       = uu(18);
    t       = uu(19);
    
 
    % construct North, East, and altitude GPS measurements
	persistent vnn;
	persistent vne;
	persistent vnh;
	if isempty(vnn)
		vnn = 0;
		vne = 0;
		vnh = 0;
	end;
	vnn = exp(-P.Ts/1100)* vnn + random(0.21);
	vne = exp(-P.Ts/1100)* vne + random(0.21);
	vnh = exp(-P.Ts/1100)* vnh + random(0.40);
    y_gps_n = pn + vnn;
    y_gps_e = pe + vne; 
    y_gps_h = -pd + vnh; 
    
    % construct groundspeed and course measurements
	Vg = sqrt(wn^2 + we^2);
	sigma_Vg = 0.21;
	if Vg==0
		sigma_chi = 0;
	else
		sigma_chi = sigma_Vg/Vg;
	end
    y_gps_Vg     = sqrt((Va*cos(psi)+wn)^2 + (Va*sin(psi)+we)^2) + random(sigma_Vg);
    y_gps_course = atan2(Va*sin(psi)+we, Va*cos(psi)+wn) + random(sigma_chi);

    % construct total output
    y = [...
        y_gps_n;...
        y_gps_e;...
        y_gps_h;...
        y_gps_Vg;...
        y_gps_course;...
        ];
    
end

function out = random(sigma)
% Returs random number between -1 and 1;
out = randn()*2 -1;
out = out*sigma;
end


