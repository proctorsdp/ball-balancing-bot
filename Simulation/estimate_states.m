% estimate_states
%   - estimate the MAV states using gyros, accels, pressure sensors, and
%   GPS. This will be run 10 times for each P.Ts
%
% Outputs are:
%   pnhat    - estimated North position, 
%   pehat    - estimated East position, 
%   hhat     - estimated altitude, 
%   Vahat    - estimated airspeed, 
%   alphahat - estimated angle of attack
%   betahat  - estimated sideslip angle
%   phihat   - estimated roll angle, 
%   thetahat - estimated pitch angel, 
%   chihat   - estimated course, 
%   phat     - estimated roll rate, 
%   qhat     - estimated pitch rate, 
%   rhat     - estimated yaw rate,
%   Vghat    - estimated ground speed, 
%   wnhat    - estimate of North wind, 
%   wehat    - estimate of East wind
%   psihat   - estimate of heading angle
% 
% 
% Modified:  3/15/2010 - RB
%            5/18/2010 - RB
%

function xhat = estimate_states(uu, P)
	persistent in_n;
	if isempty(in_n)
		in_n = zeros(19,1);
	end
   % rename inputs
   y_gyro_x      = uu(1);
   y_gyro_y      = uu(2);
   y_gyro_z      = uu(3);
   y_accel_x     = uu(4);
   y_accel_y     = uu(5);
   y_accel_z     = uu(6);
   y_static_pres = uu(7);
   y_diff_pres   = uu(8);
   y_gps_n       = uu(9);
   y_gps_e       = uu(10);
   y_gps_h       = uu(11);
   y_gps_Vg      = uu(12);
   y_gps_course  = uu(13);
   t             = uu(14);

	% estimate these states
	pnhat    = LPF(in_n(9), y_gps_n, .9);
	pehat    = LPF(in_n(10), y_gps_e, .9);
	hhat     = LPF(in_n(7), y_static_pres, .9)/P.rho/P.gravity; % add the LPF(gps_h) and divide by 2?
	
	Vahat    = sqrt(LPF(in_n(8), y_diff_pres, .5)*2/P.rho);
	
	phat     = LPF(in_n(1), y_gyro_x, .5);
	qhat     = LPF(in_n(2), y_gyro_y, .5);
	rhat     = LPF(in_n(3), y_gyro_z, .5);
	
	y = [y_accel_x;
		 y_accel_y;
		 y_accel_z;];
	
	[phihat, thetahat] = kallmanFilterPhiTheta(y, phat, qhat, rhat, Vahat, P.gravity, P.Ts_sensors, t);
% 	phihat   = atan(LPF(in_n(5), y_accel_y, 0.8)/LPF(in_n(6), y_accel_z, .8));
% 	thetahat = asin(LPF(in_n(4), y_accel_x, .8)/P.gravity);
	
	y = [y_gps_n; y_gps_e; y_gps_Vg; y_gps_course; 0; 0];
	u = [Vahat; qhat; rhat; phihat; thetahat;];

	[pnhat, pehat, Vghat, chihat, wnhat, wehat, psihat]=kallmanFilterGps(y, u, P.gravity, P.Ts_sensors, t);
	
% 	Vghat    = LPF(in_n(12), y_gps_Vg, .5);
% 	wnhat    = 0; %LPF(xhatn(14), , .5);
% 	wehat    = 0; %LPF(xhatn(15), , .5);
% 	psihat   = LPF(in_n(13), y_gps_course, .5); %LPF(xhatn(16), , .5);
% 	chihat   = LPF(in_n(13), y_gps_course, .5);
	   
    % not estimating these states 
    alphahat = 0;
    betahat  = 0;
    bxhat    = 0;
    byhat    = 0;
    bzhat    = 0;
    
	xhat = [...
		pnhat;...
		pehat;...
		hhat;...
		Vahat;...
		alphahat;...
		betahat;...
		phihat;...
		thetahat;...
		chihat;...
		phat;...
		qhat;...
		rhat;...
		Vghat;...
		wnhat;...
		wehat;...
		psihat;...
		bxhat;...
		byhat;...
		bzhat;...
	];

	in_n = uu;
end

function [pnhat, pehat, Vghat, chihat, wnhat, wehat, psihat]=kallmanFilterGps(y, u, g, Ts, t)
	persistent P;
	persistent X;
	persistent Q;
	persistent R;
	if t == 0
		P = eye(7,7);
		X = [0, 0, 0, 0, 0, 0, 0]';
		Q = [1e-1 0    0    0    0    0    0;
			 0    1e-1 0    0    0    0    0;
			 0    0    1e-1 0    0    0    0;
			 0    0    0    1e-1 0    0    0;
			 0    0    0    0    1e-1 0    0;
			 0    0    0    0    0    1e-1 0;
			 0    0    0    0    0    0    1e-1]*1e-4; % process
		R = [0.21 0    0    0    0    0;
			 0    0.21 0    0    0    0;
			 0    0    0.21 0    0    0;
			 0    0    0    0.21 0    0;
			 0    0    0    0    0.20 0;
			 0    0    0    0    0    0.20]; % Sigma for the accel.
	end
	pn  = X(1); pe  = X(2); Vg  = X(3); chi = X(4);
	wn  = X(5); we  = X(6); psi = X(7);
	Va = u(1); q = u(2); r = u(3); phi = u(4); theta = u(5);
	N = 1;
	for i=1:N
		psidot = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta);
		spsi = sin(psi); cpsi = cos(psi);
		if Vg == 0
			Vg = Va;
		end
		Vgdot = ((Va*cpsi+wn)*(-Va*psidot*spsi)+(Va*spsi+we)*(Va*psidot*cpsi))/Vg;
		X = X + Ts/N * [Vg*cos(chi);
						Vg*sin(chi);
						Vgdot; 
						g/Vg*tan(phi)*cos(chi-psi);
						0;
						0;
						psidot];  % Add noise?
		% I know... I'm a bit anal here about aligning columns..
		A = [0 0 cos(chi)					   -Vg*sin(chi)				     0						 0						0;
			 0 0 sin(chi)					   Vg*cos(chi)				     0						 0						0;
			 0 0 -Vgdot/Vg                     0							 -psidot*Va*sin(psi)/Vg  psidot*Va*cos(psi)/Vg  -psidot*Va*(wn*cos(psi) + we*sin(psi))/Vg;
			 0 0 -g/Vg^2*tan(phi)*cos(chi-psi) -g/Vg^2*tan(phi)*sin(chi-psi) 0						 0						g/Vg*tan(phi)*sin(chi-psi);
			 0 0 0							   0							 0						 0						0;
			 0 0 0							   0							 0						 0						0;
			 0 0 0							   0							 0						 0						0];
		P = P + Ts/N*(A*P + P*A' + Q);
	end
		C = [1    0			0		   0			0    0    0;
			 0    1			0		   0			0    0    0;
			 0    0			1		   0			0    0    0;
			 0    0			0		   1			0    0    0;
			 0    0			-cos(chi)  Vg*sin(chi)  1    0    -Va*sin(psi);
			 0    0			-sin(chi)  -Vg*cos(chi) 0    1    Va*cos(psi)];
		L = P*C'*(R+C*P*C')^-1;
		P = (eye(7,7) -L*C)*P;
		h = [pn;
			 pe;
			 Vg;
			 chi;
			 Va*cos(psi)+wn-Vg*cos(chi);
			 Va*sin(psi)+we-Vg*sin(chi)];
		X = X + L*(y-h);
		
	pnhat  = X(1);
	pehat  = X(2);
	Vghat  = X(3);
	chihat = X(4);
	wnhat  = X(5);
	wehat  = X(6);
	psihat = X(7);
end

function [phihat, thetahat]=kallmanFilterPhiTheta(y, p, q, r, Va, g, Ts, t)
	persistent P;
	persistent X;
	persistent Q;
	persistent R;
	if t == 0
		P = eye(2,2);
		X = [0; 0];
		Q = [1e-7  0;
			 0  1e-6]; % process
		R = [1   0   0;
			 0   1   0;
			 0   0   1] * 0.0025*9.8; % Sigma for the accel.
	end
	phi = X(1);
	theta = X(2);
	N = 10;
	for i=1:N
		X = X + Ts/N * [p+q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
						q*cos(phi)-r*sin(phi)]; % + [random(.0025); random(.0025)];								% Add noise?
		A = [q*cos(phi)*tan(theta)-r*sin(phi)*tan(theta)  (q*sin(phi)+r*cos(phi))/cos(theta)^2;
			 -q*sin(phi)-r*cos(phi)						  0];
		P = P + Ts/N*(A*P + P*A' + Q);
	end
		C = [0						   q*Va*cos(theta)+g*cos(theta);
			 -g*cos(phi)*cos(theta)    -r*Va*sin(theta)-p*Va*cos(theta)+g*sin(phi)*sin(theta);
		     g*sin(phi)*cos(theta)	   (q*Va+g*cos(phi))*sin(theta)];
		L = P*C'*(R+C*P*C')^-1;
		P = (eye(2,2) -L*C)*P;
		h = [q*Va*sin(theta) + g*sin(theta);
			 r*Va*cos(theta)-p*Va*sin(theta)-g*cos(theta)*sin(phi);
			 -q*Va*cos(theta)-g*cos(theta)*cos(phi)];
		X = X + L*(y-h);
% 	endR
	X = wrap_angle(X);
	phihat = X(1);
	thetahat = X(2);
end


function y_n1 = LPF(yn, un, alpha)
	% Computes the lowpass filter for the next sample
	% alpha should be in the range of [0, 1]
	y_n1 = alpha*yn + (1-alpha)*un;
end

function out = random(sigma)
% Returs random number between -1 and 1;
out = (rand()*2 -1) * sigma;
end
