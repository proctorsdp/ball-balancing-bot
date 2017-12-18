function y = autopilot(uu,P)
%
% autopilot for mavsim
% 
% Modification History:
%   2/11/2010 - RWB
%   5/14/2010 - RWB
%   9/30/2014 - RWB
%   

    % process inputs
    NN = 0;
%    pn       = uu(1+NN);  % inertial North position
%    pe       = uu(2+NN);  % inertial East position
    h        = uu(3+NN);  % altitude
    Va       = uu(4+NN);  % airspeed
%    alpha    = uu(5+NN);  % angle of attack
%    beta     = uu(6+NN);  % side slip angle
    phi      = uu(7+NN);  % roll angle
    theta    = uu(8+NN);  % pitch angle
    chi      = uu(9+NN);  % course angle
    p        = uu(10+NN); % body frame roll rate
    q        = uu(11+NN); % body frame pitch rate
    r        = uu(12+NN); % body frame yaw rate
%    Vg       = uu(13+NN); % ground speed
%    wn       = uu(14+NN); % wind North
%    we       = uu(15+NN); % wind East
%    psi      = uu(16+NN); % heading
%    bx       = uu(17+NN); % x-gyro bias
%    by       = uu(18+NN); % y-gyro bias
%    bz       = uu(19+NN); % z-gyro bias
    NN = NN+19;
    Va_c     = uu(1+NN);  % commanded airspeed (m/s)
    h_c      = uu(2+NN);  % commanded altitude (m)
    chi_c    = uu(3+NN);  % commanded course (rad)
    NN = NN+3;
    t        = uu(1+NN);   % time
    
    [delta, x_command] = autopilot_uavbook(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P);
    y = [delta; x_command];
end
    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% autopilot_uavbook
%   - autopilot defined in the uavbook
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_uavbook(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P)

    %----------------------------------------------------------
    % lateral autopilot
    if t==0
        % assume no rudder, therefore set delta_r=0
        phi_c   = course_hold(chi_c, chi, r, 1, P);
        delta_r = 0;%coordinated_turn_hold(beta, 1, P);
    else
        phi_c   = course_hold(chi_c, chi, r, 0, P);
        delta_r = 0;%coordinated_turn_hold(beta, 0, P);
	end
% 	phi_c = mod(abs(phi_c), pi)*sign(phi_c);
    delta_a = roll_hold(phi_c, phi, p, P);     
  
    
    %----------------------------------------------------------
    % longitudinal autopilot
    
    % define persistent variable for state of altitude state machine
    persistent altitude_state;
    % initialize persistent variable
    if t==0
		airspeed_from_pitch_hold(Va, Va_c, P, 1);
		altitude_from_pitch_hold(Va, h_c, h, P, 1);
		airspeed_with_throttle_hold(Va, Va_c, 1, P);
	end
	
    if h<=P.altitude_take_off_zone
		altitude_state = 1;
	elseif h<=h_c-P.altitude_hold_zone 
		altitude_state = 2;
	elseif h<=h_c+P.altitude_hold_zone
		altitude_state = 4;
	else
		altitude_state = 3;
	end
	
    % implement state machine
% 	altitude_state = 4;
    switch altitude_state
        case 1  % in take-off zone
            theta_c = P.takeoff_angle;
            delta_t = .7;
        case 2  % climb zone
            theta_c = airspeed_from_pitch_hold(Va, Va_c, P, 0);
            delta_t = .7;
        case 3 % descend zone
            theta_c = airspeed_from_pitch_hold(Va, Va_c, P, 0);
            delta_t = .2;
        case 4 % altitude hold zone
            theta_c = altitude_from_pitch_hold(Va, h_c, h, P, 0);
            delta_t = airspeed_with_throttle_hold(Va, Va_c, 0, P);
    end
%     theta_c = mod(abs(theta_c), pi)*sign(theta_c);
    delta_e = pitch_hold(theta_c, theta, q, P);
    % artificially saturation delta_t
    delta_t = sat(delta_t,1,0);
    
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        %theta_c*P.K_theta_DC;... % theta
        theta_c;
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
    y = [delta; x_command];
 
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [delta_a] = roll_hold(phi_c, phi, p, P)
%     delta_a = P.roll_kp*(phi_c-phi) + P.roll_ki/s *(phi_c-phi) - P.roll_kd*p;
	persistent integrator;
    persistent differentiator;
    persistent error_dl;
    if isempty(integrator) % reset persistent variables
        integrator = 0;
        differentiator = 0;
        error_dl = 0; % _dl means delayed by one time step
    end
    error = phi_c - phi;
    integrator = integrator + (P.Ts/2)*(error + error_dl);
    differentiator = (2*P.tau-P.Ts)/(2*P.tau-P.Ts)*differentiator + 2/(2*P.tau+P.Ts)*(error-error_dl);
    error_dl = error;
    delta_a = sat(P.roll_kp*error + P.roll_ki*integrator + P.roll_kd *differentiator*p, P.delta_a_max);
    if P.roll_ki ~= 0
        u_unsat = P.roll_kp*error + P.roll_ki*integrator + P.roll_kd*differentiator*p;
        integrator = integrator + P.Ts/P.roll_ki * (delta_a-u_unsat);
    end
end

function phi_c = course_hold(chi_c, chi, r, flag, P)
	persistent integrator;
    persistent differentiator;
    persistent error_dl;
    if flag==1 || isempty(integrator) % reset persistent variables
        integrator = 0;
        differentiator = 0;
        error_dl = 0; % _dl means delayed by one time step
    end
    error = chi_c - chi;
    integrator = integrator + (P.Ts/2)*(error + error_dl);
    differentiator = (2*P.tau-P.Ts)/(2*P.tau-P.Ts)*differentiator + 2/(2*P.tau+P.Ts)*(error-error_dl);
    error_dl = error;
    phi_c = sat(P.course_kp*error + P.course_ki*integrator + P.course_kd *differentiator*r, P.delta_a_max);
    if P.course_ki ~= 0
        u_unsat = P.course_kp*error + P.course_ki*integrator + P.course_kd*differentiator*r;
        integrator = integrator + P.Ts/P.course_ki * (phi_c-u_unsat);
    end
end

function delta_r = coordinated_turn_hold(beta, flag, P) % sideslip hold
	persistent integrator;
    persistent differentiator;
    persistent error_dl;
    if flag==1 || isnan(integrator) || isempty(integrator) % reset persistent variables
        integrator = 0;
        differentiator = 0;
        error_dl = 0; % _dl means delayed by one time step
    end
    error = beta;
    integrator = integrator + (P.Ts/2)*(error + error_dl);
    differentiator = (2*P.tau-P.Ts)/(2*P.tau-P.Ts)*differentiator + 2/(2*P.tau+P.Ts)*(error-error_dl);
    error_dl = error;
    delta_r = sat(P.sideslip_kp*error + P.sideslip_ki*integrator + P.sideslip_kd *differentiator*r, P.delta_r_max);
    if P.course_ki ~= 0
        u_unsat = P.sideslip_kp*error + P.sideslip_ki*integrator + P.sideslip_kd*differentiator*r;
        integrator = integrator + P.Ts/P.sideslip_ki * (delta_r-u_unsat);
    end
end

function delta_e = pitch_hold(theta_c, theta, q, P)
%     delta_e = P.pitch_kp*(theta_c-theta)-P.pitch_kd*q;
	
	persistent integrator;
    persistent differentiator;
    persistent error_dl;
    if isempty(integrator)% reset persistent variables
        integrator = 0;
        differentiator = 0;
        error_dl = 0; % _dl means delayed by one time step
    end
    error = theta_c - theta;
    integrator = integrator + (P.Ts/2)*(error + error_dl);
    differentiator = (2*P.tau-P.Ts)/(2*P.tau-P.Ts)*differentiator + 2/(2*P.tau+P.Ts)*(error-error_dl);
    error_dl = error;
    delta_e = sat(P.pitch_kp*error + P.pitch_ki*integrator + P.pitch_kd *differentiator*q, P.delta_e_max);
    if P.pitch_ki ~= 0
        u_unsat = P.pitch_kp*error + P.pitch_ki*integrator + P.pitch_kd*differentiator*q;
        integrator = integrator + P.Ts/P.pitch_ki * (delta_e-u_unsat);
    end
end

function theta_c = airspeed_from_pitch_hold(Va, Va_c, P, flag)
	persistent integrator;
    persistent differentiator;
    persistent error_dl;
    if flag==1 || isempty(integrator) % reset persistent variables
        integrator = 0;
        differentiator = 0;
        error_dl = 0; % _dl means delayed by one time step
    end
    error = Va_c - Va;
    integrator = integrator + (P.Ts/2)*(error + error_dl);
    differentiator = (2*P.tau-P.Ts)/(2*P.tau-P.Ts)*differentiator + 2/(2*P.tau+P.Ts)*(error-error_dl);
    error_dl = error;
    theta_c = sat(P.airspeed_pitch_kp*error + P.airspeed_pitch_ki*integrator + P.airspeed_pitch_kd *differentiator, P.delta_e_max);
    if P.airspeed_pitch_ki ~= 0
        u_unsat = P.airspeed_pitch_kp*error + P.airspeed_pitch_ki*integrator + P.airspeed_pitch_kd*differentiator;
        integrator = integrator + P.Ts/P.airspeed_pitch_ki * (theta_c-u_unsat);
    end
end

function theta_c = altitude_from_pitch_hold(Va, h_c, h, P, flag)
%     theta_c = P.altitude_kp*(h_c-h)+P.altitude_ki/s*(h_c-h);
% 	theta_c = altitude_from_pitch_hold_pid(h_c, h, flag, P.altitude_kp, P.altitude_ki, 0, P.delta_e_max, P.Ts, P.tau);
	persistent integrator;
    persistent differentiator;
    persistent error_dl;
    if flag==1 || isempty(integrator)% reset persistent variables
        integrator = 0;
        differentiator = 0;
        error_dl = 0; % _dl means delayed by one time step
    end
    error = h_c - h;
    integrator = integrator + (P.Ts/2)*(error + error_dl);
    differentiator = (2*P.tau-P.Ts)/(2*P.tau-P.Ts)*differentiator + 2/(2*P.tau+P.Ts)*(error-error_dl);
    error_dl = error;
    theta_c = sat(P.altitude_kp*error + P.altitude_ki*integrator + P.altitude_kd *differentiator, P.delta_a_max);
    if P.altitude_ki ~= 0
        u_unsat = P.altitude_kp*error + P.altitude_ki*integrator + P.altitude_kd*differentiator;
        integrator = integrator + P.Ts/P.altitude_ki * (theta_c-u_unsat);
    end
end

function delta_t = airspeed_with_throttle_hold(Va, Va_c, flag, P)
%     delta_t = deltat_star+k_pv*(Va_c-Va)+k_iv/s*(Va_c-Va);
% 	delta_t = airspeed_from_throttle_pid(Va_c, Va, flag, P.airspeed_throttle_kp, P.airspeed_throttle_ki, 0, 1, P.Ts, P.tau);
	persistent integrator;
    persistent differentiator;
    persistent error_dl;
    if flag==1 || isempty(integrator)% reset persistent variables
        integrator = 0;
        differentiator = 0;
        error_dl = 0; % _dl means delayed by one time step
    end
    error = Va_c - Va;
    integrator = integrator + (P.Ts/2)*(error + error_dl);
    differentiator = (2*P.tau-P.Ts)/(2*P.tau-P.Ts)*differentiator + 2/(2*P.tau+P.Ts)*(error-error_dl);
    error_dl = error;
    delta_t = sat(P.airspeed_throttle_kp*error + P.airspeed_throttle_ki*integrator + P.airspeed_throttle_kd *differentiator, 1, 0);
    if P.airspeed_throttle_ki ~= 0
        u_unsat = P.airspeed_throttle_kp*error + P.airspeed_throttle_ki*integrator + P.airspeed_throttle_kd*differentiator;
        integrator = integrator + P.Ts/P.airspeed_throttle_ki * (delta_t-u_unsat);
    end
end

  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sat
%   - saturation function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function out = sat(in, up_limit, low_limit)
  if nargin == 2
	  low_limit = -up_limit;
  end
  
  if in > up_limit
      out = up_limit;
  elseif in < low_limit
      out = low_limit;
  else
      out = in;
  end
end
  
 