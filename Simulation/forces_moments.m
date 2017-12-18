% forces_moments.m
%   Computes the forces and moments acting on the airframe. 
%
%   Output is
%       F     - forces
%       M     - moments
%       Va    - airspeed
%       alpha - angle of attack
%       beta  - sideslip angle
%       wind  - wind vector in the inertial frame
%

function out = forces_moments(x, delta, P)

    % relabel the inputs
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
    phi     = x(7); % roll
    theta   = x(8); % pitch
    psi     = x(9); % yaw
    p       = x(10);
    q       = x(11);
    r       = x(12);
    delta_A = delta(1);
    delta_B = delta(2);
    delta_C = delta(3);
    
    % compute wind data in NED
    ctheta = cos(theta); cphi = cos(phi); cpsi = cos(psi);
    stheta = sin(theta); sphi = sin(phi); spsi = sin(psi);

	R_roll = [
          1, 0, 0;
          0, cphi, sphi;
          0, -sphi, cphi];
  R_pitch = [
          ctheta, 0, -stheta;
          0, 1, 0;
          stheta, 0, ctheta];
  R_yaw = [
          cpsi, spsi, 0;
          -spsi, cpsi, 0;
          0, 0, 1];
  R = R_roll*R_pitch*R_yaw;  
	
    CM = R' * [pn; pe; pd+P.R_cm];
    % compute air data
    Vab = [u; v; w;];
    Va = sqrt(Vab(1)^2 + Vab(2)^2 + Vab(3)^2);
 
    % compute external forces on the robot
    A_g = P.mass*P.gravity;
    Grav = [A_g*stheta*cpsi; 
            A_g*stheta*spsi; 
            A_g*ctheta;];
    Force = Grav;	% need anything else here?
	Force(3) = 0;	% Ball on the ground. Vertical forces cancel?
						
    % compute external torques on the robot											 
   

	Torque = [A_g*sqrt(CM(2)^2 + CM(3)^2);
			  A_g*sqrt(CM(1)^2 + CM(3)^2);
			  P.k_m*(delta_A + delta_B + delta_C)];
	
    out = [Force(1) Force(2) Force(3) Torque(1) Torque(2) Torque(3) Va]';
end



