%% Roll_hold parameters
z_phi = .7; % Damping ratio for phi

w_n_phi = sqrt(abs(P.a_phi2)*P.delta_a_max/P.e_phi_max);
P.roll_kp = P.delta_a_max/P.e_phi_max*sign(P.a_phi2);
P.roll_kd = (2*z_phi*w_n_phi-P.a_phi1)/P.a_phi2;
P.roll_ki = 0;

%% Course Hold 
W_x = 20; % "Usually greater than 5"
z_x = 2; % Damping ratio for x

w_n_phi = sqrt(abs(P.a_phi2)*P.delta_a_max/P.e_phi_max);
w_n_x = 1/W_x*w_n_phi;
P.course_ki = w_n_x^2*P.Va0/P.gravity;
P.course_kp = 2*z_x*w_n_x*P.Va0/P.gravity;
P.course_kd = 0;

%% Sideslip Hold
z_beta = .8;

P.sideslip_kp = 0.1; %P.delta_r_max/P.e_beta_max * sign(P.a_beta2);
P.sideslip_ki = 1/P.a_beta2 * ((P.a_beta1 + P.a_beta2*P.sideslip_kp)/(2*z_beta))^2;
% Check this:
P.sideslip_kd = 0;

%% Pitch Hold
z_theta = .5;

w_n_theta = sqrt(P.a_theta2 + P.delta_e_max/P.e_theta_max*abs(P.a_theta3));

P.pitch_kp = P.delta_e_max/P.e_theta_max/5 * sign(P.a_theta3);
P.pitch_ki = 0;
P.pitch_kd = (2*z_theta*w_n_theta-P.a_theta1)/P.a_theta3; % Check this
P.K_theta_DC = P.pitch_kp*P.a_theta3/(P.a_theta2 + P.pitch_kp*P.a_theta3);

%% Airspeed from pitch hold
Wv2 = 10;
z_v2 = .85;

w_n_v2 = w_n_theta/Wv2;
P.airspeed_pitch_ki = -w_n_v2^2/(P.K_theta_DC*P.gravity);
P.airspeed_pitch_kp = (P.a_V1-2*z_v2*w_n_v2)/(P.K_theta_DC*P.gravity);
P.airspeed_pitch_kd = 0;

%% Altitude from pitch hold
z_h = .7;
Wh = 40;

w_n_h = w_n_theta/Wh;
P.altitude_ki = w_n_h^2/(P.K_theta_DC*P.Va0);
P.altitude_kp = (2*z_h*w_n_h)/(P.K_theta_DC*P.Va0);
P.altitude_kd = 0;

%% airspeed with throttle hold
z_v = .8;
w_n_v = 20;

P.airspeed_throttle_ki = w_n_v^2/P.a_V2;
P.airspeed_throttle_kp = (2*z_v*w_n_v-P.a_V1)/P.a_V2;
P.airspeed_throttle_kd = 0;

