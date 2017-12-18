% function [T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]...
%     = compute_tf_model(x_trim,u_trim,P)
    % x_trim is the trimmed state,
    % u_trim is the trimmed input

    T=P.Jx*P.Jz - P.Jxz^2;
    T1=(P.Jxz*(P.Jx-P.Jy+P.Jz))/T; T2=(P.Jz*(P.Jz-P.Jy)+P.Jxz^2)/T; T3=P.Jz/T;
    T4=P.Jxz/T; T5=(P.Jz-P.Jx)/P.Jy;  T6=P.Jxz/P.Jy;
    T7=((P.Jx-P.Jy)*P.Jx+P.Jxz^2)/T; T8=P.Jx/T;

    C_p_p = T3*P.C_ell_p + T4*P.C_n_p;
    C_p_delta_a = T3*P.C_ell_delta_a + T4*P.C_n_delta_a;


    % add stuff here
    Va_trim = sqrt(x_trim(4)^2 + x_trim(5)^2 + x_trim(6)^2);
	Va = Va_trim;
    alpha_star = atan2(x_trim(6),x_trim(4));
    deltae_star = u_trim(1);
    deltat_star = u_trim(4);
    k = P.k_motor;
    theta_star = x_trim(8);
    chi_star = alpha_star;

    P.a_phi1 = -1/2 * P.rho *Va^2*P.S_wing*P.b*C_p_p*P.b/(2*Va);
    P.a_phi2 = 1/2 *P.rho*Va^2*P.S_wing*P.b*C_p_delta_a;
	
	P.a_beta0 = P.rho*Va*P.S_wing/2/P.mass;
    P.a_beta1 = -P.a_beta0*P.C_Y_beta;
    P.a_beta2 = P.a_beta0*P.C_Y_delta_r;

    a_theta0 = P.rho*Va^2*P.c*P.S_wing/(2*P.Jy);
    P.a_theta1 = -a_theta0*P.C_m_q*P.c/(2*Va);
    P.a_theta2 = -a_theta0*P.C_m_alpha;
    P.a_theta3 = a_theta0*P.C_m_delta_e;

    a_V0 = P.rho*P.S_wing*P.C_prop/P.mass;
    P.a_V1 = P.rho*Va_trim*P.S_wing/P.mass*(P.C_D_0 + P.C_D_alpha * alpha_star + P.C_D_delta_e*deltae_star);
    P.a_V1 = P.a_V1 + a_V0*Va_trim;
    P.a_V2 = a_V0*k^2*deltat_star;
    P.a_V3 = P.gravity*cos(theta_star-alpha_star);


    % define transfer functions
    P.T_phi_delta_a   = tf([P.a_phi2],[1,P.a_phi1,0]);
    P.T_chi_phi       = tf([P.gravity/Va_trim],[1,0]);
    P.T_theta_delta_e = tf(P.a_theta3,[1,P.a_theta1,P.a_theta2]);
    P.T_h_theta       = tf([Va_trim],[1,0]);
    P.T_h_Va          = tf([theta_star],[1,0]);
    P.T_Va_delta_t    = tf([P.a_V2],[1,P.a_V1]);
    P.T_Va_theta      = tf([-P.a_V3],[1,P.a_V1]);
    P.T_v_delta_r     = tf([Va_trim*P.a_beta2],[1,P.a_beta1]);
