function [A_lon,B_lon,A_lat,B_lat] = compute_ss_model(filename,x_trim,u_trim)
% x_trim is the trimmed state,
% u_trim is the trimmed input
  
% add stuff here  
[A, B, ~, ~] = linmod(filename, x_trim, u_trim);
E1 = [...
   %pn pe pd u v w ph th ps p q r
    0  0  0  1 0 0 0  0  0  0 0 0;
    0  0  0  0 0 1 0  0  0  0 0 0;
    0  0  0  0 0 0 0  0  0  0 1 0;
    0  0  0  0 0 0 0  1  0  0 0 0;
    0  0  0  0 0 0 0  0  0  0 0 0;
    ];
E2 = [...
   %e a r t
    1 0 0 0;
    0 0 0 1;
    ];
A_lon = E1*A*E1';
B_lon = E1*B*E2';

E3 = [...
   %pn pe pd u v w ph th ps p q r
    0  0  0  0 1 0 0  0  0  0 0 0;
    0  0  0  0 0 0 0  0  0  1 0 0;
    0  0  0  0 0 0 0  0  0  0 0 1;
    0  0  0  0 0 0 1  0  0  0 0 0;
    0  0  0  0 0 0 0  0  1  0 0 0;
    ];
E4 = [...
   %e a r t
    0 1 0 0;
    0 0 1 0;
    ];
A_lat = E3 *A *E3';
B_lat = E3 *B *E4';
