
function drawVehicle(uu,V,F,patchcolors)

    % process inputs to function
    px       = uu(1);       % inertial North position     
    py       = uu(2);       % inertial East position
    pd       = uu(3);           
    u        = uu(4);       
    v        = uu(5);       
    w        = uu(6);       
    phi      = uu(7);       % roll angle         
    theta    = uu(8);       % pitch angle     
    psi      = uu(9);       % yaw angle     
    p        = uu(10);       % roll rate
    q        = uu(11);       % pitch rate     
    r        = uu(12);       % yaw rate    
    t        = uu(13);       % time

    % define persistent variables 
    persistent vehicle_handle;
    persistent Vertices
    persistent Faces
    persistent facecolors
    
    % first time function is called, initialize plot and persistent vars
    if t==0
        figure(1), clf
        [Vertices,Faces,facecolors] = defineVehicleBody;
        vehicle_handle = drawVehicleBody(Vertices,Faces,facecolors,...
                                               px,py,pd,phi,theta,psi,...
                                               [],'normal');
        title('Vehicle')
        xlabel('px')
        ylabel('py')
        zlabel('z')
        view(32,47)  % set the view angle for figure
        axis([-5,5,-5,5,-1,5]);
        grid on
        hold on
        
    % at every other time step, redraw base and rod
    else 
        drawVehicleBody(Vertices,Faces,facecolors,...
                           px,py,pd,phi,theta,psi,...
                           vehicle_handle);
    end
end

  
%=======================================================================
% drawVehicle
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawVehicleBody(V,F,patchcolors,...
                                     pn,pe,pd,phi,theta,psi,...
                                     handle,mode)
%   pd = 0;
  V = rotate(V, phi, theta, psi);  % rotate vehicle
  V = translate(V, pn, pe, pd);  % translate vehicle
  % transform vertices from NED to XYZ (for matlab rendering)
  R = [...
      1, 0, 0;...
      0, 1, 0;...
      0, 0, 1;...
      ];
  V = R*V;
  
  if isempty(handle),
  handle = patch('Vertices', V', 'Faces', F,...
                 'FaceVertexCData',patchcolors,...
                 'FaceColor','flat',...
                 'EraseMode', mode);
  else
    set(handle,'Vertices',V','Faces',F);
    drawnow
  end
end

%%%%%%%%%%%%%%%%%%%%%%%
function pts=rotate(pts,phi,theta,psi)

  % define rotation matrix (right handed)
  R_roll = [...
          1, 0, 0;...
          0, cos(phi), sin(phi);...
          0, -sin(phi), cos(phi)];
  R_pitch = [...
          cos(theta), 0, -sin(theta);...
          0, 1, 0;...
          sin(theta), 0, cos(theta)];
  R_yaw = [...
          cos(psi), sin(psi), 0;...
          -sin(psi), cos(psi), 0;...
          0, 0, 1];
  R = R_roll*R_pitch*R_yaw;  
    % note that R above either leaves the vector alone or rotates
    % a vector in a left handed rotation.  We want to rotate all
    % points in a right handed rotation, so we must transpose
  R = R';

  % rotate vertices
  pts = R*pts;
  
end
% end rotateVert

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function pts = translate(pts,pn,pe,pd)

  pts = pts + repmat([pn;pe;pd],1,size(pts,2));
  
end

% end translate


%=======================================================================
% defineVehicleBody
%=======================================================================
function [V,F,facecolors] = defineVehicleBody

% Define the vertices (physical location of vertices
V = [...
			 1,  1, 3; % "body"
			 1, -1, 3;
			-1, -1, 3;
			-1,  1, 3;
			 1,  1, 1;
			 1, -1, 1;
			-1, -1, 1;
			-1,  1, 1;
			
			 0,  0,  1; %"ball"
			 1,  0,  0;
			 0, -1,  0;
			-1,  0,  0;
			 0,  1,  0;
			 0,  0,  -1;
       ]';

% define faces as a list of vertices numbered above
  F = [...
			1,2,3,4; % "body"
			1,2,6,5;
			2,3,7,6;
			3,4,8,7;
			1,4,8,5;
			5,6,7,8;
			
			9, 9, 10, 11; %"ball"
			9, 9, 11, 12;
			9, 9, 12, 13;
			9, 9, 13, 10;
			14, 14, 10, 11;
			14, 14, 11, 12;
			14, 14, 12, 13;
			14, 14, 13, 10;
        ];

% define colors for each face    
  red    = [1, 0, 0];
  green  = [0, 1, 0];
  blue   = [0, 0, 1];
  yellow = [1, 1, 0];
  cyan   = [0, 1, 1];

  facecolors = [...   
    blue;...     % body
    blue;...    
    blue;...    
    blue;...    
    blue;...     
    blue;...  
	red;		 %"ball"
	red;
	red;
	red;
	red;
	red;
	red;
	red;
    ];
end
  