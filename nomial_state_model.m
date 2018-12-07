function dx = nomial_state_model( x, omxDt, axDt )
% the function is for generating nomial state 
% x = [   pgi',   qgi',   vgi',   ba',   bg', ... 
%         1~3     4~7    8~10   11~13  14~16 
%         pgi1,   qgi1,   pgi2,   qgi2   ];
%        17~19   20~23   24~26   27~30    

pgi_nom_index = 1:3; qgi_nom_index = 4:7; vgi_nom_index = 8:10;
ba_nom_index = 11:13; bg_nom_index = 14:16;
pgi1_nom_index = 17:19; qgi1_nom_index = 20:23;
pgi2_nom_index = 24:26; qgi2_nom_index = 27:30;

global gg;

wm = omxDt-x(bg_nom_index);
OM = [ 0.5*[     0, -wm(1), -wm(2), -wm(3); ];... % quaternion differentiation
       0.5*[ wm(1),      0,  wm(3), -wm(2); ];...
       0.5*[ wm(2), -wm(3),      0,  wm(1); ];...
       0.5*[ wm(3),  wm(2), -wm(1),      0; ]; ];

dx = zeros( 30, 1 );
dx(pgi_nom_index) = x(vgi_nom_index);
dx(qgi_nom_index) = OM*x(qgi_nom_index); 
dx(vgi_nom_index) = q2r(x(qgi_nom_index))*(axDt-x(ba_nom_index))-gg;
dx(ba_nom_index)  = 0;
dx(bg_nom_index)  = 0;

dx(pgi1_nom_index) = 0;
dx(qgi1_nom_index) = 0;
dx(pgi2_nom_index) = 0;
dx(qgi2_nom_index) = 0;

end

