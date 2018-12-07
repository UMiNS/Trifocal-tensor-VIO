function x = nom_plus_err( x_nom, dx )
% nomial state + error state
% x_nom = x_nom+dx

% nomial state 
% x = [   pgi',   qgi',   vgi',   ba',   bg', ... 
%         1~3     4~7    8~10   11~13  14~16 
%         pgi1,   qgi1,   pgi2,   qgi2   ];
%        17~19   20~23   24~26   27~30    
pgi_nom_index = 1:3; qgi_nom_index = 4:7; vgi_nom_index = 8:10;
ba_nom_index = 11:13; bg_nom_index = 14:16;
pgi1_nom_index = 17:19; qgi1_nom_index = 20:23;
pgi2_nom_index = 24:26; qgi2_nom_index = 27:30;

% dx = [   pgi',   thgi',   vgi',   ba',   bg', ... 
%          1~3     4~6      7~9    10~12  13~15 
%          pgi1,   thgi1,   pgi2,   thgi2   ];
%         16~18   19~21    22~24   25~27
pgi_index = 1:3; thgi_index = 4:6; vgi_index = 7:9;
ba_index = 10:12; bg_index = 13:15;
pgi1_index = 16:18; thgi1_index = 19:21;
pgi2_index = 22:24; thgi2_index = 25:27;

x = zeros( 30, 1 );
x(pgi_nom_index) = x_nom(pgi_nom_index)+dx(pgi_index);
x(qgi_nom_index) = qprod( x_nom(qgi_nom_index), [1; 0.5*dx(thgi_index)] );
x(vgi_nom_index) = x_nom(vgi_nom_index)+dx(vgi_index);

x(ba_nom_index) = x_nom(ba_nom_index)+dx(ba_index);
x(bg_nom_index) = x_nom(bg_nom_index)+dx(bg_index);

x(pgi1_nom_index) = x_nom(pgi1_nom_index)+dx(pgi1_index);
x(qgi1_nom_index) = qprod( x_nom(qgi1_nom_index), [1; 0.5*dx(thgi1_index)] );

x(pgi2_nom_index) = x_nom(pgi2_nom_index)+dx(pgi2_index);
x(qgi2_nom_index) = qprod( x_nom(qgi2_nom_index), [1; 0.5*dx(thgi2_index)] );

end

