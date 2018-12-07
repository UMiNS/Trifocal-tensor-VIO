function [ dx, P ,phi] = error_state_prediction( dx_init, P_init, Q, x_nom, IMUData,phi_1 )
% error state prediction

% dx = [   pgi',   thgi',   vgi',   ba',   bg', ... 
%          1~3     4~6      7~9    10~12  13~15 
%          pgi1,   thgi1,   pgi2,   thgi2   ];
%         16~18   19~21    22~24   25~27
pgi_index = 1:3; thgi_index = 4:6; vgi_index = 7:9;
ba_index = 10:12; bg_index = 13:15;
pgi1_index = 16:18; thgi1_index = 19:21;
pgi2_index = 22:24; thgi2_index = 25:27;

% nomial state 
% x = [   pgi',   qgi',   vgi',   ba',   bg', ... 
%         1~3     4~7    8~10   11~13  14~16 
%         pgi1,   qgi1,   pgi2,   qgi2   ];
%        17~19   20~23   24~26   27~30    
pgi_nom_index = 1:3; qgi_nom_index = 4:7; vgi_nom_index = 8:10;
ba_nom_index = 11:13; bg_nom_index = 14:16;
pgi1_nom_index = 17:19; qgi1_nom_index = 20:23;
pgi2_nom_index = 24:26; qgi2_nom_index = 27:30;

global Tic idt;

Ric = Tic(1:3,1:3);
pic = Tic(1:3,4);
omxDt = IMUData(1:3);
axDt = IMUData(4:6);
Rgi = q2r( x_nom(qgi_nom_index) );

O3 = zeros( 3 );
I3 = eye( 3 );
nrv = norm(phi_1);
if nrv == 0
    kphi = 1/12.0;
else
    kphi = (1- nrv*sin(nrv)/2/(1-cos(nrv)))/nrv/nrv;
end

  
wxSkew = skew( omxDt-x_nom(bg_nom_index) );
axSkew = skew( axDt-x_nom(ba_nom_index) ); 

phi = phi_1;
A = Rgi*axSkew*(-1/2*idt^2*I3+1/6*wxSkew*idt^3-1/24*wxSkew^2*idt^4);
B = I3-idt*wxSkew+1/2*idt^2*wxSkew^2;
C = Rgi*axSkew*(-idt*I3+1/2*idt^2*wxSkew-1/6*idt^3*wxSkew^2);
D = Rgi*axSkew*(1/6*idt^3*I3-1/24*wxSkew*idt^4+1/120*wxSkew^2*idt^5);
E = -idt*I3+1/2*idt^2*wxSkew-1/6*idt^3*wxSkew^2;
F = -A;

Fd = [ I3,  A, idt*I3, -Rgi*0.5*idt^2, D, O3, O3, O3, O3;
       O3,  B,    O3,            O3,  E, O3, O3, O3, O3;
       O3,  C,    I3,       -Rgi*idt,  F, O3, O3, O3, O3;
       O3, O3,    O3,            I3,  O3, O3, O3, O3, O3;
       O3, O3,    O3,            O3,  I3, O3, O3, O3, O3;
       O3, O3,    O3,            O3,  O3, I3, O3, O3, O3;
       O3, O3,    O3,            O3,  O3, O3, I3, O3, O3;
       O3, O3,    O3,            O3,  O3, O3, O3, I3, O3;
       O3, O3,    O3,            O3,  O3, O3, O3, O3, I3; ];
   
ng_var  = Q(1,1); 
na_var  = Q(4,4);
nba_var = Q(7,7); 
nbg_var = Q(10,10);

Qd11 = ng_var*Rgi*axSkew*(-1/20*idt^5*I3+1/252*wxSkew^2*idt^7)*axSkew*Rgi'...
       +1/3*na_var*idt^3*I3+1/20*nba_var*idt^5*I3...
       -nbg_var*Rgi*axSkew*(1/252*idt^7*I3+1/8640*wxSkew^2*idt^9)*axSkew*Rgi';

Qd12 = ng_var*Rgi*axSkew*(-1/6*idt^3*I3-1/12*wxSkew*idt^4-1/40*wxSkew^2*idt^5)...
       +nbg_var*Rgi*axSkew*(-1/30*idt^5*I3-1/144*wxSkew*idt^6-11/5040*wxSkew^2*idt^7);
   
Qd13 = -ng_var*Rgi*axSkew*(1/8*idt^4*I3+1/60*wxSkew*idt^5+1/144*wxSkew^2*idt^6)*axSkew*Rgi'...
       +1/2*na_var*idt^2*I3+1/8*nba_var*idt^4*I3...
       +nbg_var*Rgi*axSkew*(-1/72*idt^6*I3-1/1008*wxSkew*idt^7-1/1920*wxSkew^2*idt^8)*axSkew*Rgi';

Qd14 = -1/6*nba_var*Rgi*idt^3;
   
Qd15 = nbg_var*Rgi*axSkew*(1/24*idt^4*I3-1/120*wxSkew*idt^5+1/720*wxSkew^2*idt^6);

Qd21 = ng_var*(1/6*idt^3*I3-1/12*wxSkew*idt^4+1/40*wxSkew^2*idt^5)*axSkew*Rgi'...
       +nbg_var*(1/30*idt^5*I3-1/144*wxSkew*idt^6+11/5040*wxSkew^2*idt^7)*axSkew*Rgi';
   
Qd22 = ng_var*idt*I3+nbg_var*(1/3*idt^3*I3+1/60*wxSkew^2*idt^5);

Qd23 = ng_var*(1/2*idt^2*I3-1/6*wxSkew*idt^3+1/24*wxSkew^2*idt^4)*axSkew*Rgi'...
       +nbg_var*(1/8*idt^4*I3-1/60*wxSkew*idt^5+1/144*wxSkew^2*idt^6)*axSkew*Rgi';
   
Qd25 = nbg_var*(-1/2*idt^2*I3+1/6*wxSkew*idt^3-1/24*wxSkew^2*idt^4);   

Qd31 = Qd13';

Qd32 = Qd23';

Qd33 = -ng_var*Rgi*axSkew*(1/3*idt^3*I3+1/60*wxSkew^2*idt^5)*axSkew*Rgi'...
       +na_var*idt*I3+1/3*nba_var*idt^3*I3...
       -nbg_var*Rgi*axSkew*(1/20*idt^5*I3+1/504*wxSkew^2*idt^7)*axSkew*Rgi';

Qd34 = -1/2*nba_var*Rgi*idt^2;

Qd35 = nbg_var*Rgi*axSkew*(1/6*idt^3*I3-1/24*wxSkew*idt^4+1/120*wxSkew^2*idt^5);

Qd41 = Qd14';

Qd43 = Qd34';

Qd44 = nba_var*idt*I3;

Qd51 = Qd15';

Qd52 = Qd25';

Qd53 = Qd35';

Qd55 = nbg_var*idt*I3;

Qd = [ Qd11, Qd12, Qd13, Qd14, Qd15, O3, O3, O3, O3;...
       Qd21, Qd22, Qd23,   O3, Qd25, O3, O3, O3, O3;...
       Qd31, Qd32, Qd33, Qd34, Qd35, O3, O3, O3, O3;...
       Qd41,   O3, Qd43, Qd44,   O3, O3, O3, O3, O3;...
       Qd51, Qd52, Qd53,   O3, Qd55, O3, O3, O3, O3;...
         O3,   O3,   O3,   O3,   O3, O3, O3, O3, O3;...
         O3,   O3,   O3,   O3,   O3, O3, O3, O3, O3;...
         O3,   O3,   O3,   O3,   O3, O3, O3, O3, O3;...
         O3,   O3,   O3,   O3,   O3, O3, O3, O3, O3; ];
  
dx = Fd*dx_init;
P = Fd*P_init*Fd'+Qd;

end

