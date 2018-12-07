function z = updateModel( x, uv123 )
% measurement model

% x = [   pgi',   qgi',   vgi',   ba',   bg', ... 
%         1~3     4~7    8~10   11~13  14~16 
%         pgi1,   qgi1,   pgi2,   qgi2   ];
%        17~19   20~23   24~26   27~30    

global K;
global Tic;

invK = inv(K);

pgi_index = 1:3; qgi_index = 4:7; vgi_index = 8:10;
ba_index = 11:13; bg_index = 14:16;
pgi1_index = 17:19; qgi1_index = 20:23;
pgi2_index = 24:26; qgi2_index = 27:30;

Ric = Tic(1:3,1:3);
pic = Tic(1:3,4);

pgc1 = x(pgi1_index)+q2r(x(qgi1_index))*pic;
Rgc1 = q2r(x(qgi1_index))*Ric;
pgc2 = x(pgi2_index)+q2r(x(qgi2_index))*pic;
Rgc2 = q2r(x(qgi2_index))*Ric;
pgc3 = x(pgi_index)+q2r(x(qgi_index))*pic;
Rgc3 = q2r(x(qgi_index))*Ric;

R12 = Rgc1'*Rgc2;
t12 = Rgc1'*(pgc2-pgc1);
R13 = Rgc1'*Rgc3;
t13 = Rgc1'*(pgc3-pgc1);
R23 = Rgc2'*Rgc3;
t23 = Rgc2'*(pgc3-pgc2);

PA = [ eye(3), zeros( 3, 1 ) ];
PB = [ R12', -R12'*t12 ];
PC = [ R13', -R13'*t13 ];

F12 = R12'*skew( t12 );
F23 = R23'*skew( t23 );

z = zeros( 4, size( uv123, 2 ) );
for m = 1:size( uv123, 2 )
    uv1_normal = invK*[ uv123(:,m,1); 1 ]; uv1_normal = uv1_normal/uv1_normal(3);
    uv2_normal = invK*[ uv123(:,m,2); 1 ]; uv2_normal = uv2_normal/uv2_normal(3);
    uv3_normal = invK*[ uv123(:,m,3); 1 ]; uv3_normal = uv3_normal/uv3_normal(3);

    uv3_est = zeros( 3, 1 ); % use trifocal tensor to estimate
    epLine = F12*uv1_normal;
    epLineNormal = [                          epLine(2);
                                             -epLine(1);
                     -uv2_normal(1)*epLine(2)+uv2_normal(2)*epLine(1); ];
    for k = 1:3
        for i = 1:3
            for j = 1:3
                Tijk = PB(j,i)*PC(k,4)-PB(j,4)*PC(k,i);
                uv3_est(k) = uv3_est(k)+uv1_normal(i)*epLineNormal(j)*Tijk;
            end
        end
    end
    
    uv3_est = K*uv3_est;
    uv3_est = uv3_est/uv3_est(3);
    
    z(1,m) = uv2_normal'*F12*uv1_normal; % epipolar constraint
    z(2,m) = uv3_normal'*F23*uv2_normal; % epipolar constraint
    z(3:4,m) = uv3_est(1:2);             % trifocal tensor
end

z = z(:);

end



