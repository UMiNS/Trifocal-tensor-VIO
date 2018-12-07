function [ dx, P ] = update( dx_init, P_init, nc_var, x_nom, h, uv123, filterParam )
% state update

% x : state
% P : covariance
% h : measurement function
% hParam : parameters for measurement function
% filterParam : parameters for filter

L  = filterParam{1};
c  = filterParam{2};
Ws = filterParam{3};
Wc = filterParam{4};

z = zeros( 4, size( uv123, 2 ) );
z(3:4,:) = uv123(:,:,3);
z = z(:);
% R = nc_var*eye( 2*size( uv123, 2 ) );
R = diag( repmat( [ 1 1 nc_var nc_var ], 1, size( uv123, 2 ) ) );

P_root = chol( validateCovMatrix( P_init ) )'; % make sure P is positive definite
dX = [ dx_init, dx_init(:,ones(1,L))+c*P_root(:,1:L), dx_init(:,ones(1,L))-c*P_root(:,1:L) ];
Z = zeros( size( z, 1 ), 2*L+1 );
z_hat = zeros( size( z ) );
for k = 1:2*L+1
    x = nom_plus_err( x_nom, dX(:,k) );
    Z(:,k) = h( x, uv123 );
    z_hat = z_hat+Ws(k)*Z(:,k);
end

Pzz = (Z-z_hat*ones( 1, 2*L+1 ))*diag(Wc)*(Z-z_hat*ones( 1, 2*L+1 ))'+R;
Pxz = (dX-dx_init*ones( 1, 2*L+1 ))*diag(Wc)*(Z-z_hat*ones( 1, 2*L+1 ))';
Kgain = Pxz/Pzz;
dx = dx_init+Kgain*(z-z_hat);
P = P_init-Kgain*Pzz*Kgain';

end


