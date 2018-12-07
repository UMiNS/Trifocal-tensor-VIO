function Pnew = validateCovMatrix( P )
% make sure P is positive definite

VAL = 10^-12;

Pnew = P;
[ T error ] = cholcov( Pnew, 0 );

if error ~= 0
    % the covariance matrix is not positive definite
    [ V D ] = eig( Pnew );

    % assign the eigenvalues that are smaller than eps to small positive value
    for k = 1:size( D, 1 )
        if D(k, k) <= eps
            D(k, k) = VAL;
        end
    end
    
    % recompose the covariance matrix
    Pnew = V*D*V';

    [ T error ] = cholcov( Pnew, 0 );
    if error ~= 0
        disp('the covariance matrix is not positive definite');
    end
end

end