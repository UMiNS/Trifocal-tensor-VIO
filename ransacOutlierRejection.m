function [ pickedLmkList, outlierList ] = ransacOutlierRejection( dx, P, nc_var, x_nom, h, uv123, filterParam, threshold  )
% RANSAC outlier Rejection
p_at_least_one_spurious_free = 0.99; % default value
s = 3;                               % minimum number of data points needed for estimation

n_hyp = 1000;                        % initial number of iterations, will be updated
max_hypothesis_support = 0;

num_matches = size( uv123, 2 );
uv3 = uv123(:,:,3);
uv3 = uv3(:);

k = 1;
while( k <= n_hyp )
    randNum = ceil( rand()*num_matches );
    [ dx_k, P_k ] = update( dx, P, nc_var, x_nom, h, uv123(:,randNum,:), filterParam );
    x_k = nom_plus_err( x_nom, dx_k );

    uv3_est = trifocalTransfer( x_k, uv123 );
    inlierIndex = sqrt( sum( reshape( (uv3_est-uv3).*(uv3_est-uv3), 2, size(uv3,1)/2 ) ) ) < threshold;
    hypothesis_support = sum( inlierIndex );
    
    if hypothesis_support > max_hypothesis_support
        max_hypothesis_support = hypothesis_support;
        pickedLmkList = find( inlierIndex );
        outlierList = find( ~inlierIndex );
        epsilon = 1-(hypothesis_support/num_matches);
        n_hyp = ceil(log(1-p_at_least_one_spurious_free)/log(1-(1-epsilon)^s));
        if n_hyp==0 
            break; 
        end
    end
    
    k = k+1;
end

end

