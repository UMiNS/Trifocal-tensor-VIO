function lmk = createLmk( locs )
% locs : lanmark location (row, column, scale, orientation).
num_lmk = size( locs, 1 );

lmk = cell( 1, num_lmk );
for k = 1:num_lmk
    vis = zeros( 1, 3 );
    uv = zeros( 2, 3 );
    
  
    vis(1) = 1;
    %uv(:,1) = [ locs(k,4); locs(k,3); ];
    
    lmk{k} = struct(    'num',      k,...
                    'matchID',      k,...
                        'vis',    vis,...
                         'uv',     uv );
end

end



