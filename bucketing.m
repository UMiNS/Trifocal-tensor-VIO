function num_of_feature = bucketing( lmk, commonLmkList, max_feature, bucketSize, imSize )
% use "bucketing" concept to choose a subset of features
bucket = cell( bucketSize(1), bucketSize(2) );
bucketCnt = zeros( bucketSize(1), bucketSize(2) );
for k = 1:size( commonLmkList, 2 )
    bucketID(1) = floor( lmk{commonLmkList(k)}.uv(2,1)/(imSize(2)/bucketSize(1)) )+1;
    bucketID(2) = floor( lmk{commonLmkList(k)}.uv(1,1)/(imSize(1)/bucketSize(2)) )+1;
    bucket{bucketID(1),bucketID(2)} = [ bucket{bucketID(1),bucketID(2)}, commonLmkList(k) ];
    bucketCnt(bucketID(1),bucketID(2)) = bucketCnt(bucketID(1),bucketID(2))+1;
end

bucketCnt_ini = bucketCnt;
for k = 1:max_feature
    bucketWeight = bucketCnt/sum( bucketCnt(:) );
    for i = 1:bucketSize(1)
        for j = 1:bucketSize(2)
            if bucketWeight(i,j) ~= 0
                bucketWeight(i,j) = bucketWeight(i,j)+100; % make choose fairly
                bucketWeight(i,j) = 1/bucketWeight(i,j);   
            end
        end
    end
    
    bucketWeight = bucketWeight/sum( bucketWeight(:) );
    bucketRan = randsample( 0:bucketSize(1)*bucketSize(2)-1, 1, true, bucketWeight(:) );
    bucketID(1) = mod( bucketRan, bucketSize(1) )+1;
    bucketID(2) = floor( bucketRan/bucketSize(1) )+1;
    bucketCnt(bucketID(1),bucketID(2)) = bucketCnt(bucketID(1),bucketID(2))-1;
end

num_of_feature = [];
bucketCnt = bucketCnt_ini-bucketCnt;
for i = 1:bucketSize(1)
    for j = 1:bucketSize(2)
        if bucketCnt(i,j) > 0
            randNum = randsample( 1:size( bucket{i,j}, 2), bucketCnt(i,j) );
            num_of_feature = [ num_of_feature, bucket{i,j}(randNum) ];
        end
    end
end

num_of_feature = sort( num_of_feature );

end

