function T0 = getT0( oxts )

scale = latToScale( oxts(1) );
[ t(1,1), t(2,1) ] = latlonToMercator( oxts(1), oxts(2), scale );
t(3,1) = oxts(3);

rx = oxts(4); % roll
ry = oxts(5); % pitch
rz = oxts(6); % heading
Rx = [1 0 0; 0 cos(rx) -sin(rx); 0 sin(rx) cos(rx)]; % body frame => navigation frame
Ry = [cos(ry) 0 sin(ry); 0 1 0; -sin(ry) 0 cos(ry)]; % body frame => navigation frame
Rz = [cos(rz) -sin(rz) 0; sin(rz) cos(rz) 0; 0 0 1]; % body frame => navigation frame
R  = Rz*Ry*Rx;
T0 = [R t;0 0 0 1];

end

