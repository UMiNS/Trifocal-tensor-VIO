close all; 
clear all; 
format long e;
global K;                   % global variables for camera params
global idt gg Tic;          % global variables for IMU params

%Modified by xiphix(xiphix@126.com)
%replace SIFT feature detector by Harris corner detector¡¢
%SIFT-like descritor¡¢NNDR match¡¢simple optic flow uv
%compare orignal version using preprocessed SIFT feature,
%this modified version can run online without preprocessed data.
%notes:some mistakes  inside,maybe

idt       = 1/10;           % IMU    @10Hz
cdt       = 1/10;           % Camera @10Hz
sampleRatio = cdt/idt;

bucketSize  = [ 4, 8 ];     % [ row, col ]
max_feature = 9;

% is new data?
% 1 : yes, use SIFT to extract and match features
% 0 : no, load .mat file to get the pre-possessing of features 
newData = 1;                

% choose the experiment case
% 1 : KITTI case 1, 2 : KITTI case 2, 3 : KITTI case 3
expCase = 2;   

setupParams;

imDir = [ BaseDir 'image_00\data\' ];

    camCalib = loadCalibrationCamToCam( [ calibDir 'calib_cam_to_cam.txt' ] );
    IMU_to_vel_Calib = loadCalibrationRigid( [ calibDir 'calib_imu_to_velo.txt' ] );
    vel_to_cam_Calib = loadCalibrationRigid( [ calibDir 'calib_velo_to_cam.txt' ] );
    oxtsTab = loadOxtsliteData( BaseDir, imInit+1:imInit+M );
    pose = convertOxtsToPose( oxtsTab );


imSize = camCalib.S_rect{1};
K = camCalib.P_rect{1}*[ camCalib.R_rect{1} zeros(3,1); zeros(1,3) 1 ];
K = K(1:3,1:3);

IMU_to_cam_Calib = vel_to_cam_Calib*IMU_to_vel_Calib;
Rci = IMU_to_cam_Calib(1:3,1:3);
pci = IMU_to_cam_Calib(1:3,4);

Ric = Rci';
pic = -Ric * pci;

%% load ground truth data
RgiTab = zeros( 3, 3, M );
pgiTab = zeros( 3, M );

RgcTab = zeros( 3, 3, M );
pgcTab = zeros( 3, M );
% figure; hold on; axis equal;
% L = 1; % coordinate axis length
% A = [0 0 0 L; L 0 0 L; 0 0 0 L; 0 L 0 L; 0 0 0 L; 0 0 L L]';
for k = 1:1
    RgiTab(:,:,k) = pose{k}(1:3,1:3);
    pgiTab(:,k) = pose{k}(1:3,4);
    RgcTab(:,:,k) = RgiTab(:,:,k)*Ric;
    pgcTab(:,k) = pgiTab(:,k)+RgiTab(:,:,k)*pic;
    
%     T = [ RgiTab(:,:,k), pgiTab(:,k); 
%              zeros(1,3),           1; ];
%     B = T*A;
%     plot3(B(1,1:2),B(2,1:2),B(3,1:2),'-r','LineWidth',1); % x: red
%     plot3(B(1,3:4),B(2,3:4),B(3,3:4),'-g','LineWidth',1); % y: green
%     plot3(B(1,5:6),B(2,5:6),B(3,5:6),'-b','LineWidth',1); % z: blue
%     plot3( pgiTab(1,k), pgiTab(2,k), pgiTab(3,k), '.k', 'Markersize', 10 );
% 
%     T = [ RgcTab(:,:,k), pgcTab(:,k); 
%              zeros(1,3),           1; ];
%     B = T*A;
%     plot3(B(1,1:2),B(2,1:2),B(3,1:2),'-r','LineWidth',1); % x: red
%     plot3(B(1,3:4),B(2,3:4),B(3,3:4),'-g','LineWidth',1); % y: green
%     plot3(B(1,5:6),B(2,5:6),B(3,5:6),'-b','LineWidth',1); % z: blue
%     plot3( pgcTab(1,k), pgcTab(2,k), pgcTab(3,k), '.b', 'Markersize', 10 );
end
% xlabel('x'); ylabel('y'); zlabel('z'); 

%% load IMU Data
vel_index = 9:11; % FLU frame
am_index = 12:14; % 12:14 body frame, 15:17 FLU frame
wm_index = 18:20; % 18:20 body frame, 21:23 FLU frame

IMUTab = zeros(6,M);
for k = 1:M
    IMUTab(1:3,k) = oxtsTab{k}(wm_index);
    IMUTab(4:6,k) = oxtsTab{k}(am_index);
end

% figure();
% subplot( 3, 1, 1 ); plot( 1:M, IMUTab(1,:) ); title( 'x gyro' );
% subplot( 3, 1, 2 ); plot( 1:M, IMUTab(2,:) ); title( 'y gyro' ); 
% subplot( 3, 1, 3 ); plot( 1:M, IMUTab(3,:) ); title( 'z gyro' ); 
% figure();
% subplot( 3, 1, 1 ); plot( 1:M, IMUTab(4,:) ); title( 'x acc' ); 
% subplot( 3, 1, 2 ); plot( 1:M, IMUTab(5,:) ); title( 'y acc' ); 
% subplot( 3, 1, 3 ); plot( 1:M, IMUTab(6,:) ); title( 'z acc' ); 

%%  only use IMU to calculate pose
% nomial state 
% x = [   pgi',   qgi',   vgi',   ba',   bg', ... 
%         1~3     4~7    8~10   11~13  14~16 
%         pgi1,   qgi1,   pgi2,   qgi2   ];
%        17~19   20~23   24~26   27~30    
pgi_nom_index = 1:3; qgi_nom_index = 4:7; vgi_nom_index = 8:10;
ba_nom_index = 11:13; bg_nom_index = 14:16;
pgi1_nom_index = 17:19; qgi1_nom_index = 20:23;
pgi2_nom_index = 24:26; qgi2_nom_index = 27:30;

% error state
% dx = [   pgi',   thgi',   vgi',   ba',   bg', ... 
%          1~3     4~6      7~9    10~12  13~15 
%          pgi1,   thgi1,   pgi2,   thgi2   ];
%         16~18   19~21    22~24   25~27
pgi_index = 1:3; thgi_index = 4:6; vgi_index = 7:9;
ba_index = 10:12; bg_index = 13:15;
pgi1_index = 16:18; thgi1_index = 19:21;
pgi2_index = 22:24; thgi2_index = 25:27;

gg = getRnb(oxtsTab{1})'*[ 0; 0; 9.81; ]; % gravity in body frame
Tic = [ Ric, pic ];

% nominal state
x_nom = zeros( 30, 1 );
x_nom(vgi_nom_index) = getRnb(oxtsTab{1})'*[ oxtsTab{1}(8); oxtsTab{1}(7); 0; ];
x_nom(qgi_nom_index) = [ 1; 0; 0; 0; ];

pgiIMURec = zeros( 3, M );               
vgiIMURec = zeros( 3, M );
vgiIMURec(:,1) = x_nom(vgi_nom_index);
qgiIMURec = zeros( 4, M );
qgiIMURec(:,1) = x_nom(qgi_nom_index); 
for k = 2:M
    x_nom = nomial_state_prediction( x_nom, IMUTab(:,k-1) );
    pgiIMURec(:,k) = x_nom(pgi_nom_index);
    vgiIMURec(:,k) = x_nom(vgi_nom_index);
    qgiIMURec(:,k) = x_nom(qgi_nom_index);
end
phi = zeros(3,1);
%% visual inertial odometry
% filter state covariance matrix
P = diag( [	  var_pgi*ones(1,3),...
	         var_thgi*ones(1,3),...
	          var_vgi*ones(1,3),...
	           var_ba*ones(1,3),...
	           var_bg*ones(1,3),...
	         var_pgi1*ones(1,3),...
	        var_thgi1*ones(1,3),...
             var_pgi2*ones(1,3),...
    	    var_thgi2*ones(1,3) ] );

% system noise covariance matrix
Q = diag( [  ng_var*ones( 1, 3 ),...
	         na_var*ones( 1, 3 ),...
	        nba_var*ones( 1, 3 ),...
	        nbg_var*ones( 1, 3 ) ] );  

% nominal state
x_nom = zeros( 30, 1 );
x_nom(vgi_nom_index) = getRnb(oxtsTab{1})'*[ oxtsTab{1}(8); oxtsTab{1}(7); 0; ];
x_nom(qgi_nom_index)  = [ 1; 0; 0; 0; ];
x_nom(qgi1_nom_index) = [ 1; 0; 0; 0; ];
x_nom(qgi2_nom_index) = [ 1; 0; 0; 0; ];

% error state
dx = zeros( 27, 1 );

% nomial state record
pgiRec = []; qgiRec = []; vgiRec = [];
baRec  = []; bgRec  = [];
pgi1Rec = []; qgi1Rec = [];
pgi2Rec = []; qgi2Rec = [];

% error state record
dpgiRec = []; dthgiRec = []; dvgiRec = [];
dbaRec  = []; dbgRec   = [];
dpgi1Rec = []; dthgi1Rec = []; 
dpgi2Rec = []; dthgi2Rec = [];

% error covariance record
std_pgiRec = []; std_thgiRec = []; std_vgiRec = [];
std_baRec  = []; std_bgRec   = [];
std_pgi1Rec = []; std_thgi1Rec = []; 
std_pgi2Rec = []; std_thgi2Rec = [];

if newData
    uvRec  = cell( 1, M );
end
outlierRec = cell( 1, M ); 
for step = 1:M
   %step  
   if newData
       % use SIFT to extract and match features
       if step == 1
           
          % [ im1,  locs1 ] = HarrisF( [ imDir sprintf( '%.10d.png', step+imInit-1 ) ]  );
          frame1 = imread([ imDir sprintf( '%.10d.png', step+imInit-1 ) ] );
          im1 =double(frame1);
          [x1, y1,num1] = get_interest_points(im1,16);
          locs1 = [x1 y1];

       elseif step == 2
         
          %[ im2,  locs2 ] = HarrisF( [ imDir sprintf( '%.10d.png', step+imInit-1 ) ]  );
          % matchTab12 = match( im1, des1, locs1, im2, des2, locs2 );
          frame2 = imread([ imDir sprintf( '%.10d.png', step+imInit-1 ) ] );
          im2 =double(frame2);
          [x2, y2,num2] = get_interest_points(im2,16);
          locs2 = [x2 y2];
          
          features1 = get_features(im1, x1, y1, 16);
          features2 = get_features(im2, x2, y2, 16);
  
          [matchTab12, confidences12] = match_features(features1, features2);
          uv12 = get_uv(matchTab12,locs1,locs2);
           %matchTab12 = OpticalFlowMatch( locs1, locs2);
         
       else
           if step > 3
               im1 = im2;  locs1 = locs2;
               im2 = im3;  locs2 = locs3;
               matchTab12 = matchTab23;
               uv12 = uv23;
               x1 = x2;y1=y2;
               x2 = x3;y2=y3;
           end
           
         
          % [ im3,  locs3 ] = HarrisF( [ imDir sprintf( '%.10d.png', step+imInit-1 ) ]  );
          % matchTab23 = match( im2, des2, locs2, im3, des3, locs3 );
          % matchTab23 = OpticalFlowMatch( locs2, locs3);
          frame3 = imread([ imDir sprintf( '%.10d.png', step+imInit-1 ) ] );
          im3 =double(frame3);
          [x3, y3,num3] = get_interest_points(im3,16);
          locs3 = [x3 y3];
          
          features2 = get_features(im2, x2, y2, 16);
          features3 = get_features(im3, x3, y3, 16);
  
          [matchTab23, confidences23] = match_features(features2, features3);
          uv23 = get_uv(matchTab23,locs2,locs3);
       end
   end
   
   if step > 1
       % filter predict
       for k = 1:sampleRatio
           [ dx, P ,phi] = error_state_prediction( dx, P, Q, x_nom, IMUTab(:,(step-2)*sampleRatio+k) ,phi);
           x_nom     = nomial_state_prediction( x_nom, IMUTab(:,(step-2)*sampleRatio+k) );
       end
       
      % step
       if step >= 3
           if newData
               % use SIFT to extract and match features
               lmk = createLmk( locs1 );
               
               for i = 1:size( lmk, 2 )
                   if lmk{i}.matchID > 0
                       if matchTab12(lmk{i}.matchID,1) > 0
                           lmk{i}.vis(2) = 1;
                           %lmk{i}.uv(:,2) = [ locs2(matchTab12(lmk{i}.matchID),2); locs2(matchTab12(lmk{i}.matchID),1); ];
                           lmk{i}.uv(:,2) = [ uv12(i,4); uv12(i,3); ];
                           lmk{i}.matchID = uv12(i,2);%matchTab12(lmk{i}.matchID,2);
                       else
                           lmk{i}.matchID = 0;
                       end
                   end
               end
               
               [line col] = size(matchTab23);
               if (size( lmk, 2 ) > line)
                   len = line;
               else
                   len = size( lmk, 2 );
               end
               for i = 1:len %size( lmk, 2 )
                   if lmk{i}.matchID > 0
                       if matchTab23(lmk{i}.matchID,1) > 0
                           lmk{i}.vis(3) = 1;
                           %lmk{i}.uv(:,3) = [ locs3(matchTab23(lmk{i}.matchID),2); locs3(matchTab23(lmk{i}.matchID),1); ];
                           lmk{i}.uv(:,3) = [ uv23(i,4); uv23(i,3); ];
                           lmk{i}.matchID = uv23(i,2);%matchTab23(lmk{i}.matchID);
                       else
                           lmk{i}.matchID = 0;
                       end
                   end
               end
               
               commonLmkList = []; % common feature points list
               for k = 1:size( lmk, 2 )
                   if sum( lmk{k}.vis(1:3) ) == 3
                       commonLmkList = [ commonLmkList, lmk{k}.num ];
                   end
               end
               
               % use "bucketing" concept to choose a subset of features
               if size( commonLmkList, 2 ) > max_feature
                   pickedLmkList = bucketing( lmk, commonLmkList, max_feature, bucketSize, imSize );
               else
                   pickedLmkList = commonLmkList;
               end

               uv123 = zeros( 2, size( pickedLmkList, 2 ), 2 );
               z = zeros( 2, size( pickedLmkList, 2 ) );
               for k = 1:size( pickedLmkList, 2 )
                   uv123(:,k,1) = lmk{pickedLmkList(k)}.uv(:,1);
                   uv123(:,k,2) = lmk{pickedLmkList(k)}.uv(:,2);
                   uv123(:,k,3) = lmk{pickedLmkList(k)}.uv(:,3);
               end
                
               uvRec{step} = uv123;
           else
               % get the pre-possessing of features 
               uv123 = uvRec{step+imInit};
           end
           
           % RANSAC outliers rejection
           [ pickedLmkList, outlierList ] = ransacOutlierRejection( dx, P, nc_var, x_nom, @updateModel, uv123, filterParam, RANSAC_threshold );
           uv123 = uv123(:,pickedLmkList,:);
           outlierRec{step} = outlierList;
           
           % filter update
           [ dx, P ] = update( dx, P, nc_var, x_nom, @updateModel, uv123, filterParam );
           x_nom = nom_plus_err( x_nom, dx );
           
           % record data
           recordState;
           
           % reset error state
           dx = zeros( 27, 1 );
           
           % plot estimated trajectory
           figure( 1 );
           subplot( 3, 2, [ 2 4 6 ] ); hold on; axis equal; grid on;
           plot3( pgiTab(1,step-2), pgiTab(2,step-2), pgiTab(3,step-2), '.k', 'Markersize', 5 );                         
           plot3( x_nom(pgi1_nom_index(1)), x_nom(pgi1_nom_index(2)), x_nom(pgi1_nom_index(3)), '.r', 'Markersize', 5 );
           hold off;
           
%            % show last three images
%            for k = 1:3
%                subplot( 3, 2, 2*k-1 );
%                im = imread( [ imDir sprintf( '%.10d.png', (step-k+1)+imInit-1 ) ] );
%                imshow( im ); hold on;
%                for i = 1:size( uvRec{step+imInit}, 2 )
%                    plot( uvRec{step+imInit}(1,i,4-k), uvRec{step+imInit}(2,i,4-k), 'b+', 'Markersize', 8  );
%                end
%                
%                for i = 1:size( outlierRec{step}, 2 )
%                    plot( uvRec{step+imInit}(1,outlierRec{step}(i),4-k), uvRec{step+imInit}(2,outlierRec{step}(i),4-k), 'ro', 'Markersize', 8  );
%                end             
%                hold off;
%            end
                                           
       end
       
       % replace old state by current state and revise covariance matrix 
       [ x_nom, dx, P ] = replace_state_and_revise_covariance( x_nom, dx, P );
    end
end

%% plot result on map
figure( 2 ); hold on;
lat = zeros( 1, M ); 
lon = zeros( 1, M ); 
for k = 1:M
    lat(k) = oxtsTab{k}(1);
    lon(k) = oxtsTab{k}(2);
end

%cd devkit
    scale = latToScale( oxtsTab{1}(1) );
    T0 = getT0( oxtsTab{1} );

    lonIMU = zeros( 1, size( pgiRec, 2 ) );
    latIMU = zeros( 1, size( pgiRec, 2 ) );
    for k = 1:size( pgiRec, 2 )
        pgi = T0*[ pgiIMURec(:,k); 1 ];
        [ lonIMU(k), latIMU(k) ] = mercatorToLatLon( pgi(1), pgi(2), scale );
    end

    lon_Fusion = zeros( 1, size( pgiRec, 2 ) );
    lat_Fusion = zeros( 1, size( pgiRec, 2 ) );
    for k = 1:size( pgiRec, 2 )
        pgi = pgiRec(:,k);
        pgi = T0*[ pgi; 1 ];
        [ lon_Fusion(k), lat_Fusion(k) ] = mercatorToLatLon( pgi(1), pgi(2), scale );
    end
%cd ..

plot( lon, lat , 'k' , 'LineWidth' , 3 );
plot( lonIMU, latIMU , 'b' , 'LineWidth' , 3 );
plot( lon_Fusion, lat_Fusion, 'r', 'LineWidth', 3 );
%plot_google_map( 'maptype', 'roadmap' )
dlon = (lon_Fusion(end)-lon(end))*1852*60;
dlat = (lat_Fusion(end)-lat(end))*1852*60;
fprintf('dlong:%f dlat:%f\n',dlon,dlat);
ddlon= lon_Fusion-lon(1:end-2);
ddlat= lat_Fusion-lat(1:end-2);
fprintf('dlong RMS:%f dlat RMS:%f\n',sqrt(sum(ddlon.^2)/780)*60*1852,sqrt(sum(ddlat.^2)/780)*60*1852);
%% plot nomial state
plot_nomial_state;

%% plot error state
plot_error_state;

%% plot error covariance evaluate
%plot_error_covariance;
