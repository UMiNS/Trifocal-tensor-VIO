% filter parameters
L      = 27;
alpha  = 0.5;
kappa  = 3-L;
beta   = 2;
lambda = alpha^2*(L+kappa)-L;
Ws     = [ lambda/(L+lambda), 1/(2*(L+lambda))*ones(1,2*L) ];
Wc     = [ lambda/(L+lambda)+(1-alpha^2+beta), 1/(2*(L+lambda))*ones(1,2*L) ];
c      = (L+lambda)^0.5;
filterParam = { L, c, Ws, Wc };

% the dataset can be downloaded from:
% http://www.cvlibs.net/datasets/kitti/raw_data.php
% pleas select the [synced data] version
switch( expCase )
    case 1 % KITTI case 1
        BaseDir = 'F:\vslam\VIO\datasets\2011_10_03_drive_0027\';
        calibDir = 'F:\vslam\VIO\datasets\2011_10_03_calib\';
        
        imInit      = 0;   % initial image number, 0-base index
        M           = 782; % number of image
        
        if ~newData
            load uvRec10_03_0027.mat;
        end 
        
        % init state variance
        var_pgi  = 10^-2; var_thgi  = 10^-6; var_vgi  = (10)^0;
        var_ba   = 10^-2; var_bg    = 10^-2;
        var_pgi1 = 10^-2; var_thgi1 = 10^-6;
        var_pgi2 = 10^-2; var_thgi2 = 10^-6;
        % process noise
        na_var   = 2*10^-1;  % acc noise (m/s^2)
        ng_var   = 5*10^-4;	 % gyro noise (rad/s)
        nba_var  = 10^-2;    % acc bias noise (m/s^2)
        nbg_var  = 10^-5;    % gyro bias noise (rad/s)
        % measure noise
        nc_var   = 2*(10)^0; % camera noise (pixel)
        
        % RANSAC threshold
        RANSAC_threshold = 5;
            
    case 2 % KITTI case 2
        BaseDir = 'F:\vslam\VIO\datasets\2011_10_03_drive_0042\';
        calibDir = 'F:\vslam\VIO\datasets\2011_10_03_calib\';
        
        imInit      = 200; % initial image number, 0-base index
        M           = 200; %947 number of image
        
        if ~newData
            load uvRec10_03_0042.mat;
        end
        
        % init state variance
        var_pgi  = 10^-2; var_thgi  = 10^-6; var_vgi  = (10)^-0;
        var_ba   = 10^-4; var_bg    = 10^-4;
        var_pgi1 = 10^-2; var_thgi1 = 10^-6;
        var_pgi2 = 10^-2; var_thgi2 = 10^-6;
        % process noise
        na_var   = 2*10^-1;  % acc noise (m/s^2)
        ng_var   = 5*10^-5;	 % gyro noise (rad/s)
        nba_var  = 10^-10;   % acc bias noise (m/s^2)
        nbg_var  = 10^-10;   % gyro bias noise (rad/s)
        % measure noise
        nc_var   = 2*(10)^0; % camera noise (pixel)
        
        % RANSAC threshold
        RANSAC_threshold = 5;
        
    case 3 % KITTI case 3
        BaseDir = 'F:\vslam\VIO\datasets\2011_09_30_drive_0028\';
        calibDir = 'F:\vslam\VIO\datasets\2011_09_30_calib\';
        
        imInit      = 0; % initial image number, 0-base index
        M           = 4402; % number of image
        
        if ~newData
            load uvRec09_30_0028.mat;
        end
        
        % init state variance
        var_pgi  = 10^-2; var_thgi  = 10^-6; var_vgi  = (10)^-2;
        var_ba   = 10^-2; var_bg    = 10^-2;
        var_pgi1 = 10^-2; var_thgi1 = 10^-6;
        var_pgi2 = 10^-2; var_thgi2 = 10^-6;
        % process noise
        na_var   = 2*10^-1;  % acc noise (m/s^2)
        ng_var   = 5*10^-4;	 % gyro noise (rad/s)
        nba_var  = 10^-5;    % acc bias noise (m/s^2)
        nbg_var  = 10^-5;    % gyro bias noise (rad/s)
        % measure noise
        nc_var   = 2*(10)^0; % camera noise (pixel)
        
        % RANSAC threshold
        RANSAC_threshold = 5;
end