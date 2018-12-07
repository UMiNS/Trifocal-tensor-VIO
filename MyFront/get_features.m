
function [features] = get_features(image, x, y, feature_width)

%y_gradient = imfilter(image,[-1,0,1]');

% calculate gradient scale matrix.
hx = [-1,0,1];
hy = -hx';
x_gradient = imfilter(image,hx);
y_gradient = imfilter(image,hy);

features = zeros(length(x),128);

for k = 1:length(x)                         %window of 16x16 of each feature 

    x_subwindow = x(k)-7:x(k)+8;   

    y_subwindow = y(k)-7:y(k)+8;

    x_subwindow_gradient = x_gradient(y_subwindow,x_subwindow);

    y_subwindow_gradient = y_gradient(y_subwindow,x_subwindow);

    angles = atan2(y_subwindow_gradient,x_subwindow_gradient); 

   %for i = 1:16

       %for j = 1:16

          % if angles(i,j)<0

              % angles(i,j) = angles(i,j) + 2*pi;

           %end

       %end

   %end

   angles(angles<0) = angles(angles<0) + 2*pi;

   %angles_bin = [0 pi/4 pi/2 3*pi/4 pi 5*pi/4 3*pi/2 7*pi/4 2*pi];

   angles_binranges = 0:pi/4:2*pi;

   B = zeros(1,128);

   for i = 1:feature_width/4:feature_width

       for j = 1:feature_width/4:feature_width

          %A = reshape(angles(i:i+3,j:j+3)',1,16);

          subwindow = angles(i:i+3,j:j+3);

          angles_bincounts = histc(subwindow(:),angles_binranges);

          begin = 1 + 32*(floor(i/4)) + 8*(floor(j/4));  %tab the orientation of each 4x4 window

          B(begin:begin+7) = angles_bincounts(1:8);    %get the eight orientation of each window

          %figure;

          %bar(angles_binranges,angles_bincounts,'histc');

       end

   end

   %disp(length(B/norm(B)));

   features(k,:) = B/norm(B);

end

%cut the end

power = 0.8;

features = features.^power;

end














