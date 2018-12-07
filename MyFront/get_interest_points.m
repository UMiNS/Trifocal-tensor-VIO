
function [x, y,num_cnr] = get_interest_points(image,feature_width)

 

feature_width = 16;

fx = [-1,0,1;-2,0,2;-1,0,1];

fy = [-1,-2,-1;0,0,0;1,2,1];

Ix = imfilter(image,fx);

Iy = imfilter(image,fy);

Ix2 = Ix.* Ix;

%subplot(2,2,1);

%imshow(Ix2);

Ixy = Ix.* Iy;

% subplot(2,2,2);

% imshow(Ixy);

Iy2 = Iy.* Iy;

% subplot(2,2,3);

% imshow(Iy2);

h = fspecial('gaussian',25,2);

% subplot(2,2,4);

% imshow(h);

Ix2 = imfilter(Ix2,h);

Ixy = imfilter(Ixy,h);

Iy2 = imfilter(Iy2,h);

 

%寻找最大R值

Rmax = 0;

l = 0.05;

[img_height,img_width] = size(image);

M = zeros(img_height,img_width);

for i = 1:img_height

    for j = 1:img_width

        A = [Ix2(i,j),Ixy(i,j);Ixy(i,j),Iy2(i,j)];

        M(i,j) = det(A) -  l*(trace(A))^2;

        if M(i,j) > Rmax

            Rmax = M(i,j);

        end

    end

end

 

 

%局部非极大值抑制

k = 0.06;

cnt = 0;   %记录角点数目

result = zeros(img_height,img_width);

for i=2:img_height-1

    for j =2:img_width-1

        if M(i,j)>k*Rmax && M(i,j)>M(i-1,j-1) && M(i,j)>M(i-1,j)&&M(i,j)>M(i-1,j+1)&&M(i,j)>M(i,j-1)&&M(i,j)>M(i,j+1)&&M(i,j)>M(i+1,j-1)&&M(i,j)>M(i+1,j)&&M(i,j)>M(i+1,j+1)

            result(i,j) = 1;

            cnt = cnt + 1;

        end

    end

end

 

[y,x] = find(result==1);

%x_indexleft = find(x <= feature_width/2 - 1);   %寻找x方向上左边边缘角点
x_indexleft = find(x <= feature_width/2);   %寻找x方向上左边边缘角点

%去除边缘角点

x(x_indexleft) = [];

y(x_indexleft) = [];

%y_indexup = find(y <= feature_width/2 - 1);    %寻找y方向上部边缘角点
y_indexup = find(y <= feature_width/2);    %寻找y方向上部边缘角点

x(y_indexup) = [];

y(y_indexup) = [];

%x_indexright = find(x > img_width - 8);       %寻找x方向右边的边缘角点
x_indexright = find(x >= img_width - feature_width/2);       %寻找x方向右边的边缘角点

x(x_indexright) = [];

y(x_indexright) = [];

%y_indexdown = find(y > img_height - 8);       %寻找y方向下部的边缘角点
y_indexdown = find(y >= img_height - feature_width/2);       %寻找y方向下部的边缘角点

x(y_indexdown) = [];

y(y_indexdown) = [];

%x(find(x<feature_width/2 - 1)) = [];

%y(find(y<feature_width/2 - 1)) = [];

num_cnr = length(x);

if 0
    disp(length(x));    %显示角点数目
figure;

%imshow(image);
 imshow(uint8(image));
 
hold on;

plot(x,y,'ro');

hold off;
end
end
