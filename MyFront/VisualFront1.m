frame1 = imread('0.png');
frame2 = imread('1.png');
frame3 = imread('2.png');
I1 =double(frame1);
I2 =double(frame2);
ws =16;

 [x1, y1,num1] = get_interest_points(I1,ws);
 [x2, y2,num2] = get_interest_points(I2,ws);

 
 features1 = get_features(I1, x1, y1, ws);
 features2 = get_features(I2, x2, y2, ws);
  
 [matches, confidences] = match_features(features1, features2);
 
 
 
[line col] = size(matches);
 
 for i=1:line
     if norm(matches(i,:))>0
          j = matches(i,1);
          k = matches(i,2);
          uv(i,1:4) =[j k x2(k)-x1(j) y2(k)-y1(j) ];
          fprintf('match [features1 %d:img1(%d,%d)]->[features2 %d:img2(%d,%d)],uv=(%d,%d)\n',j,x1(j),y1(j),k,x2(k),y2(k),uv(i,3),uv(i,4));
          
     else
          fprintf('match features1 %d:none matched \n',i);
     end
 end
 
locs = [x1 y1];
[U, V] = LK1(I1, I2, ws, locs);
% show optical flow and save it to a file
figure; showOF(I1, locs, U, V);