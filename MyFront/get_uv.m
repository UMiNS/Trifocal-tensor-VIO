function uv = get_uv(matches,locs1,locs2)

[line col] = size(matches);
x1 = locs1(:,1);
y1 = locs1(:,2);
x2 = locs2(:,1);
y2 = locs2(:,2);
 for i=1:line
     if norm(matches(i,:))>0
          j = matches(i,2);
          uv(i,1:4) =[i j x2(j)-x1(i) y2(j)-y1(i) ];
          %fprintf('match [features1 %d:img1(%d,%d)]->[features2 %d:img2(%d,%d)],uv=(%d,%d)\n',i,x1(i),y1(i),j,x2(j),y2(j),uv(i,3),uv(i,4));
          
     else
          ;%fprintf('match features1 %d:none matched \n',i);
          uv(i,1:4) =[i 0 0 0 ];
     end
 end
end