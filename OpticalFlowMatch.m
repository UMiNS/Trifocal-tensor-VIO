% num = match(image1, image2)
%
% This function reads two images, finds their features, and
%   displays lines connecting the matched keypoints.  A match is accepted
%   only if its distance is less than distRatio times the distance to the
%   second closest match.
% It returns the number of matches displayed.
%
% Example: match('scene.pgm','book.pgm');

function uv = OpticalFlowMatch(PIP,b1,A1,im2)

   Size_PI=size(PIP,1);
   b2 = zeros(Size_PI,1);
   for r=1: Size_PI
        %A(r,1:2)   = [Ix(PIP(r,1),PIP(r,2)) Iy(PIP(r,1),PIP(r,2))];

        b2(r) = im2(PIP(r,1),PIP(r,2));
   end   

    
    AtA = A1'*A1;
    b = b2-b1;
  
    uv = AtA^(-1)*(A1'*b);
    uv
    %%%%%%%%%%%%%%

fprintf('Found %d matches.\n', line);




