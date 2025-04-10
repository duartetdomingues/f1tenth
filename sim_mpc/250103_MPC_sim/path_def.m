function pathArray= path_def(xyArray)
%
% Given an xy trajectory, calculates aditional parameters required
% to complete pathArray
%
% input: xyArray= [x1 x2 ... xn; y1 y2 ... yn]
%
% output:
%  pathArray column = [x y theta distOnPath curvature pathNormalX pathNormalY]'
%  each column of pathArray is a path point
%

% 22/1/99, 3/10/99 (extra pt added at end of xyArray),
% 31/3/99 (unbiased x y derivatives), J.Gaspar

if nargin<1,
 error('missing input argument (xyArray)');
end
if size(xyArray,2)<=2,
 error('xyArray transposed? (to few points, i.e. columns)')
end

% augment path with extra point for easier termination condition
%
i= size(xyArray,2);
for j=(i-1):-1:1,
   if norm(xyArray(:,i)-xyArray(:,j)) > eps,  % found different point ?
      v= xyArray(:,i)-xyArray(:,j);
      newPt= xyArray(:,i) +5*v/norm(v);
      xyArray= [xyArray newPt];
      break;
   end
end

% define path derivatives
%
xDiff= diff(xyArray(1,:));
yDiff= diff(xyArray(2,:));
xyDerivMod= sqrt(xDiff.*xDiff + yDiff.*yDiff);
 %
 % unbiased x y derivatives
 %
n= size(xyArray,2);
x= xyArray(1,[1 1:n n]);
y= xyArray(2,[1 1:n n]);
xDiff_= conv(x,[1 0 -1]); xDiff_= xDiff_(3:(n+2));
yDiff_= conv(y,[1 0 -1]); yDiff_= yDiff_(3:(n+2));
xyDerivMod_= sqrt(xDiff_.*xDiff_ + yDiff_.*yDiff_);

% define robot orientation relative to word x-axis (theta)
%
theta= unwrap(atan2(yDiff_, xDiff_));
for i=2:length(theta),
 if xyDerivMod < eps,  % repeated points in the path ...
  theta(i)= theta(i-1);
 end
end

% define distOnPath (i.e. s)
%
s= xyDerivMod;
for i=2:length(s),
 s(i)= s(i-1)+s(i);
end
s= [0 s];

% define curvature (d theta / d s)
%
thetaDiff= diff(theta);
sDiff= diff(s);
for i=2:length(sDiff),
 if abs(sDiff(i)) < eps,  % repeated points in the path ...
  sDiff(i)= sDiff(i-1);
  thetaDiff(i)= thetaDiff(i-1);
 end
end
curv= thetaDiff./sDiff; curv= [curv curv(length(curv))];
if 1,
 % filter curv
 %
l= length(curv);
curv= conv(curv,ones(1,50)/50);
% curv= conv(curv,ones(1,20)/20);
% curv= conv(curv,ones(1,1)/1);
l2= round((length(curv)-l)/2);
curv= curv(l2:(l2+l-1));
else
   % i= find(abs(curv)>1/200);  % minimum allowed radius is 200 (so max curv is 1/200)
   % curv(i)= sign(curv(i))*1/200;
end

% define local normal to path
%
for i=2:length(xyDerivMod_),
 if xyDerivMod_(i) < eps,
  xDiff_(i)= xDiff_(i-1);
  yDiff_(i)= yDiff_(i-1);
  xyDerivMod_(i)= xyDerivMod_(i-1);
 end
end
normalXY= [-yDiff_./xyDerivMod_; xDiff_./xyDerivMod_];

% ... join everything and return ...
%
pathArray= [xyArray; theta; s; curv; normalXY];
