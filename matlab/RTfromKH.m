function [R,T]=RTfromKH(K,H)
% compute plane pose (R and T) from K and Homography
% by Chen Feng <simbaforrest@gmail.com>

% H = const * K[r1,r2,T]
if det(H)<0
	H = -H;
end
A = inv(K) * H;
A = A/norm(A(:,1)); % get rid of const by apply ||r1||=1
r1 = A(:,1);
r2 = A(:,2);
T  = A(:,3);
r3 = cross(r1,r2);
R = [r1,r2,r3];
end
