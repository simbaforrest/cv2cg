function [R,T,N] = decomposeH(H,K)
%INPUT
%   H<3x3>: homography computed from two image point sets
%   K<3x3>: calibration matrix
%OUTPUT
%   R<2 cells,3x3>: two rotation matrix solutions
%   T<2 cells,3x1>: two tranlation vector solutions
%   N<2 cells,3x1>: two plane normal solutions
%REFERENCE
%   An Invitation to 3-D Vision, Yi Ma, et. al. , p137-p139, Theorem 5.19
%AUTHOR
%   Chen Feng <simbaforrest@gmail.com>
H = inv(K) * H * K;
H = normalizeH(H);

A = H'*H;
[V,S,~] = svd(A);
if det(V) == -1
    V = -V;
end

s1 = S(1,1);
s2 = S(2,2);
s3 = S(3,3);

v1 = V(:,1);
v2 = V(:,2);
v3 = V(:,3);

u1 =( sqrt(1-s3)*v1 + sqrt(s1-1)*v3 )/sqrt(s1-s3);
u2 =( sqrt(1-s3)*v1 - sqrt(s1-1)*v3 )/sqrt(s1-s3);

U1 = [v2 u1 skew(v2)*u1];
W1 = [H*v2 H*u1 skew(H*v2)*H*u1];
U2 = [v2 u2 skew(v2)*u2];
W2 = [H*v2 H*u2 skew(H*v2)*H*u2];

R = cell(2,1);
T = cell(2,1);
N = cell(2,1);

% solution 1
R{1} = W1*U1';
N{1} = skew(v2)*u1;
T{1} = (H-R{1})*N{1};
if N{1}(3)<0 %impose N'*e3 = n3 >0, i.e. positive depth constraint
    R{1} = R{1};
    N{1} = -N{1};
    T{1} = -T{1};
end

% solution 2
R{2} = W2*U2';
N{2} = skew(v2)*u2;
T{2} = (H-R{2})*N{2};
if N{2}(3)<0 %impose N'*e3 = n3 >0, i.e. positive depth constraint
    R{2} = R{2};
    N{2} = -N{2};
    T{2} = -T{2};
end
end