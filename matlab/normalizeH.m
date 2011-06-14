function H = normalizeH(Hl)
%INPUT
%   Hl<3x3>: Hl = lambda*(R+T*n'/d), computed from any two point sets
%OUTPUT
%   H<3x3>: lambda = sigma2(Hl), the second sigular value of Hl, so H
%   =R+T*n'/d
%REFERENCE
%   An Invitation to 3-D Vision, Yi Ma, et. al. , p135, Lemma5.18
%AUTHOR
%   Chen Feng <simbaforrest@gmail.com>
[~,S,~] = svd(Hl);
H = Hl/S(2,2);
end