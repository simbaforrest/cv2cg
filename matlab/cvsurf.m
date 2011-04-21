% [keys,matches]=cvsurf(imnames)
% INPUT
%   imnames <cell, string, Kx1>: file paths for each image
% OUTPUT
%   [keys <cell, Kx1>]: optional, detected keypoints for each image, <Nx3>
%   [matches <cell, KxK>]: optional, matched idx, <Mx2>
%       matches{i,j}(k,:)=[m,n] means image i is training image, image j is
%       query image, and keys{i}(m,:) is matched with keys{j}(n,:)
%       normally, n will be equal to k, since n stores the trainIdx
% AUTHOR
%   Chen Feng <simbaforrest@gmail.com>
