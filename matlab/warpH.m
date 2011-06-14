function wimg = warpH(img, roi, H)
x1=roi(1); x2=roi(3);
y1=roi(2); y2=roi(4);
M=y2-y1+1; %rows
N=x2-x1+1; %cols

% Get all points in destination to sample
[xg yg] = meshgrid(x1:x2, y1:y2);
xy = double([reshape(xg, numel(xg), 1)'; reshape(yg, numel(yg), 1)']);
xy = [xy; ones(1,length(xy))];

% Transform into source
uv = H * xy;
% Divide for homography
uv = uv ./ repmat(uv(3,:),3,1);
% Remove homogeneous
uv = uv(1:2,:)';

% Sample
xi = reshape(uv(:,1),M,N);
yi = reshape(uv(:,2),M,N);
wimg = interp2(double(img), xi, yi, 'linear');

% Check for NaN background pixels - replace them with a background of 0
idx = find(isnan(wimg));
if ~isempty(idx)
	wimg(idx) = 0;
end

end