function [F,inliers]=cvfundmat(p1,p2,verbose)
if size(p1,2) ~=2 || size(p2,2) ~=2
    error('[cvfundmat error] input must be Nx2 matrix!');
end
mpts = [p1 p2];
save('mpts.tmp','mpts','-ascii');
[status,result]=system('cvfundmat mpts.tmp F.tmp inliers.tmp');
if status~=0
    disp(result);
    error('[cvfundmat error] failed to calculate fundamental matrix!');
elseif verbose
    disp('[cvfundmat] done!');
end
F = load('F.tmp');
inliers = logical(load('inliers.tmp'));
delete('mpts.tmp','F.tmp','inliers.tmp');
end