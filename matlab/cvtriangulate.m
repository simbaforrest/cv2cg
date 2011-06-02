function X = cvtriangulate(p1,p2,P1,P2)
x1 = p1(1); y1 = p1(2);
x2 = p2(1); y2 = p2(2);
A = [x1*P1(3,:) - P1(1,:);
     y1*P1(3,:) - P1(2,:);
     x2*P2(3,:) - P2(1,:);
     y2*P2(3,:) - P2(2,:)];
[~,~,X]=svd(A,0);
X = X(:,end);
X = X/X(end);
end