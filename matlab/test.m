clear; close all; clc;
imnames = cell(5,1);
for i=0:4
    imnames{i+1} = ['../data/books00',num2str(i),'.png'];
end
tic;
[keys,matches]=cvsurf(imnames);
toc

for i=1:5
    for j=(i+1):5
        disp(['img: ',num2str(i),'->',num2str(j)]);
        mat12 = zeros(length(keys{i}),length(keys{j}));
        idx = sub2ind(size(mat12),matches{i,j}(:,1)+1,matches{i,j}(:,2)+1);
        mat12(idx)=1;
        mat21 = zeros(length(keys{j}),length(keys{i}));
        idx = sub2ind(size(mat21),matches{j,i}(:,1)+1,matches{j,i}(:,2)+1);
        mat21(idx)=1;
        subplot(1,3,1)
        spy(mat12,5)
        subplot(1,3,2)
        spy(mat21',5)
        subplot(1,3,3)
        spy(mat12==mat21'&mat12==1,5)
        pause;
        close all;
    end
end