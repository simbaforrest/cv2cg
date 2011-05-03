clear; close all; clc;
if exist('tmp.mat','file')
    load('tmp.mat');
else
    imnames = cell(6,1);
    for i=1:6
        imnames{i} = ['../../orientation/Dataset/Mar7/',num2str(i),'.jpg'];
    end
    tic;
    try
        [keys,matches]=cvsurf(imnames);
    catch ErrMsg
        disp(ErrMsg);
        return
    end
    toc
    mask = rgb2gray(imread('../../orientation/Dataset/mask.jpg'));
    save('tmp.mat');
end

for i=1:5
    for j=(i+1):5
        matches{i,j} = matches{i,j} + 1; %opencv use 0-based index
        matches{j,i} = matches{j,i} + 1;
        mat12 = zeros(length(keys{i}),length(keys{j}));
        idx = sub2ind(size(mat12),matches{i,j}(:,1),matches{i,j}(:,2));
        mat12(idx)=1;
        mat21 = zeros(length(keys{j}),length(keys{i}));
        idx = sub2ind(size(mat21),matches{j,i}(:,1),matches{j,i}(:,2));
        mat21(idx)=1;
        subplot(1,3,1)
        spy(mat12,5)
        subplot(1,3,2)
        spy(mat21',5)
        subplot(1,3,3)
        correct = mat12==mat21' & mat12==1;
        disp(['img: ',num2str(i),'->',num2str(j), ' ', ...
            num2str(sum(correct(:))),' correct matches']);
        spy(correct,5)
        % extract correct matches (correct if i->j and j->i)
        cmatch = matches{i,j}(...
            matches{i,j}(:,2)==matches{j,i}(matches{i,j}(:,1),1), :);
        figure, hold on;
        im0 = rgb2gray(imread(imnames{i}));
        im1 = rgb2gray(imread(imnames{j}));
        [r0,c0] = size(im0);
        [r1,c1] = size(im1);
        r = max(r0,r1);
        c = c0+c1;
        combine = uint8(zeros(r,c));
        combine(1:r0,1:c0)=im0;
        combine(1:r1,1+c0:c1+c0)=im1;
        imshow(uint8(combine)); hold on;
        plot(keys{i}(cmatch(:,1),1),keys{i}(cmatch(:,1),2),'b*');
        plot(keys{j}(cmatch(:,2),1)+c0,keys{j}(cmatch(:,2),2),'r*');
%         disp(num2str(length(cmatch)));
        for k=1:length(cmatch)
            if mask(round(keys{i}(cmatch(k,1),2)),round(keys{i}(cmatch(k,1),1)))==255 && ...
                    mask(round(keys{j}(cmatch(k,2),2)),round(keys{j}(cmatch(k,2),1)))==255
                plot([keys{i}(cmatch(k,1),1),keys{j}(cmatch(k,2),1)+c0],...
                    [keys{i}(cmatch(k,1),2),keys{j}(cmatch(k,2),2)],'g-');
            end
        end
        pause;
        close all;
    end
end