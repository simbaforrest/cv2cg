function [K,R,T]=cvreadpar(name)
fid = fopen(name);

% cnt=0;
while true
%     cnt = cnt+1;
    tline = fgetl(fid);
    if ~ischar( tline )
        break;
    elseif isempty(tline) || tline(1)=='#'
        continue;
    end
%     disp(['[line ',num2str(cnt),' =]',tline]);
    if strcmp(tline,'K(alphaX alphaY u0 v0)=')
        tmp=fscanf(fid,'%f%*[^0-9-+.eE]%f%*[^0-9-+.eE]%f%*[^0-9-+.eE]%f');
        K=[tmp(1) 0 tmp(3);
            0 tmp(2) tmp(4)
            0 0 1];
    elseif strcmp(tline, 'R=')
        tmp=fscanf(fid,'%f%*[^0-9-+.eE]%f%*[^0-9-+.eE]%f%*[^0-9-+.eE]%f%*[^0-9-+.eE]%f%*[^0-9-+.eE]%f%*[^0-9-+.eE]%f%*[^0-9-+.eE]%f%*[^0-9-+.eE]%f');
        R=[tmp(1:3)';tmp(4:6)';tmp(7:9)'];
    elseif strcmp(tline, 'T=')
        tmp=fscanf(fid,'%f%*[^0-9-+.eE]%f%*[^0-9-+.eE]%f');
        T=tmp(:);
    end
end

fclose(fid);
end