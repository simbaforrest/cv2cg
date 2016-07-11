function [m,img]=apriltag_listner(showImage, verbose)
% listen AprilTagFinderLCM.tag, and plot it, close window to
% stop listener, close twice to close window.
% --simbaforrest
if ~exist('showImage','var'); showImage=false; end;
if ~exist('verbose','var'); verbose=false; end;

if ~exist('apriltag_lcm.TagPoseArray_t','class')
  javaaddpath(fullfile(fileparts(mfilename('fullpath')),...
    'apriltag_lcm.jar'));
end
if showImage
  if ~exist('image_lcm.image_t','class')
    javaaddpath(fullfile(fileparts(mfilename('fullpath')),...
      'image_lcm.jar'));
  end
end

if verbose
  fprintf('opening lcm...\n');
end
tag_listener = LCM_matlab('AprilTagFinderLCM.tag',...
  'apriltag_lcm.TagPoseArray_t', 1024^2);
if showImage
  img_listener = LCM_matlab('AprilTagFinderLCM.img',...
    'image_lcm.image_t', 5*1024^2);
end

% prepare your own matlab figure
isDone=false;
  function onClose(~,~) %a nested function to stop the detection loop below
    isDone = true;
    set(fig_apriltag,'CloseRequestFcn','closereq');
  end
fig_apriltag=figure('name',...
  'apriltag_listener: click X once to stop, twice to close',...
  'CloseRequestFcn',@onClose);

m=[];
img=[];
img_msg=[];
while ~isDone
  clf(fig_apriltag);
  if ~showImage
    vz.img(640,480);
  end
  
  if showImage
    img_msg = img_listener.waitForNext(25);
  end
  m = tag_listener.waitForNext(25);
  if isempty(m) && (showImage && isempty(img_msg))
    if verbose
      fprintf('......\n');
    end
    pause(.1);
    continue;
  end
  
  if verbose
    fprintf('#detections=%3d @ %d\n', m.num_detections, m.timestamp);
  end
  
  if showImage && ~isempty(img_msg)
    img=image_t2uint8(img_msg);
    imshow(img);
  end
  hold on;
  if ~isempty(m)
    for j=1:m.num_detections
      mj = m.detections(j);
      vz.lineloop(mj.p','color',rand(3,1));
      text(mj.cxy(1), mj.cxy(2),...
        sprintf('%s.%d', char(mj.familyName), mj.id));
    end
  end
  drawnow;
end

delete(tag_listener);
end