function apriltag_listner
% listen AprilTagFinderLCM.tag, and plot it, close window to
% stop listener, close twice to close window.
% --simbaforrest
if ~exist('apriltag_lcm.TagPoseArray_t','class')
  javaaddpath('apriltag_lcm.jar');
end

fprintf('opening lcm...\n');
listener = LCM_matlab('AprilTagFinderLCM.tag', 'apriltag_lcm.TagPoseArray_t');

% prepare your own matlab figure
isDone=false;
  function onClose(~,~) %a nested function to stop the detection loop below
    isDone = true;
  end
fig_kuka=figure('name','apriltag_listener',...
  'CloseRequestFcn',@onClose);

while ~isDone
  clf(fig_kuka);
  vz.img(640,480);
  
  m = listener.waitForNext(100);
  if isempty(m)
    fprintf('......\n');
    pause(.1);
    continue;
  end
    
  fprintf('#detections=%3d @ %d\n', m.num_detections, m.timestamp);
  
  hold on;
  for j=1:m.num_detections
    mj = m.detections(j);
    vz.lineloop(mj.p','color',rand(3,1));
    text(mj.cxy(1), mj.cxy(2), sprintf('%s.%d', char(mj.familyName), mj.id));
  end
  drawnow;
end

delete(listener);
set(fig_kuka,'CloseRequestFcn','closereq');
end