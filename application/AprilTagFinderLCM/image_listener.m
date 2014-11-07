lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();

lc.subscribe('AprilTagFinderLCM.img', aggregator);

while true
    disp waiting
    millis_to_wait = 1000;
    msg = aggregator.getNextMessage(millis_to_wait);
    if length(msg) > 0
        break
    end
end

disp(sprintf('channel of received message: %s', char(msg.channel)))

m = image_lcm.image_t(msg.data);

disp(sprintf('decoded message:\n'))
disp([ 'timestamp:   ' sprintf('%d ', m.timestamp) ])
disp([ 'w:           ' sprintf('%f ', m.w) ])
disp([ 'h:           ' sprintf('%f ', m.h) ])
disp([ 'c:           ' sprintf('%f ', m.c) ])
disp([ 'len:         ' sprintf('%f ', m.len) ])
