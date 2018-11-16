%%
% 1 wp_current.pos[0]
% 2 wp_current.pos[1]
% 3 wp_current.pos[2]
% 4 wp_current.hdg
% 5 wp_average.pos[0]
% 6 wp_average.pos[1]
% 7 wp_average.pos[2]
% 8 wp_average.hdg
% 9 bebop_p[0][0]
% 10 bebop_p[1][0]
% 11 bebop_p[2][0]
% 12 bebop_q[3]
% 13 bebop_q[0]
% 14 bebop_q[1]
% 15 bebop_q[2]
% 16 heading_to_gate
% 17 std deviation
% 18 number
% 19 time stamp

                     
%% All
dataTemp2 = Gate_data(Gate_indx,:);
dataTemp = [];
for k = 1:10000
    dataTemp = [dataTemp;cellfun(@str2num,strsplit(dataTemp2{k}{1},', '))];
    
    if ~iscell(Gate_data{Gate_indx,k+1})
       break 
    end
end

figure
hold on
grid on
plot(dataTemp(:,19),dataTemp(:,1))
plot(dataTemp(:,19),dataTemp(:,2))
plot(dataTemp(:,19),dataTemp(:,3))
plot(dataTemp(:,19),dataTemp(:,4))
plot(dataTemp(:,19),dataTemp(:,5))
plot(dataTemp(:,19),dataTemp(:,6))
plot(dataTemp(:,19),dataTemp(:,7))
plot(dataTemp(:,19),dataTemp(:,8))
plot(dataTemp(:,19),dataTemp(:,9))
plot(dataTemp(:,19),dataTemp(:,10))
plot(dataTemp(:,19),dataTemp(:,11))
plot(dataTemp(:,19),dataTemp(:,12))
plot(dataTemp(:,19),dataTemp(:,13))
plot(dataTemp(:,19),dataTemp(:,14))
plot(dataTemp(:,19),dataTemp(:,15))
plot(dataTemp(:,19),dataTemp(:,16))
legend('wp current.pos[0]','wp current.pos[1]','wp current.pos[2]','wp current.hdg','wp average.pos[0]','wp average.pos[1]','wp average.pos[2]','wp average.hdg','bebop p[0][0]','bebop p[1][0]','bebop p[2][0]','bebop q[3]','bebop q[0]','bebop q[1]','bebop q[2]','heading to gate')

%% Gate Pos
dataTemp2 = Gate_data(Gate_indx,:);
dataTemp = [];
for k = 1:10000
    dataTemp = [dataTemp;cellfun(@str2num,strsplit(dataTemp2{k}{1},', '))];
    
    if ~iscell(Gate_data{Gate_indx,k+1})
       break 
    end
end

figure
hold on
grid on
plot(dataTemp(:,19),dataTemp(:,1))
plot(dataTemp(:,19),dataTemp(:,2))
plot(dataTemp(:,19),dataTemp(:,3))
plot(dataTemp(:,19),dataTemp(:,4))
plot(dataTemp(:,19),dataTemp(:,5),'x-')
plot(dataTemp(:,19),dataTemp(:,6),'x-')
plot(dataTemp(:,19),dataTemp(:,7),'x-')
plot(dataTemp(:,19),dataTemp(:,8),'x-')

legend('wp current.pos[0]','wp current.pos[1]','wp current.pos[2]','wp current.hdg','wp average.pos[0]','wp average.pos[1]','wp average.pos[2]','wp average.hdg')

%% Particular

dataTemp2 = Gate_data(Gate_indx,:);
dataTemp = [];
for k = 1:10000
    dataTemp = [dataTemp;cellfun(@str2num,strsplit(dataTemp2{k}{1},', '))];
    
    if ~iscell(Gate_data{Gate_indx,k+1})
       break 
     end
end


hold on
grid on
plot(dataTemp(:,19),dataTemp(:,4)*180/pi)
plot(dataTemp(:,19),dataTemp(:,8)*180/pi)
legend('curr','avg')