rosshutdown
rosinit('192.168.1.2')

global zed_data
global zed_indx
global temp_index_zed
zed_indx = zed_indx+1;
temp_index_zed = 1;
sub_zed = rossubscriber('/zed/odom','nav_msgs/Odometry',@zed_callback);

global bebop_data
global bebop_indx
global temp_index_bebop
bebop_indx = bebop_indx+1;
temp_index_bebop = 1;
sub_bebop = rossubscriber('/bebop/odom','nav_msgs/Odometry',@bebop_callback);

fprintf('connected, recording on index %d and %d \n',zed_indx, bebop_indx)

close all
figure(1);
plot3(0,0,0)
hold on

%%
clear sub_zed
clear sub_bebop

%%

zed_indx = 0;
zed_data = cell(1000,10000);

bebop_indx = 0;
bebop_data = cell(1000,10000);



%%
dataTemp3 = zed_data(zed_indx,:);
dataTemp4 = zed_data(zed_indx,:);
dataTemp1 = [];
dataTemp2 = [];
for k = 1:10000
    dataTemp1 = [dataTemp1;cellfun(@str2num,strsplit(dataTemp2{k}{1},', '))];
    
    
    if ~iscell(NavPID_data{NavPID_indx,k+1})
       break 
    end
end

figure
hold on
grid on
plot(dataTemp(:,40),dataTemp(:,1))
plot(dataTemp(:,40),dataTemp(:,2))
plot(dataTemp(:,40),dataTemp(:,3))
plot(dataTemp(:,40),dataTemp(:,4))
plot(dataTemp(:,40),dataTemp(:,5))
plot(dataTemp(:,40),dataTemp(:,6))
% legend('X pos error','X Pos P','X Pos I','X Pos D','X Vel Des','Vel x')
% title('Position X')


%%

function zed_callback(src, msg)
    global bebop_data
    global bebop_indx
    bebop_q = bebop_data{bebop_indx,1}.Pose.Pose.Orientation;
    bebop_q = [bebop_q.W, bebop_q.X, bebop_q.Y, bebop_q.Z];
    
    global zed_data
    global zed_indx
    global temp_index_zed
    zed_data{zed_indx,temp_index_zed} = msg;
    temp_index_zed = temp_index_zed+1;
    pos = [msg.Pose.Pose.Position.X, msg.Pose.Pose.Position.Y, msg.Pose.Pose.Position.Z];
    pos = quatrotate(quatinv(bebop_q),pos);
    plot3(pos(1), pos(2), pos(3), 'xb')
    axis equal
end

function bebop_callback(src, msg)
    global bebop_data
    global bebop_indx
    global temp_index_bebop
    bebop_data{bebop_indx,temp_index_bebop} = msg;
    temp_index_bebop = temp_index_bebop+1;
    plot3(msg.Pose.Pose.Position.X, msg.Pose.Pose.Position.Y, msg.Pose.Pose.Position.Z, 'xr')
    axis equal
end