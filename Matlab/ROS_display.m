% x_pos_error 
% x_vel_des[0] 
% x_vel_des[1] 
% x_vel_des[2] 
% sum(x_vel_des) 
% global_vel[0] 
% x_vel_error 
% nav_cmd_x[0] 
% nav_cmd_x[1] 
% nav_cmd_x[2] 
% sum(nav_cmd_x) 
% msg.x 
% y_pos_error 
% y_vel_des[0] 
% y_vel_des[1] 
% y_vel_des[2] 
% sum(y_vel_des) 
% global_vel[1] 
% y_vel_error 
% nav_cmd_y[0] 
% nav_cmd_y[1] 
% nav_cmd_y[2] 
% sum(nav_cmd_y) 
% msg.y 
% diff_global[2] 
% z_error 
% nav_cmd_z[0] 
% nav_cmd_z[1] 
% nav_cmd_z[2] 
% sum(nav_cmd_z) 
% msg.z 
% pos_theta 
% angle 
% r_error 
% nav_cmd_r[0] 
% nav_cmd_r[1] 
% nav_cmd_r[2] 
% sum(nav_cmd_r))
% msg.r
% time.time()
% 0
% 0
% 0
% 0

%% X and Y Position/Des Vel

dataTemp2 = NavPID_data(NavPID_indx,:);
dataTemp = [];
for k = 1:1000
    dataTemp = [dataTemp;cellfun(@str2num,strsplit(dataTemp2{k}{1},', '))];
    
    if ~iscell(NavPID_data{NavPID_indx,k+1})
       break 
    end
end

figure
hold on
grid on
plot(dataTemp(:,40),dataTemp(:,1))
plot(dataTemp(:,40),dataTemp(:,5),'--')
plot(dataTemp(:,40),dataTemp(:,6),'-o')
plot(dataTemp(:,40),dataTemp(:,13))
plot(dataTemp(:,40),dataTemp(:,17),'--')
plot(dataTemp(:,40),dataTemp(:,18),'-o')

legend('X pos error','X vel des','X vel','Y pos error','Y vel des','Y vel')
title('Error and Des')
%% X and Y Position

dataTemp2 = NavPID_data(NavPID_indx,:);
dataTemp = [];
for k = 1:1000
    dataTemp = [dataTemp;cellfun(@str2num,strsplit(dataTemp2{k}{1},', '))];
    
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
legend('X pos error','X Pos P','X Pos I','X Pos D','X Vel Des','Vel x')
title('Position X')

figure
hold on
grid on
plot(dataTemp(:,40),dataTemp(:,13))
plot(dataTemp(:,40),dataTemp(:,14))
plot(dataTemp(:,40),dataTemp(:,15))
plot(dataTemp(:,40),dataTemp(:,16))
plot(dataTemp(:,40),dataTemp(:,17))
plot(dataTemp(:,40),dataTemp(:,18))
legend('Y Pos Error','Y Pos P','Y Pos I','Y Pos D','Y Vel Des','Vel Y')
title('Position Y')


%% X Total
dataTemp2 = NavPID_data(NavPID_indx,:);
dataTemp = [];
for k = 1:1000
    dataTemp = [dataTemp;cellfun(@str2num,strsplit(dataTemp2{k}{1},', '))];
    
    if ~iscell(NavPID_data{NavPID_indx,k+1})
       break 
    end
end

figure
hold on
grid on
% plot(dataTemp(:,40)-dataTemp(1,40),dataTemp(:,1))
plot(dataTemp(:,40),dataTemp(:,1))
plot(dataTemp(:,40),dataTemp(:,2))
plot(dataTemp(:,40),dataTemp(:,3))
plot(dataTemp(:,40),dataTemp(:,4))
plot(dataTemp(:,40),dataTemp(:,5))
plot(dataTemp(:,40),dataTemp(:,6))
plot(dataTemp(:,40),dataTemp(:,7))
plot(dataTemp(:,40),dataTemp(:,8))
plot(dataTemp(:,40),dataTemp(:,9))
plot(dataTemp(:,40),dataTemp(:,10))
plot(dataTemp(:,40),dataTemp(:,11))
plot(dataTemp(:,40),dataTemp(:,12))
legend('x pos error','X Pos P','X Pos I','X Pos D','X Vel Des','Vel x','X vel error','X Vel P','X Vel I','X Vel D','X Vel Out','msg.x')

%% X Pos/Vel
dataTemp2 = NavPID_data(NavPID_indx,:);
dataTemp = [];
for k = 1:1000
    dataTemp = [dataTemp;cellfun(@str2num,strsplit(dataTemp2{k}{1},', '))];
    
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
legend('X pos error','X Pos P','X Pos I','X Pos D','X Vel Des','Vel x')
title('Position')


figure
hold on
grid on
plot(dataTemp(:,40),dataTemp(:,5),'o-')
plot(dataTemp(:,40),dataTemp(:,6))
plot(dataTemp(:,40),dataTemp(:,7))
plot(dataTemp(:,40),dataTemp(:,8))
plot(dataTemp(:,40),dataTemp(:,9))
plot(dataTemp(:,40),dataTemp(:,10))
plot(dataTemp(:,40),dataTemp(:,11))
plot(dataTemp(:,40),dataTemp(:,12))
legend('X Vel Des','Vel x','X vel error','X Vel P','X Vel I','X Vel D','X Vel Out','msg.x')
title('Velocity')

%% Y Total
dataTemp2 = NavPID_data(NavPID_indx,:);
dataTemp = [];
for k = 1:1000
    dataTemp = [dataTemp;cellfun(@str2num,strsplit(dataTemp2{k}{1},', '))];
    
    if ~iscell(NavPID_data{NavPID_indx,k+1})
       break 
    end
end

figure
hold on
grid on
plot(dataTemp(:,40),dataTemp(:,13))
plot(dataTemp(:,40),dataTemp(:,14))
plot(dataTemp(:,40),dataTemp(:,15))
plot(dataTemp(:,40),dataTemp(:,16))
plot(dataTemp(:,40),dataTemp(:,17))
plot(dataTemp(:,40),dataTemp(:,18))
plot(dataTemp(:,40),dataTemp(:,19))
plot(dataTemp(:,40),dataTemp(:,20))
plot(dataTemp(:,40),dataTemp(:,21))
plot(dataTemp(:,40),dataTemp(:,22))
plot(dataTemp(:,40),dataTemp(:,23))
plot(dataTemp(:,40),dataTemp(:,24))
legend('Y Pos Error','Y Pos P','Y Pos I','Y Pos D','Y Vel Des','Vel Y','Y Vel Error','Y Vel P','Y Vel I','Y Vel D','Y Vel Out','msg.y')




%% Y Pos/Vel
dataTemp2 = NavPID_data(NavPID_indx,:);
dataTemp = [];
for k = 1:1000
    dataTemp = [dataTemp;cellfun(@str2num,strsplit(dataTemp2{k}{1},', '))];
    
    if ~iscell(NavPID_data{NavPID_indx,k+1})
       break 
    end
end

figure
hold on
grid on
plot(dataTemp(:,40),dataTemp(:,13))
plot(dataTemp(:,40),dataTemp(:,14))
plot(dataTemp(:,40),dataTemp(:,15))
plot(dataTemp(:,40),dataTemp(:,16))
plot(dataTemp(:,40),dataTemp(:,17))
plot(dataTemp(:,40),dataTemp(:,18))
legend('Y Pos Error','Y Pos P','Y Pos I','Y Pos D','Y Vel Des','Vel Y')
title('Position')

figure
hold on
grid on
plot(dataTemp(:,40),dataTemp(:,17),'o-')
plot(dataTemp(:,40),dataTemp(:,18))
plot(dataTemp(:,40),dataTemp(:,19))
plot(dataTemp(:,40),dataTemp(:,20))
plot(dataTemp(:,40),dataTemp(:,21))
plot(dataTemp(:,40),dataTemp(:,22))
plot(dataTemp(:,40),dataTemp(:,23))
plot(dataTemp(:,40),dataTemp(:,24))
legend('Y Vel Des','Vel Y','Y Vel Error','Y Vel P','Y Vel I','Y Vel D','Y Vel Out','msg.y')
title('Velocity')


%% Z
dataTemp2 = NavPID_data(NavPID_indx,:);
dataTemp = [];
for k = 1:1000
    dataTemp = [dataTemp;cellfun(@str2num,strsplit(dataTemp2{k}{1},', '))];
    
    if ~iscell(NavPID_data{NavPID_indx,k+1})
       break
    end
end

figure
hold on
grid on
plot(dataTemp(:,40),dataTemp(:,26),'-o')
plot(dataTemp(:,40),dataTemp(:,27))
plot(dataTemp(:,40),dataTemp(:,28))
plot(dataTemp(:,40),dataTemp(:,29))
plot(dataTemp(:,40),dataTemp(:,30))
plot(dataTemp(:,40),dataTemp(:,31))
legend('Z Error','Z Pos P','Z Pos I','D term','Z Vel Out','msg.z')

%% R
dataTemp2 = NavPID_data(NavPID_indx,:);
dataTemp = [];
for k = 1:1000
    dataTemp = [dataTemp;cellfun(@str2num,strsplit(dataTemp2{k}{1},', '))];
    
    if ~iscell(NavPID_data{NavPID_indx,k+1})
       break 
    end
end

figure
hold on
grid on
plot(dataTemp(:,40),dataTemp(:,32),'x-')
plot(dataTemp(:,40),dataTemp(:,33))
plot(dataTemp(:,40),dataTemp(:,34))
plot(dataTemp(:,40),dataTemp(:,35))
plot(dataTemp(:,40),dataTemp(:,36))
plot(dataTemp(:,40),dataTemp(:,37))
plot(dataTemp(:,40),dataTemp(:,38))
plot(dataTemp(:,40),dataTemp(:,39))
legend('Angle Des','Vehicle Angle','R error','P term','I term','D term','nav cmd R','msg.r')


%% mics
dataTemp2 = NavPID_data(NavPID_indx,:);
dataTemp = [];
for k = 1:1000
    dataTemp = [dataTemp;cellfun(@str2num,strsplit(dataTemp2{k}{1},', '))];
    
    if ~iscell(NavPID_data{NavPID_indx,k+1})
       break 
    end
end
figure
hold on
grid on
plot(dataTemp(:,40),dataTemp(:,40))
plot(dataTemp(:,40),dataTemp(:,41))
plot(dataTemp(:,40),dataTemp(:,42))
plot(dataTemp(:,40),dataTemp(:,43))
plot(dataTemp(:,40),dataTemp(:,44))
legend('1','2','3','4','5')


%% dyn
dataTemp2 = NavPID_data(NavPID_indx,:);
dataTemp = [];

for k = 1:1000
    dataTemp2{k}{1} = strrep(dataTemp2{k}{1},'through','-1');
    
    if ~iscell(NavPID_data{NavPID_indx,k+1})
       break 
    end
end

for k = 1:1000
    if k ~= 8
        dataTemp = [dataTemp;cellfun(@str2num,strsplit(dataTemp2{k}{1},', '))];
    end

    if ~iscell(NavPID_data{NavPID_indx,k+1})
       break 
    end
end

figure
hold on
grid on
first_val = 1;
last_val = size(dataTemp,1);
% last_val = 481;

vec = dataTemp(first_val:last_val,11);
vec(vec==0)=[];

plot(dataTemp(first_val:last_val,40),dataTemp(first_val:last_val,2))
plot(dataTemp(first_val:last_val,40),dataTemp(first_val:last_val,3))
plot(dataTemp(first_val:last_val,4),dataTemp(first_val:last_val,5))
plot(dataTemp(first_val:last_val,40),dataTemp(first_val:last_val,6))
plot(dataTemp(first_val:last_val,40),dataTemp(first_val:last_val,7),'og')
plot(dataTemp(first_val:last_val,40),dataTemp(first_val:last_val,9),'xr')
plot(dataTemp(first_val:last_val,40),dataTemp(first_val:last_val,10),'ok')
plot(vec,0*ones(length(vec),1),'xm')
plot(dataTemp(first_val:last_val,40),dataTemp(first_val:last_val,12))

legend('freq','offset','measurement','deviation','current angle','theta trigger','angle diff','exec time', 'throttle')
