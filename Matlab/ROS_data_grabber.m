rosshutdown
rosinit('192.168.1.2')

clear sub_nav
clear sub_gate

global NavPID_data
global NavPID_indx
global temp_index_Nav
NavPID_indx = NavPID_indx + 1;
temp_index_Nav = 1;
sub_nav = rossubscriber('/auto/navigation_logger','std_msgs/String',@nav_callback);

global Gate_data
global Gate_indx
global temp_index_Gate
Gate_indx = Gate_indx+1;
temp_index_Gate = 1;
% sub_gate = rossubscriber('/auto/visual_logger','std_msgs/String',@gate_callback);

fprintf('connected, recording on index %d and %d \n',NavPID_indx, Gate_indx)


%%
clear sub_nav
clear sub_gate

%%
save('GateData.mat','Gate_data','Gate_indx')
save('NavPID.mat','NavPID_data','NavPID_indx')
fprintf('Saved the data')

%%
load('NavPID.mat','NavPID_data','NavPID_indx')
load('GateData.mat','Gate_data','Gate_indx')
fprintf('Data loaded')

%%
NavPID_indx = NavPID_indx - 1
Gate_indx = Gate_indx - 1

%%
NavPID_indx = NavPID_indx + 1
Gate_indx = Gate_indx + 1

%%

NavPID_indx = 0;
NavPID_data = cell(1000,10000);

Gate_indx = 0;
Gate_data = cell(1000,10000);


%%

function nav_callback(src, msg)
    global NavPID_data
    global NavPID_indx
    global temp_index_Nav
    NavPID_data{NavPID_indx,temp_index_Nav} = cellstr(msg.Data);
    temp_index_Nav = temp_index_Nav+1;
end
function gate_callback(src, msg)
    global Gate_data
    global Gate_indx
    global temp_index_Gate
    temp = strsplit(msg.Data,', ');
    if temp{1} ~= '0'
        Gate_data{Gate_indx,temp_index_Gate} = cellstr(msg.Data);
        temp_index_Gate = temp_index_Gate+1;
    end
end