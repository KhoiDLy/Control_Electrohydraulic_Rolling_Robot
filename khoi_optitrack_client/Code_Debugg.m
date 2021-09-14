% Code debugger and tester 
clear all;
close all;
clc;
Table = readtable('output_file_Sun_18_Apr_2021_08_35_57AM.csv');
Ts = 0.003333;

arm_data_previous = table2array(Table(1652,2:4));
pivot_data_previous = table2array(Table(1652,5:7));
Teensy_data_previous = table2array(Table(1652,11:14));

arm_data = table2array(Table(1652,2:4));
pivot_data = table2array(Table(1652,5:7));
Teensy_data = table2array(Table(1652,11:14));

arm_body = [];
pivot_body = [];
TeensyData = [];
vec_psi =[];
angular_vel_wheel_raw = [];
angular_vel_wheel = [];
data = [];
i = 1653;
while i < height(Table)
    
    arm_data_previous = arm_data;
    pivot_data_previous = pivot_data;
    Teensy_data_previous = Teensy_data;

    arm_data = table2array(Table(i,2:4));
    pivot_data = table2array(Table(i,5:7));
    Teensy_data = table2array(Table(i,11:14)); 
    
    
    if isnan(table2array(Table(i,:)))
        arm_data = arm_data_previous;
        pivot_data = pivot_data_previous;
        Teensy_data = Teensy_data_previous; 
    elseif Teensy_data(1,1) < 1000 || Teensy_data(1,2) < 1 || Teensy_data(1,2) > 16
        arm_data = arm_data_previous;
        pivot_data = pivot_data_previous;
        Teensy_data = Teensy_data_previous; 
    end
        
    arm_body = [arm_body; arm_data];
    pivot_body = [pivot_body; pivot_data];
    TeensyData = [TeensyData; Teensy_data];
    
    % Computing the new psi 
    xra_lc = arm_data(1) - pivot_data(1); 
    yra_lc = arm_data(3) - pivot_data(3); 
    psi =  atan2((yra_lc), (xra_lc)); % No need filtering, but this will loop 0 - 360 degree (doesn't really matter)
    % Adding new psi to the vector
    vec_psi = [vec_psi;psi];
    
    % If there are more than 200 data in each vector
    % We perform a moving median of 2oo-point window the angular_vel_wheel for
    if length(vec_psi) >= 2 
        angular_vel_wheel_raw = [angular_vel_wheel_raw; -0.64173 / 0.15677 * (vec_psi(end) - vec_psi(end-1)) / (Ts)]; % computing the raw angular velocity
        data(end+1) = angular_vel_wheel_raw(end);
        if length(angular_vel_wheel_raw) > 200
            angular_vel_wheel_raw(1) = []; % Remove the first element of the array
            % Assigning the vector angular_vel_wheel_raw to something before we sort them
            angular_vel_wheel = [angular_vel_wheel; median(angular_vel_wheel_raw)];
        end
    end
    % Remove the first index in the vectors only if the sizes are greater than 1000
    if length(vec_psi) > 1000
        arm_body(1,:) = [];
        pivot_body(1,:) = [];
        %TeensyData(1,:) = [];
        vec_psi(1) = [];
    end

%     if length(angular_vel_wheel) > 800
%         angular_vel_wheel(1) = [];
%     end 
    i = i +1;
    
    if Teensy_data(1,2)==8 && Teensy_data(1,4) == 1
       angular_vel_wheel(end)
       disp(angular_vel_wheel(end));
    end
    
end