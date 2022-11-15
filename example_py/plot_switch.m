%% Import data from text file
% Script for importing data from the following text file:
%
%    filename: /home/zhangchi/RDK/flexiv_rdk/example_py/csv_test.csv
%
% Auto-generated by MATLAB on 03-Nov-2022 10:21:58

clear


for i=0:2

    % Set up the Import Options and import the data
    opts = delimitedTextImportOptions("NumVariables", 2);

    % Specify range and delimiter
    opts.DataLines = [2, Inf];
    opts.Delimiter = ",";

    % Specify column names and types
    % opts.VariableNames = ["displacement", "force"];
    % opts.VariableTypes = ["double", "double"];
%     opts.VariableNames = ["displacement", "force0", "force1", "force2", "rawforce0", "rawforce1", "rawforce2"];
%     opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double"];
    opts.VariableNames = ["x","y","displacement", "forceZ", "rawforceZ", "theta1", "theta2", "theta3", "theta4", "theta5", "theta6", "theta7", "q1", "q2", "q3", "q4", "q5", "q6", "q7", "rawq1", "rawq2", "rawq3", "rawq4", "rawq5", "rawq6", "rawq7", "lq1", "lq2", "lq3", "lq4", "lq5", "lq6", "lq7"];
    opts.VariableTypes = ["double","double","double","double",  "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];

%     opts.VariableNames = ["displacement", "forceZ", "rawforceZ", "theta1", "theta2", "theta3", "theta4", "theta5", "theta6", "theta7", "q1", "q2", "q3", "q4", "q5", "q6", "q7", "rawq1", "rawq2", "rawq3", "rawq4", "rawq5", "rawq6", "rawq7", "lq1", "lq2", "lq3", "lq4", "lq5", "lq6", "lq7"];
%     opts.VariableTypes = ["double","double",  "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];

%     opts.VariableNames = ["displacement", "force", "theta1", "theta2", "theta3", "theta4", "theta5", "theta6", "theta7", "q1", "q2", "q3", "q4", "q5", "q6", "q7"];
%     opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];


    % Specify file level properties
    opts.ExtraColumnsRule = "ignore";
    opts.EmptyLineRule = "read";

    % Import the data
    % force_profile = readtable("/home/zhangchi/RDK/flexiv_rdk/example_py/m_jntPosFromOutputEncoder/0.001/csv_test1.csv", opts);

    % force_profile = readtable("/home/zhangchi/RDK/flexiv_rdk/example_py/m_jntPosRawOutputEncoder/0.001/csv_test2.csv", opts);
    % force_profile = readtable("/home/zhangchi/RDK/flexiv_rdk/example_py/csv_test_diagauge2.csv", opts);
    % force_profile = readtable("/home/zhangchi/RDK/flexiv_rdk/example_py/csv_test_linear_good_offseted.csv", opts);
    % force_profile = readtable("/home/zhangchi/RDK/flexiv_rdk/example_py/csv_test"+i+".csv", opts);

    %             csv = "PLAN-ControlTestA02L-MoveJointSpaceA"+joint_index+"_ClosedLoop_*.csv";
    folder_path = "/home/zhangchi/RDK/flexiv_rdk/example/build/";
    csv = "PressTest"+i+"*.csv";

    % csv = "PLAN-ControlTest-MoveJointSpaceA"+joint_index+"*.csv";
    %     csv = "SingleDOFSineTestA02L_A"+joint_index+"_CloseLoop_*.csv";
    % "/home/zhangchi/RDK/flexiv_rdk/example_py/csv_test"+i+".csv"

    filename = dir(folder_path+csv);
    filename = {filename.name};
    file_in = folder_path+filename{end};
    filename{end}

    force_profile = readtable(file_in, opts);
%     force_profile = readtable("/home/zhangchi/RDK/flexiv_rdk/example_py/LTT_data/link_side_off_noObv/brown/1/csv_test"+i+".csv", opts);
%     force_profile = readtable("/home/zhangchi/RDK/flexiv_rdk/example_py/LTT_data/RCA filter/csv_test"+i+".csv", opts);
    % Clear temporary variables
    clear opts

    %%
    % value = Press2.Value(159:408);
    % value = value-value(1)
    % time = Press2.Date.Second(159:408);
    % time = time-time(1);



    %%
    travel  = force_profile.displacement;
    z= force_profile.displacement;
    x= force_profile.x;
    y= force_profile.y;
    travel = -(travel-travel(1));
    % travel = travel(1:end-199);
    force = -force_profile.forceZ;
    raw_force = -force_profile.rawforceZ;
    theta = [force_profile.theta1 force_profile.theta2 force_profile.theta3 force_profile.theta4 force_profile.theta5 force_profile.theta6 force_profile.theta7 ];
    % raw_force = raw_force-raw_force(1);


    fc = 50;
    fs = 1000;
    [b,a] = butter(1,fc/(fs/2),'low');
    filter_raw_force = filtfilt(b,a,raw_force)

    revert_point = find(filter_raw_force==max(filter_raw_force));

    % force = force(200:end);
    figure()

    % plot(travel*1000,raw_force*100)
    hold on
    plot(travel(revert_point:end)*1000,filter_raw_force(revert_point:end)*100,'LineWidth',2)
    plot(travel(1:revert_point)*1000,filter_raw_force(1:revert_point)*100,'LineWidth',2)
    % plot(travel(revert_point:end)*1000,force(revert_point:end)*100,'LineWidth',2)
    % plot(travel(1:revert_point)*1000,force(1:revert_point)*100,'LineWidth',2)
    % hold on

    % xlim([2.5 7])
    xlabel("displacement (mm)")
    ylabel("Force (gram)")
    legend("release","press")
    grid on

    figure(101)
    hold on
    plot(travel)
    figure()
    plot3(x,y,z)


    figure(101);
    axis = [];
    for j=1:7
        subplot(2,4,j)
        axis = [axis gca()];
        hold on
        plot(theta((revert_point:end),j))
        plot(flip(theta((1:revert_point),j)))
        title("joint "+j);
    end
    sgtitle('Joint position');
    linkaxes(axis,"x")
    %
    %
    % travel_time = (16.594/14741):(16.594/14741):16.594;
    %
    % figure
    % plot(time,value)
    % hold on
    % plot(travel_time,travel*1000)
    % grid on
    %
    % figure
    % plot(time,value)
    % hold on
    % plot(travel_time,travel*1000)

end
filename{end}