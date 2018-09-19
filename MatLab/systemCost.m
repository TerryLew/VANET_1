function [outputArg1,outputArg2] = systemCost(filename_)
%SYSTEMCOST Summary of this function goes here
%   Detailed explanation goes here
if exist('filename_','var')
    data = dlmread(filename_);
    t = data(:,1);
    sc = data(:,2);
    plot(t, sc, 'LineWidth', 2);
else
    
step = 20;

costFile0 = 'cost0_0.txt';
costFile1 = 'cost0_1.txt';
costFile2 = 'cost0_2.txt';
costFile3 = 'cost0_3.txt';
costFile4 = 'cost0_4.txt';
costFile5 = 'cost0_5.txt';
costFile6 = 'cost0_6.txt';
costFile7 = 'cost0_7.txt';

data = dlmread(costFile0);
t = data(:,1);
sc = data(:,2);
t = t(1:step:end);
sc = sc(1:step:end);
plot(t, sc, 'LineWidth', 2);%, 'Color','b', 'Marker','o', 'MarkerSize', 8);
hold on;

data = dlmread(costFile3);
t = data(:,1);
sc = data(:,2);
t = t(1:step:end);
sc = sc(1:step:end);
plot(t, sc, 'LineWidth', 2);%, 'Color','b', 'Marker','^', 'MarkerSize', 8);

data = dlmread(costFile7);
t = data(:,1);
sc = data(:,2);
t = t(1:step:end);
sc = sc(1:step:end);
plot(t, sc, 'LineWidth', 2);%, 'Color','b', 'Marker','*', 'MarkerSize', 8);

ylabel('System Cost', 'FontSize', 20);
legend('System Cost, B = [3 3]^T', 'System Cost, B = [3 9]^T', 'System Cost, B = [3 27]^T');

figure;

flowFile0 = 'flow0_0.txt';
flowFile1 = 'flow0_1.txt';
flowFile2 = 'flow0_2.txt';
flowFile3 = 'flow0_3.txt';
flowFile4 = 'flow0_4.txt';
flowFile5 = 'flow0_5.txt';
flowFile6 = 'flow0_6.txt';
flowFile7 = 'flow0_7.txt';

data = dlmread(flowFile0);
t = data(:,1);
x1 = data(:,2);
t = t(1:step:end);
x1 = x1(1:step:end);
plot(t, x1, 'LineWidth', 2);
hold on;

data = dlmread(flowFile3);
t = data(:,1);
x1 = data(:,2);
t = t(1:step:end);
x1 = x1(1:step:end);
plot(t, x1, 'LineWidth', 2);

data = dlmread(flowFile7);
t = data(:,1);
x1 = data(:,2);
t = t(1:step:end);
x1 = x1(1:step:end);
plot(t, x1, 'LineWidth', 2);

legend('Flow on Route 1, B = [3 3]^T', 'Flow on Route 1, B = [3 9]^T', 'Flow on Route 1, B = [3 27]^T');

ylabel('Flow on Route 1','FontSize', 20);
xlabel('Time (s)','FontSize', 20);


end

end

