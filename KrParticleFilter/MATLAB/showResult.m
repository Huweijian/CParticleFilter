clc;clear;
data = csvread('res.csv');
x = data(:, 1);
y = data(:, 2);

Num = 1;
px = reshape(x, Num, []);
py = reshape(y, Num, []);
for i=1:length(px)
%     axis([-1 1 -1 1])
    plot(px(:, i), py(:, i), '.');
    hold on;
end
    