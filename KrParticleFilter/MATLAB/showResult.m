clc;clear;
data = csvread('res.csv');
x = data(:, 1);
y = data(:, 2);
% w = data(:, 3);

Num = 0;
px = reshape(x, Num+1, []);
py = reshape(y, Num+1, []);
% pw = reshape(w, Num+1, []);


% cm = colormap; % returns the current color map
% f = i/Num;
% colorID = max(1, sum(f > [0:1/length(cm(:,1)):1]));
% myColor = cm(colorID, :); % returns your color

for i=1:Num
    plot(px(i, :), py(i, :), '.');
    hold on;
%     plot(px(i, :), pw(i, :),'Color', myColor);
    axis([-1 25  -3 3])
end
plot(px(end, :), py(end, :));
    axis([-1 25  -3 3])
% colorbar;

