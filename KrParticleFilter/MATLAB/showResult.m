clc;clear;
data = csvread('res.csv');
plot(data(:, 2), data(:, 3));