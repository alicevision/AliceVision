function [f,des] = readSiftFromTextFile(fileName)

M = dlmread(fileName, ' ', 2, 0);

f = M(:,1:5)';
f(1:2,:) = f(1:2,:)+1;
des = M(:,6:end)';

f(3,:) = sqrt(1./f(3,:));