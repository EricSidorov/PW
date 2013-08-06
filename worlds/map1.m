clc; clear all; close all;
N = 129;
% varying slopes
A=ones(N,N);
EndPoint = round([N,N/1.2,N/1.3,N/1.4]);
w = 30;
k = 3;
for i = 1:length(EndPoint)
    row = linspace(0,1,EndPoint(i));
    for j = k:k+w
        A(j,1:EndPoint(i))=row;
    end
    k = k+w+1;
end
%parabole
B = ones(N,N);
x = linspace(0,1,round(7*N/8));
B(1:length(x),:) = repmat(x.^2,N,1).';
%downhill parabole
D = 1-B;
A = mat2gray(A.');
B = mat2gray(B);
%contract to single map
map = [B(1:end-1,1:end-1),A(1:end-1,:);D(:,1:end-1),flipud(A)];
%show and save
figure
imshow(map)
imwrite(map,'playground.png','png');