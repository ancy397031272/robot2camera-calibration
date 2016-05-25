function [ result ] = convert_kuka_points( kukapoints )
%Function to convert Kuka coordinates into the form that the UR uses so
%that they can be input into the calibration software

%Kuka points should be an nx6 matrix where n is the number of points

result = zeros(size(kukapoints,1),6);
result(:,1:3)=kukapoints(:,1:3);
result(:,4)=kukapoints(:,6);
result(:,5)=kukapoints(:,5);
result(:,6)=kukapoints(:,4);

end

