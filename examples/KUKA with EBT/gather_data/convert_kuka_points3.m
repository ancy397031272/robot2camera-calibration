function [ result ] = convert_kuka_points3( kukapoints )
%Function to convert Kuka coordinates into the form that the UR uses so
%that they can be input into the calibration software

%Kuka points should be an nx6 matrix where n is the number of points with
% indecies 1-3 in mm and 4-6 in degrees

result = zeros(size(kukapoints,1),6);
result(:,1:3)=kukapoints(:,1:3);

RotM = eul2rotm(degtorad(kukapoints(:,4:6)));
R=rotm2axang(RotM);
result(:,4:6) = bsxfun(@times,R(:,1:3),R(:,4));
end
