function [ result ] = convert_kuka_points( kukapoints )
%Function to convert Kuka coordinates into the form that the UR uses so
%that they can be input into the calibration software

%Kuka points should be an nx6 matrix where n is the number of points

result = zeros(size(kukapoints,1),6);
result(:,1:3)=kukapoints(:,1:3);

euls = kukapoints(:,4:6);
rotms = eul2rotm(euls);
axangs = rotm2axang(rotms);
rvecs = zeros(size(axangs,1),3);
for i = 1:size(axangs,1)
    rvecs(i,:) = axangs(i,1:3)*axangs(i,4);
end
result(:,4:6) = rvecs;

end

