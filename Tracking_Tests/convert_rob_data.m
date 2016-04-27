function [ Rmatrices ] = convert_rob_data( filename )
%Function which takes the correspondences.json file from robot 2 camera
%calibration and returns the corresponding rotation matrices

addpath('jsonlab/jsonlab/')

fid=fopen(filename);
scanned_file=fscanf(fid,'%s');
parsed_json=loadjson(scanned_file);
tcp2robot=parsed_json.tcp2robot;
Rmatrices=zeros(4,4,length(tcp2robot));
for i=1:size(tcp2robot,1)
    rmag=sqrt(tcp2robot(i,4)^2+tcp2robot(i,5)^2+tcp2robot(i,6)^2);
    rotation=rotm2tform(vrrotvec2mat([tcp2robot(i,4) tcp2robot(i,5) tcp2robot(i,6) rmag]));
    translation=trvec2tform([tcp2robot(i,1) tcp2robot(i,2) tcp2robot(i,3)]);
    %Rmatrices(1:4,1:4,i)=translation*rotation;
    Rmatrices(1:4,1:4,i)=rotation*translation;
end
fclose(fid);

