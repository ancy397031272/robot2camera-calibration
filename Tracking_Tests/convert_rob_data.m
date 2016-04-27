function [ Rmatrices ] = convert_rob_data( filename )
%Function which takes the correspondences.json file from robot 2 camera
%calibration and returns the corresponding rotation matrices

fid=fopen(filename);
scanned_file=fscanf(fid,'%s');
parsed_json=parse_json(scanned_file);
jstruct=parsed_json{1};
tcp2robot=jstruct.tcp2robot;
Rmatrices=zeros(4,4,length(tcp2robot));
for i=1:length(tcp2robot)
    datacell=tcp2robot{i};
    rmag=sqrt(datacell{4}^2+datacell{5}^2+datacell{6}^2);
    rotation=quat2tform([rmag datacell{4} datacell{5} datacell{6}]);
    translation=trvec2tform([datacell{1} datacell{2} datacell{3}])
    %Rmatrices(1:4,1:4,i)=translation*rotation;
    Rmatrices(1:4,1:4,i)=rotation*translation;
end
fclose(fid);

