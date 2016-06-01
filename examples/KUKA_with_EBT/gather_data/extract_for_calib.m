function [ robot, ebt ] = extract_for_calib( filename, out )

if nargin<2
    [dir,name,~] = fileparts(filename);
    out = fullfile(dir,[name,'.json']);
end

%% startup
% filename = '2016-5-20_18-34-34.txt';

datastruct=importdata(filename);
raw_data=datastruct.data;

data = raw_data;

data(:,1)=data(:,1)-data(1,1);
data(:,21)=data(:,21)-data(1,21);


%% Find valid areas:
gaussFilter = fspecial('gaussian',[1,30],2);

velocities = abs(diff(data(:,22:27)));
norm_velocities = bsxfun(@rdivide,velocities,mean(velocities));
smooth_sum_velocity = conv(sum(norm_velocities,2), gaussFilter, 'same');
still = smooth_sum_velocity<(mean(smooth_sum_velocity)*.1);

tracking_index = raw_data(1:end-1,2)==1;
tracking_data = raw_data(tracking_index,:);

% still = s2d>1e-4;
tracking_still = still(tracking_index);

stop_start = find(diff(tracking_still)==1)+1;
stop_end = find(diff(tracking_still)==-1);

if ~isempty(stop_end)
    if isempty(stop_start)
        stop_start = [1];
    elseif stop_start(1)>stop_end(1)
        stop_start = [1;stop_start];
    end
end

if length(stop_end)<length(stop_start)
    stop_end = [stop_end;size(tracking_data,1)];
end

stopped = [stop_start, stop_end];
stopped_mean = mean(stopped,2);
stopped_range = range(stopped,2);
valid_ranges = floor([stopped_mean-.25*stopped_range,stopped_mean+.25*stopped_range]);
final_values = zeros(size(valid_ranges,1),size(tracking_data,2));
for i = 1:size(valid_ranges,1)
   final_values(i,:) = mean(tracking_data(valid_ranges(i,1):valid_ranges(i,2),:)); 
end

%% build matrix of ebt x,y,z,angle-axis vals
ebt_matricies = double(permute(reshape(final_values(:,4:19),[],4,4),[3,2,1]));
ebt = zeros(6,size(ebt_matricies,3));
ebt(1:3,:) = ebt_matricies(1:3,4,:);
R = rotm2axang(ebt_matricies(1:3,1:3,:));
ebt(4:6,:) = bsxfun(@times,R(:,1:3),R(:,4))';
ebt = ebt';
%convert to mm:
ebt(:,1:3) = ebt(:,1:3)*1000;

%% build matrix of robot x,y,z,angle-axis vals
robot = convert_kuka_points3(final_values(:,22:27));

%% Build JSON file
json_struct = struct('tcp2robot',robot,'camera2grid',ebt,'time',datestr(datetime('now')));

currPath = fileparts(mfilename('fullpath'));
addpath([currPath '/jsonlab']);
savejson('',json_struct,out);
rmpath([currPath '/jsonlab']);

%% Make Finding no motion graph
figure
subplot(3,1,1)
plot(velocities)
title('velocities (1st derivative) of KUKA motion')
legend('dx','dy','dz','da','db','dc')
ylim([0,max(quantile(velocities,.99))])
subplot(3,1,2)
plot(norm_velocities)
title('normalized velocities (1st derivative) of KUKA motion')
legend('dx','dy','dz','da','db','dc')
ylim([0,max(quantile(norm_velocities,.99))])
subplot(3,1,3)
plot(smooth_sum_velocity)
hold on
plot([0,length(smooth_sum_velocity)],[mean(smooth_sum_velocity)*.1,mean(smooth_sum_velocity)*.1])
area(still*max(quantile(smooth_sum_velocity,.99)),'FaceAlpha',.2,'LineStyle','none')
hold off
title('sum of normalized velocities (1st derivative) of KUKA motion (dx+dy+dz+da+db+dc), smoothed')
legend('smoothed sum of normalized velocities','cutoff','stopped')
ylim([0,max(quantile(smooth_sum_velocity,.99))])

%% Make Kuka movement graph
figure
yyaxis left
xyz_min = min(min(tracking_data(:,22:24)));
xyz_max = max(max(tracking_data(:,22:24)));
plot(tracking_data(:,21),tracking_data(:,22))
hold on
plot(tracking_data(:,21),tracking_data(:,23))
plot(tracking_data(:,21),tracking_data(:,24))
area(tracking_data(:,21),xyz_max*tracking_still,'FaceAlpha',.2,'FaceColor','g','LineStyle','none')

yyaxis right
plot(tracking_data(:,21),tracking_data(:,25))
plot(tracking_data(:,21),tracking_data(:,26))
plot(tracking_data(:,21),tracking_data(:,27))

hold off
legend('x','y','z','stopped','a','b','c')
hold on
yyaxis right
scatter(final_values(:,1),final_values(:,25))
scatter(final_values(:,1),final_values(:,26))
scatter(final_values(:,1),final_values(:,27))
yyaxis left
scatter(final_values(:,1),final_values(:,22))
scatter(final_values(:,1),final_values(:,23))
scatter(final_values(:,1),final_values(:,24))
area(tracking_data(:,21),xyz_min*tracking_still,'FaceAlpha',.2,'FaceColor','g','LineStyle','none')
ylim([xyz_min,xyz_max])
hold off
title('KUKA Robot Movement')

%% Make UR Movement Graph
figure
xyz_min = min(min(tracking_data(:,4:15)));
xyz_max = max(max(tracking_data(:,4:15)));
hold on
for i = 4:15
    plot(tracking_data(:,1),tracking_data(:,i))
    scatter(final_values(:,1),final_values(:,i))
end
area(tracking_data(:,21),xyz_max*tracking_still,'FaceAlpha',.2,'FaceColor','g','LineStyle','none')
area(tracking_data(:,21),xyz_min*tracking_still,'FaceAlpha',.2,'FaceColor','g','LineStyle','none')
hold off

title('EBT Movement')