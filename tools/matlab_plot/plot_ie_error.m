clear;

addpath geoFunctions

% Frequency of lla0 should larger than lla1
lla0 = load('/media/cc/T72/ubuntu/datasets/gici/20230105_campus/03_parking/ie_gnssins.txt.tognss.txt');
lla1 = load('/media/cc/T72/ubuntu/datasets/gici/20230105_campus/03_parking/solution_rrr.txt.tognss.txt.ie');

D2R = pi / 180;
enu_error = zeros(size(lla1, 1), 4);
for i = 1 : size(lla1, 1)
    for j = 1 : size(lla0, 1) - 1
        if lla0(j,2) <= lla1(i,2) && lla0(j+1,2) > lla1(i,2)
            lla0_interpolate = (lla0(j+1,3:5) - lla0(j,3:5)) / (lla0(j+1,2) - lla0(j,2)) * (lla1(i,2) - lla0(j,2)) + lla0(j,3:5);
            enu_error(i,1) = lla1(i,2);
            [dn, de, du] = ComputeDeltaNEU(lla1(i,3)*D2R, lla1(i,4)*D2R, lla1(i,5), ...
                lla0_interpolate(1)*D2R, lla0_interpolate(2)*D2R, lla0_interpolate(3));
            enu_error(i,2) = de;
            enu_error(i,3) = dn;
            enu_error(i,4) = du;
            break;
        end
    end
    if enu_error(i,2) == 0 && enu_error(i,3) == 0 && enu_error(i,4) == 0
        enu_error(i,1) = nan;
    end
end
idx = 1;
while idx <= size(enu_error)
    if isnan(enu_error(idx,1))
        enu_error(idx,:) = [];
    else
        idx = idx + 1;
    end
end

%%
range = 0.2;
figure;

subplot(3, 1, 1);
plot(enu_error(:,1), enu_error(:,2));
grid on;
ylim([mean(enu_error(:,2))-range, mean(enu_error(:,2))+range]);
xlim([enu_error(1,1), enu_error(end,1)]);
legend('E');
ylabel('Error (m)');

subplot(3, 1, 2);
plot(enu_error(:,1), enu_error(:,3));
grid on;
ylim([mean(enu_error(:,3))-range, mean(enu_error(:,3))+range]);
xlim([enu_error(1,1), enu_error(end,1)]);
legend('N');
ylabel('Error (m)');

subplot(3, 1, 3);
plot(enu_error(:,1), enu_error(:,4));
grid on;
ylim([mean(enu_error(:,4))-range, mean(enu_error(:,4))+range]);
xlim([enu_error(1,1), enu_error(end,1)]);
legend('U');
ylabel('Error (m)');
xlabel('Time (s)');


