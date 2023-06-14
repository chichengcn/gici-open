%% Read
path = '/home/cc/datasets/log_estimator/ionosphere-20220721-121244.log';
fp = fopen(path, 'r');
max_prn = 100;
length_prn = 0;
data = cell(max_prn, 1);
prns = strings(max_prn, 1);
data_lengths = zeros(max_prn, 1);
initial_time = 0;
end_time = 0;
for i = 1 : max_prn
    data{i} = zeros(3600 * 6, 2);
end
while ~feof(fp)    
    line = fgetl(fp);                                 
    splitLine = strsplit(line);

    if line(1) == '>'
        time = str2double(splitLine(8));
        if initial_time == 0
            initial_time = time;
        end
        end_time = time;
    else
        prn = line(1:3);
        value = str2double(splitLine(2));
        index = 0;
        for i = 1 : max_prn
            if prn == prns(i)
                index = i;
                break;
            end
        end
        if index == 0
            length_prn = length_prn + 1;
            prns(length_prn) = prn;
            index = length_prn;
        end
        data_lengths(index) = data_lengths(index) + 1;
        data{index}(data_lengths(index), 1) = time;
        data{index}(data_lengths(index), 2) = value;
    end
end
fclose(fp);

%% Plot
figure;
legends = cell(length_prn, 1);
for i = 1 : length_prn
    plot(data{i}(1:data_lengths(i),1) - initial_time, data{i}(1:data_lengths(i),2));
    hold on;
    legends{i} = prns(i);
end
legend(legends);
grid on;
xlim([0, end_time - initial_time]);

