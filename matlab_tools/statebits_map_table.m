function statebits_map_table(maxValues)
    % 生成状态组合和索引表
    % 输入: maxValues - 一个包含每个状态位最大值的数组。例如 [1, 1, 2, 3]
    
    % 计算状态位的数量
    numFlags = length(maxValues);
    
    % 计算每个标志位的位宽，并存储偏移值
    bitWidths = arrayfun(@(x) ceil(log2(x + 1)), maxValues);
    shifts = [0, cumsum(bitWidths(1:end-1))];
    
    % 计算总组合数量
    totalCombinations = prod(maxValues + 1);
    
    % 初始化表格
    stateTable = zeros(totalCombinations, numFlags + 1); % 包含标志位值、索引值
    
    % 生成所有组合
    row = 1;
    for combo = 0:(totalCombinations-1)
        % 计算每个标志位的当前值
        flagValues = decodeCombination(combo, maxValues);
        
        % 检查是否超出每个状态位的最大值范围
        if any(flagValues > maxValues)
            continue; % 跳过超出范围的组合
        end
        
        % 计算索引值
        index = 0;
        for i = 1:numFlags
            index = index + (flagValues(i) * (2 ^ shifts(i)));
        end
        
        % 填写表格
        stateTable(row, 1:numFlags) = flagValues;
        stateTable(row, numFlags + 1) = index; % 索引值
        
        row = row + 1;
    end
    
    % 删除未使用的表格行
    stateTable = stateTable(1:row-1, :);
    
    % 显示表格
    disp("状态组合及对应索引值表格:");
    header = ['Row', arrayfun(@(x) ['Flag', num2str(x)], 1:numFlags, 'UniformOutput', false), "Index"];
    disp(header);
    
    % 显示行号和状态表格
    for r = 1:size(stateTable, 1)
        disp([r, stateTable(r, :)]);
    end
end

function values = decodeCombination(combo, maxValues)
    % 解码组合号得到每个标志位的值
    numFlags = length(maxValues);
    values = zeros(1, numFlags);
    
    for i = numFlags:-1:1
        values(i) = mod(combo, maxValues(i) + 1);
        combo = floor(combo / (maxValues(i) + 1));
    end
end

