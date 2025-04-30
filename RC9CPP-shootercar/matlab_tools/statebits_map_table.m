function statebits_map_table_GUI()
    % 创建图形界面窗口
    fig = uifigure('Name','状态组合及索引表','Position',[100 100 600 400]);
    
    % 创建输入框和标签，输入 maxValues 数组
    lbl = uilabel(fig, 'Position', [20, 350, 100, 22], 'Text', 'Max Values:');
    txt = uieditfield(fig, 'text', 'Position', [120, 350, 200, 22], 'Value', '[1,1,2,3]');
    
    % 先创建 uitable 用于显示结果
    uit = uitable(fig, 'Position', [20, 20, 560, 300], ...
                  'ColumnEditable', true, 'RowName', []);
    
    % 再创建按钮，点击后生成表格
    btn = uibutton(fig, 'push', 'Text', '生成表格', 'Position', [350, 350, 100, 22]);
    btn.ButtonPushedFcn = @(btn,event) generateTable(txt.Value, uit);
end

function generateTable(maxValuesStr, uit)
    % 将字符串转换为数值数组
    maxValues = str2num(maxValuesStr); %#ok<ST2NM>
    if isempty(maxValues)
        uialert(uit.Parent, '无效的输入，请输入类似 [1,1,2,3] 的数组', '输入错误');
        return;
    end

    % 计算标志位数量、位宽和偏移量
    numFlags = length(maxValues);
    bitWidths = arrayfun(@(x) ceil(log2(x+1)), maxValues);
    shifts = [0, cumsum(bitWidths(1:end-1))];
    
    % 计算组合总数
    totalCombinations = prod(maxValues + 1);
    
    % 定义表格列名
    headers = cell(1, numFlags+1);
    for i = 1:numFlags
        headers{i} = ['Flag', num2str(i)];
    end
    headers{numFlags+1} = 'Index';
    
    % 初始化数据存储单元（使用 cell 数组方便混合数据类型显示）
    data = cell(totalCombinations, numFlags+1);
    row = 1;
    for combo = 0:(totalCombinations-1)
        % 解码组合得到每个标志位的值
        flagValues = decodeCombination(combo, maxValues);
        
        % 如果超出最大值则跳过（一般情况不会发生）
        if any(flagValues > maxValues)
            continue;
        end
        
        % 计算唯一索引值
        index = 0;
        for i = 1:numFlags
            index = index + (flagValues(i) * (2^shifts(i)));
        end
        
        % 存入数据单元
        for i = 1:numFlags
            data{row, i} = flagValues(i);
        end
        data{row, numFlags+1} = index;
        row = row + 1;
    end
    
    % 截取有效行
    data = data(1:row-1, :);
    
    % 更新 uitable 数据和列标题
    uit.Data = data;
    uit.ColumnName = headers;
end

function values = decodeCombination(combo, maxValues)
    % 根据组合编号解码得到每个标志位的值
    numFlags = length(maxValues);
    values = zeros(1, numFlags);
    for i = numFlags:-1:1
        values(i) = mod(combo, maxValues(i) + 1);
        combo = floor(combo / (maxValues(i) + 1));
    end
end