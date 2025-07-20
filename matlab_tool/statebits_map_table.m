function statebits_map_table_GUI()
    % 创建图形界面窗口
    fig = uifigure('Name', '状态组合及索引表', 'Position', [100 100 800 600]);
    
    % 创建输入框和标签，输入 maxValues 数组
    lblMaxValues = uilabel(fig, 'Position', [20, 550, 100, 22], 'Text', 'Max Values:');
    txtMaxValues = uieditfield(fig, 'text', 'Position', [120, 550, 200, 22], 'Value', '[1,1,2,3]');
    
    % 创建 uitable 用于显示原始结果
    uitOriginal = uitable(fig, 'Position', [20, 250, 760, 280], ...
                          'ColumnEditable', true, 'RowName', []);
    
    % 创建生成表格按钮
    btnGenerate = uibutton(fig, 'push', 'Text', '生成表格', 'Position', [350, 550, 100, 22]);
    btnGenerate.ButtonPushedFcn = @(btn, event) generateTable(txtMaxValues.Value, uitOriginal);
    
    % 创建导出CSV按钮
    btnExport = uibutton(fig, 'push', 'Text', '导出CSV', 'Position', [460, 550, 100, 22]);
    btnExport.ButtonPushedFcn = @(btn, event) exportToCSV(uitOriginal);
    
    % 创建筛选条件输入框和标签
    lblFilter = uilabel(fig, 'Position', [20, 200, 100, 22], 'Text', '筛选条件:');
    txtFilter = uieditfield(fig, 'text', 'Position', [120, 200, 200, 22], 'Value', '{{5,2},{4,1},{3,0}}');
    
    % 创建筛选按钮
    btnFilter = uibutton(fig, 'push', 'Text', '筛选', 'Position', [350, 200, 100, 22]);
    btnFilter.ButtonPushedFcn = @(btn, event) filterTable(uitOriginal, txtFilter.Value, fig);
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

function exportToCSV(uit)
    % 检查表格是否有数据
    if isempty(uit.Data)
        uialert(uit.Parent, '表格中没有数据，请先生成表格', '导出错误');
        return;
    end
    
    % 弹出保存对话框
    [file, path] = uiputfile('*.csv', '保存CSV文件', 'statebits_map.csv');
    if isequal(file, 0) || isequal(path, 0)
        return; % 用户取消保存
    end
    
    % 获取表格数据和列标题
    data = uit.Data;
    headers = uit.ColumnName;
    
    % 将数据转换为table格式
    if iscell(data)
        T = cell2table(data, 'VariableNames', headers);
    else
        T = data;
        T.Properties.VariableNames = headers;
    end
    
    % 写入CSV文件
    writetable(T, fullfile(path, file));
    
    % 提示导出成功
    uialert(uit.Parent, ['CSV文件已保存至 ', fullfile(path, file)], '导出成功');
end

function filterTable(uitOriginal, filterStr, fig)
    % 解析筛选条件
    try
        filterConditions = eval(filterStr);
        if ~iscell(filterConditions) || isempty(filterConditions)
            uialert(fig, '筛选条件格式错误，请输入类似 {{5,2},{4,1},{3,0}} 的格式', '输入错误');
            return;
        end
    catch ME
        uialert(fig, ['筛选条件解析失败：', ME.message], '输入错误');
        return;
    end
    
    % 获取原始数据和列标题
    data = uitOriginal.Data;
    headers = uitOriginal.ColumnName;
    
    if isempty(data)
        uialert(fig, '原始表格中没有数据，请先生成表格', '筛选错误');
        return;
    end
    
    % 将数据转换为数值矩阵
    if iscell(data)
        dataNum = cell2mat(data);
        if ~isnumeric(dataNum)
            uialert(fig, '数据转换失败，表格数据包含非数值类型', '筛选错误');
            return;
        end
    else
        dataNum = data;
    end
    
    % 获取列数
    numCols = size(dataNum, 2);
    
    % 验证筛选条件中的列索引和值
    for i = 1:length(filterConditions)
        cond = filterConditions{i};
        if ~iscell(cond) || length(cond) ~= 2
            uialert(fig, '筛选条件格式错误，每个条件应为 {col, val} 的形式', '输入错误');
            return;
        end
        col = cond{1};
        val = cond{2};
        if ~isnumeric(col) || ~isscalar(col) || ~isnumeric(val) || ~isscalar(val)
            uialert(fig, '列索引和值必须为单一数值', '输入错误');
            return;
        end
        if col < 1 || col > numCols || col ~= floor(col)
            uialert(fig, ['列索引 ', num2str(col), ' 无效，必须为 1 到 ', num2str(numCols), ' 之间的整数'], '筛选错误');
            return;
        end
    end
    
    % 筛选数据
    mask = true(size(dataNum, 1), 1);
    for i = 1:length(filterConditions)
        cond = filterConditions{i};
        col = cond{1};
        val = cond{2};
        mask = mask & (dataNum(:, col) == val);
    end
    
    % 获取筛选后的数据
    filteredData = dataNum(mask, :);
    
    if isempty(filteredData)
        uialert(fig, '没有找到符合条件的数据', '筛选结果');
        return;
    end
    
    % 创建一个新窗口来显示子表格
    subFig = uifigure('Name', '筛选后的子表格', 'Position', [200, 200, 600, 400]);
    
    % 在新窗口中创建 uitable
    uitFiltered = uitable(subFig, 'Position', [20, 50, 560, 300], ...
                          'ColumnEditable', true, 'RowName', []);
    
    % 设置数据和列标题
    uitFiltered.Data = num2cell(filteredData);
    uitFiltered.ColumnName = headers;
    
    % 添加关闭按钮，使用默认的 pixels 单位
    btnClose = uibutton(subFig, 'push', 'Text', '关闭', 'Position', [500, 10, 80, 22]);
    btnClose.ButtonPushedFcn = @(btn, event) close(subFig);
end


