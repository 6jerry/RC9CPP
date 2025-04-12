import os
from pathlib import Path

# 定义作者信息
AUTHOR_INFO_C ="""
/*
 * @author loopgad
 * @contact 3280646246@qq.com
 * @license MIT License
 *
 * Copyright (c) 2024 loopgad
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
"""
AUTHOR_INFO_M = """% @author loopgad
% @contact 3280646246@qq.com
% @license MIT License
%
% Copyright (c) 2024 loopgad
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
%
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
%
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.
"""

def add_author_info(root_dir):
    """
    为指定目录下的 .c, .cpp, .h, .m 文件添加作者信息
    :param root_dir: 根目录路径
    """
    extensions = ['.c', '.cpp', '.h', '.m']
    for root, dirs, files in os.walk(root_dir):
        for file in files:
            if any(file.endswith(ext) for ext in extensions):
                file_path = Path(root) / file
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()

                # 检查是否已包含作者信息
                if "@author loopgad" in content:
                    print(f"Skipping {file_path} (already contains author info)")
                    continue

                # 根据文件类型选择合适的作者信息
                if file.endswith('.m'):
                    author_info = AUTHOR_INFO_M
                else:
                    author_info = AUTHOR_INFO_C

                # 在文件开头添加作者信息
                with open(file_path, 'w', encoding='utf-8') as f:
                    f.write(author_info + "\n" + content)
                print(f"Updated {file_path} with author info")

def main():
    # 获取脚本所在目录
    script_dir = Path(__file__).parent.resolve()
    root_dir = script_dir  # 设置根目录为脚本所在目录

    # 为代码文件添加作者信息
    add_author_info(root_dir)
    print("Author information added to code files.")

if __name__ == "__main__":
    main()