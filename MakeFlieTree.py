import os
from pathlib import Path

def generate_directory_tree(root_dir, exclude_dirs=None, exclude_files=None):
    """
    生成目录树结构
    :param root_dir: 根目录路径
    :param exclude_dirs: 需要排除的目录列表
    :param exclude_files: 需要排除的文件列表
    :return: 目录树字符串
    """
    if exclude_dirs is None:
        exclude_dirs = ['.git', '__pycache__', 'venv', 'node_modules']
    if exclude_files is None:
        exclude_files = ['.gitignore', '.DS_Store']

    tree = []
    root = Path(root_dir)

    def walk(dir_path, prefix=''):
        """递归遍历目录"""
        try:
            entries = sorted(os.listdir(dir_path), key=lambda x: (os.path.isfile(os.path.join(dir_path, x)), x))
        except PermissionError:
            return

        for index, entry in enumerate(entries):
            # 过滤掉以 '.' 开头的目录和文件
            if entry.startswith('.'):
                continue

            # 过滤掉非 ASCII 字符
            entry = ''.join(c if ord(c) < 128 else f'\\u{ord(c):04x}' for c in entry)

            if os.path.isdir(os.path.join(dir_path, entry)):
                if entry in exclude_dirs:
                    continue
                tree.append(f"{prefix}+ {entry}/")
                if index == len(entries) - 1:
                    extension = '    '
                else:
                    extension = '    '
                walk(os.path.join(dir_path, entry), prefix + extension)
            elif os.path.isfile(os.path.join(dir_path, entry)) and entry not in exclude_files:
                tree.append(f"{prefix}- {entry}")

    tree.append(f"+ {root.name}/")
    walk(root)
    return '\n'.join(tree)


def create_readme(tree_content, output_file='README.md', template=None):
    """
    创建规范的 README 文档
    :param tree_content: 目录树内容
    :param output_file: 输出文件名
    :param template: 自定义 README 模板字符串
    """
    if template is None:
        template = f"""# Project Directory Structure

## Project Overview
This project is designed to demonstrate the directory structure and organization of a typical software project.

## Directory Tree
```plaintext
{tree_content}
```
## Usage
To use this project, simply clone the repository and navigate to the root directory.

## License
This project is licensed under the MIT License.
"""
    with open(output_file, 'w') as f:
        f.write(template)


def main():
    # 获取脚本所在目录
    script_dir = Path(__file__).parent.resolve()
    root_dir = script_dir  # 设置根目录为脚本所在目录
    output_file = script_dir / 'README.md'  # 输出文件保存在脚本所在目录

    # 生成目录树
    tree_content = generate_directory_tree(root_dir)

    # 创建 README 文件
    create_readme(tree_content, output_file=output_file)
    print(f"README file created at {output_file}")


if __name__ == "__main__":
    main()