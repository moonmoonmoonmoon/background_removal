import os
import argparse
from pathlib import Path


def generate_tree(
        directory: str,
        ignore_dirs: list = None,
        ignore_files: list = None,
        max_depth: int = None
) -> str:
    """
    生成目录树结构的字符串表示

    参数:
        directory: 要遍历的目录路径
        ignore_dirs: 要忽略的目录名列表
        ignore_files: 要忽略的文件名列表
        max_depth: 最大遍历深度
    """
    if ignore_dirs is None:
        ignore_dirs = ['.git', '__pycache__', 'node_modules', '.idea', 'venv','dataset','output','backup','results','logs']
    if ignore_files is None:
        ignore_files = ['.gitignore', '.DS_Store']

    output = []
    root_dir = Path(directory)

    def should_ignore(path: Path) -> bool:
        return (path.name.startswith('.') or
                (path.is_dir() and path.name in ignore_dirs) or
                (path.is_file() and path.name in ignore_files))

    def add_to_tree(path: Path, prefix: str = '', depth: int = 0):
        if max_depth is not None and depth > max_depth:
            return

        if should_ignore(path):
            return

        output.append(f'{prefix}{"└──" if "└──" in prefix else "├──"} {path.name}')

        if path.is_dir():
            # 获取目录下的所有内容并排序
            contents = sorted(list(path.iterdir()), key=lambda x: (x.is_file(), x.name))
            # 过滤掉需要忽略的项
            contents = [x for x in contents if not should_ignore(x)]

            for i, item in enumerate(contents):
                # 确定是否是最后一个项目
                is_last = (i == len(contents) - 1)
                # 根据是否是最后一个项目选择不同的前缀
                new_prefix = prefix + ('    ' if '└──' in prefix else '│   ')
                add_to_tree(item, new_prefix, depth + 1)

    output.append(root_dir.name)
    # 处理根目录下的内容
    contents = sorted(list(root_dir.iterdir()), key=lambda x: (x.is_file(), x.name))
    contents = [x for x in contents if not should_ignore(x)]

    for i, path in enumerate(contents):
        is_last = (i == len(contents) - 1)
        prefix = '└──' if is_last else '├──'
        add_to_tree(path, prefix)

    return '\n'.join(output)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='生成目录树结构')
    parser.add_argument('path', nargs='?', default='/home/max/PycharmProjects/background removal', help='要生成树结构的目录路径')
    parser.add_argument('--max-depth', type=int, default=3, help='最大遍历深度')
    args = parser.parse_args()

    tree = generate_tree(args.path, max_depth=args.max_depth)
    print(tree)