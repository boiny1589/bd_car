import re

with open(r'E:\Python_projects\bd_car\requirements_old.txt', 'r', encoding='utf-8') as fin, open(r'E:\Python_projects\bd_car\requirements_new.txt', 'w', encoding='utf-8') as fout:
    for line in fin:
        # 跳过本地路径依赖
        if 'file:///' in line or 'croot' in line or 'build' in line or 'tmp' in line or 'work' in line:
            continue
        # 跳过以 -e git+ 开头的源码依赖（如不需要）
        if line.strip().startswith('-e git+'):
            continue
        # 保留标准包名
        fout.write(line)