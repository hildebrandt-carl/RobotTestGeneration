# prints recursive count of lines of python source code from current directory
# includes an ignore_list. also prints total sloc

import os
cur_path = os.getcwd()
ignore_set = set(["__init__.py", "count_sourcelines.py"])

loc_list = []

for py_dir, _, files in os.walk(cur_path):
    for files in files:
        print(files)
        if (files.endswith(".py") or files.endswith(".sh")) and files not in ignore_set:
            total_path = os.path.join(py_dir, files)
            non_blank_count = sum([1 for i in open(total_path,"r").readlines() if i.strip()])
            lines_of_code = (non_blank_count)
            loc_list.append((lines_of_code, total_path.split(cur_path)[1]))

for line_number_count, filename in loc_list: 
    print("%05d lines in %s" % (line_number_count, filename))

print("\nTotal: %s lines (%s)" %(sum([x[0] for x in loc_list]), cur_path))