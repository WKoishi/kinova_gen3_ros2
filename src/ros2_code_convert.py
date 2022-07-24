import os
import re
from time import sleep
from ros2_name_check import msg_name_format_check

PATH_SPLIT = '/'

def header_name_convert(name):

    new_name = []
    change_flag = False
    name_len = len(name)

    if not name.islower():
        change_flag = True
        for i, ch in enumerate(name):
            if i > 0 and ch.isalpha() and ch.isupper():
                if i >= name_len - 1:
                    new_name.append('_')
                elif (name[i+1].islower() or name[i-1].islower()) and name[i-1] != '_':
                    new_name.append('_')
            new_name.append(ch.lower())

    new_name = ''.join(new_name)

    return change_flag, new_name

if __name__ == "__main__":

    ros2_header_list = {
        "msg": [],
        "srv": []
    }

    ros2_msg_srv_list = {
        "msg": [],
        "srv": []
    }

    header_change_count = 0
    type_define_change_count = 0

    ros2_header_dir = "install/kortex_driver/include/kortex_driver/kortex_driver"
    ros2_msg_dir = "src/kortex_driver/msg"
    ros2_srv_dir = "src/kortex_driver/srv"
    target_directory = "src/kortex_driver/include/kortex_driver"

    for root, dirs, files in os.walk(ros2_header_dir):
        root_struct = root.split(PATH_SPLIT)
        if root_struct[-1] == "msg" or root_struct[-1] == "srv":
            for file in files:
                file_name, file_type = os.path.splitext(os.path.join(root, file))
                if file_type == ".hpp":
                    file_name_struct = file_name.split(PATH_SPLIT)
                    ros2_header_list[root_struct[-1]].append(file_name_struct[-1])

    if not ros2_header_list["msg"]:
        exit("error1")

    for root, dirs, files in os.walk(ros2_msg_dir):
        for file in files:
            file_name, file_type = os.path.splitext(os.path.join(root, file))
            if file_type == ".msg":
                file_name_struct = file_name.split(PATH_SPLIT)
                ros2_msg_srv_list["msg"].append(file_name_struct[-1])

    if not ros2_msg_srv_list["msg"]:
        exit("error2")

    for root, dirs, files in os.walk(ros2_srv_dir):
        for file in files:
            file_name, file_type = os.path.splitext(os.path.join(root, file))
            if file_type == ".srv":
                file_name_struct = file_name.split(PATH_SPLIT)
                ros2_msg_srv_list["srv"].append(file_name_struct[-1])

    if not ros2_msg_srv_list["srv"]:
        exit("error3")


    define_pattern = re.compile(r"kortex_driver::\S{1,}")

    file_change_flag = False

    for root, dirs, files in os.walk(target_directory):
        for file in files:
            file_path = os.path.join(root, file)
            file_name, file_type = os.path.splitext(file_path)
            if file_type == ".h":
                with open(file_path, 'r+', encoding='utf-8') as file_obj:
                    f_contents = file_obj.read()
                    f_data = f_contents.split('\n')
                    for line in f_data:
                        if line:
                            line_struct = line.split(' ')
                            if line_struct[0] == "#include":
                                include_struct = line_struct[1].split(PATH_SPLIT)
                                if include_struct[0] == "\"kortex_driver" and len(include_struct) <= 2:
                                    flag, new_name = header_name_convert(include_struct[-1])
                                    if flag:
                                        new_name = new_name.split('.')[0]

                                        if new_name in ros2_header_list["msg"]:
                                            file_change_flag = True
                                            new_name = "msg" + PATH_SPLIT + new_name + ".hpp\""

                                        elif new_name in ros2_header_list["srv"]:
                                            file_change_flag = True
                                            new_name = "srv" + PATH_SPLIT + new_name + ".hpp\""

                                        else:
                                            print("[WARNING]", file_path, new_name, "is not in include files")
                                            continue

                                        new_line = line.replace(include_struct[-1], new_name)
                                        f_contents = f_contents.replace(line, new_line)
                                        print(include_struct[-1], "->", new_name, "in file", file)
                                        sleep(0.01)
                                        header_change_count += 1

                                        pass

                            else:
                                type_define_list = define_pattern.findall(line)
                                if type_define_list:
                                    new_line = line  # copy a str (it's ok!)
                                    for type_define in type_define_list:
                                        type_define_struct = type_define.split("::")
                                        flag, _, new_name = msg_name_format_check(type_define_struct[1])

                                        if new_name in ros2_msg_srv_list["msg"]:
                                            file_change_flag = True
                                            new_name = "msg::" + new_name

                                        elif new_name in ros2_msg_srv_list["srv"]:
                                            file_change_flag = True
                                            new_name = "srv::" + new_name

                                        else:
                                            print("[WARNING]", file_path, new_name, "is not in msg\srv files")
                                            continue
                                        
                                        new_type_define = type_define.replace(type_define_struct[1], new_name)
                                        new_line = new_line.replace(type_define, new_type_define)
                                        print(type_define_struct[1], "->", new_name, "in file", file)
                                        sleep(0.01)
                                        
                                    f_contents = f_contents.replace(line, new_line)
                                    type_define_change_count += 1

                                
                    if file_change_flag:
                        file_change_flag = False
                        file_obj.seek(0, 0)
                        file_obj.truncate()
                        ret = file_obj.write(f_contents)
                        if ret != len(f_contents):
                            print("[WRITE ERROR]", file_path)
                            exit("[WRITE ERROR]", file_path)

            sleep(0.01)
     

    print("header_change_count:", header_change_count)
    print("type_define_change_count:", type_define_change_count)

    pass



