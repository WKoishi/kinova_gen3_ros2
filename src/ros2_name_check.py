import os
from time import sleep

PATH_SPILT = '/'
FILE_TYPE = '.srv'

def msg_name_format_check(name):

    change_flag = False
    need_upper_flag = False
    new_name_ls = []
    new_name = ''

    if name[0].islower():
        change_flag = True
        need_upper_flag = True
    
    for ch in name:
        if ch == '_':
            change_flag = True
            need_upper_flag = True
        else:
            if need_upper_flag:
                need_upper_flag = False
                new_name_ls.append(ch.upper())
            else:
                new_name_ls.append(ch)

    if change_flag:
        new_name = ''.join(new_name_ls)
        #print('changed:', name)

    return change_flag, name, new_name


if __name__ == "__main__":

    target_directory = "kortex_driver/srv"

    changed_msg_origin_name_list = []

    print("--- step 1: check the file name ---")

    for root, dirs, files in os.walk(target_directory):
        for file in files:
            file_path = os.path.join(root, file)
            file_name_with_path, type = os.path.splitext(file_path)
            if type == FILE_TYPE:
                #print("found: ", file_name_with_path)
                file_path_struct = file_name_with_path.split(PATH_SPILT)
                file_name = file_path_struct[-1]

                flag, old_name, new_name = msg_name_format_check(file_name)
                if flag:
                    changed_msg_origin_name_list.append(file_name)
                    file_path_struct[-1] = new_name
                    new_file_path = ''.join([PATH_SPILT.join(file_path_struct), type])
                    os.rename(file_path, new_file_path)
                    print(file_name, "-->", new_name)

    print("--- step 2: check the variables in each file ---")

    var_changed_flag = False

    for root, dirs, files in os.walk(target_directory):
        for file in files:
            file_path = os.path.join(root, file)
            file_name_with_path, type = os.path.splitext(file_path)
            if type == FILE_TYPE:
                with open(file_path, 'r+', encoding='utf-8') as file_obj:
                    f_contents = file_obj.read()
                    f_data = f_contents.split('\n')
                    for i, lines in enumerate(f_data):
                        f_data[i] = lines.split(' ')
                    
                    for i, f_line in enumerate(f_data):
                        if f_line[0]:
                            var_name_struct = f_line[0].split('[')
                            if var_name_struct[0] in changed_msg_origin_name_list:
                                flag, old_name, new_name = msg_name_format_check(f_line[0])
                                if flag:
                                    var_changed_flag = True
                                    f_contents = f_contents.replace(f_line[0], new_name)
                                    print(f_line[0], "-->", new_name, "in file", file_path)
                                    pass

                    if var_changed_flag:
                        var_changed_flag = False
                        file_obj.seek(0, 0)
                        file_obj.truncate()
                        ret = file_obj.write(f_contents)
                        if ret != len(f_contents):
                            print(file_path, "write error!")
                        pass

                    sleep(0.01)
                        

