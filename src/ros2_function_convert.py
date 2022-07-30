import os
import re
from time import sleep

PATH_SPLIT = '/'

if __name__ == "__main__":

    file_change_flag = False

    bracket_pattern = re.compile(r"(?<=\()[^}]*(?=\))")

    target_directory = "src/kortex_driver/src"

    # for root, dirs, files in os.walk(target_directory):
    #     for file in files:
    #         file_path = os.path.join(root, file)
    #         file_path_struct = file_path.split(PATH_SPLIT)
    #         file_name, file_type = os.path.splitext(file_path)
    #         if file_type == ".cpp" and "services" in file_path_struct[-1]:
    #             with open(file_path, 'r+', encoding='utf-8') as file_obj:
    #                 f_contents = file_obj.read()
    #                 f_data = f_contents.split('\n')
    #                 for line in f_data:
    #                     if line:
    #                         line_struct = line.split('=')
    #                         if len(line_struct) == 2:
    #                             if "m_service" in line_struct[0] and "->create_service<" in line_struct[1]:
    #                                 bracket_list = bracket_pattern.findall(line_struct[1])
    #                                 if len(bracket_list) == 1:
    #                                     parameter_list = bracket_list[0].split(',')
    #                                     if len(parameter_list) == 2:
    #                                         file_change_flag = True
    #                                         callback_function = parameter_list[1].strip(' ')
    #                                         new_parameter = "std::bind(" + callback_function + ", this, std::placeholders::_1, std::placeholders::_2)"

    #                                         new_line = line.replace(callback_function, new_parameter)
    #                                         f_contents = f_contents.replace(line, new_line)
    #                                         print(new_parameter, file_path_struct[-1])

    #                                     else:
    #                                         print("[WARNING]", file_path)

    #                 if file_change_flag:
    #                     file_change_flag = False
    #                     file_obj.seek(0, 0)
    #                     file_obj.truncate()
    #                     ret = file_obj.write(f_contents)
    #                     if ret != len(f_contents):
    #                         print("[WRITE ERROR]", file_path)
    #                         exit("[WRITE ERROR]", file_path)

    #             sleep(0.04)


    for root, dirs, files in os.walk(target_directory):
        for file in files:
            file_path = os.path.join(root, file)
            file_path_struct = file_path.split(PATH_SPLIT)
            file_name, file_type = os.path.splitext(file_path)
            if file_type == ".cpp" and "services" in file_path_struct[-1]:
                with open(file_path, 'r+', encoding='utf-8') as file_obj:
                    f_contents = file_obj.read()
                    f_data = f_contents.split('\n')
                    for line in f_data:
                        if line:
                            bracket_list = bracket_pattern.findall(line)
                            if len(bracket_list) == 1:
                                parameter_list = re.split(r'[;,\s]\s*', bracket_list[0])
                                if len(parameter_list) == 4:
                                    if "kortex_driver::srv::" in parameter_list[0] and "kortex_driver::srv::" in parameter_list[2]:
                                        file_change_flag = True

                                        new_bracket = "const std::shared_ptr<" + parameter_list[0] + "> "
                                        new_bracket += parameter_list[1].strip('&') + ", "
                                        new_bracket += "std::shared_ptr<" + parameter_list[2] + "> "
                                        new_bracket += parameter_list[3].strip('&')

                                        new_line = line.replace(bracket_list[0], new_bracket)
                                        f_contents = f_contents.replace(line, new_line)
                                        print(new_bracket, file_path_struct[-1])

                    if file_change_flag:
                        file_change_flag = False
                        file_obj.seek(0, 0)
                        file_obj.truncate()
                        ret = file_obj.write(f_contents)
                        if ret != len(f_contents):
                            print("[WRITE ERROR]", file_path)
                            exit("[WRITE ERROR]", file_path)

                sleep(0.04)
                                

                                    

                                            



