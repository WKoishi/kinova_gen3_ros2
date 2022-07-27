import os
from time import sleep
from ros2_name_check import msg_name_format_check

PATH_SPLIT = '/'

if __name__ == "__main__":

    ros2_msg_srv_list = {
        "msg": [],
        "srv": []
    }

    MSG_OR_SRV = "srv"

    ros2_srv_dir = "src/kortex_driver/srv"
    target_directory = "src/kortex_driver/include/kortex_driver/generated/interfaces"

    for root, dirs, files in os.walk(ros2_srv_dir):
        for file in files:
            file_name, file_type = os.path.splitext(os.path.join(root, file))
            if file_type == ".srv":
                file_name_struct = file_name.split(PATH_SPLIT)
                ros2_msg_srv_list["srv"].append(file_name_struct[-1])

    if not ros2_msg_srv_list["srv"]:
        exit("error4")

    file_change_flag = False

    for root, dirs, files in os.walk(target_directory):
        for file in files:
            file_path = os.path.join(root, file)
            file_path_struct = file_path.split(PATH_SPLIT)
            file_name, file_type = os.path.splitext(file_path)
            if file_type == ".h":
                with open(file_path, 'r+', encoding='utf-8') as file_obj:
                    f_contents = file_obj.read()
                    f_data = f_contents.split('\n')
                    for line in f_data:
                        if line:
                            line_temp = line.strip('\t')
                            line_temp = line_temp.strip(' ')
                            line_temp = line_temp.strip(';')

                            line_struct = line_temp.split(' ')
                            if line_struct[0] == "ros::ServiceServer" and len(line_struct) == 2:
                                template_detect = line_struct[1].replace("m_service", '')
                                _, _, template_detect = msg_name_format_check(template_detect)

                                if template_detect in ros2_msg_srv_list[MSG_OR_SRV]:
                                    file_change_flag = True
                                    template_add = "rclcpp::Service<kortex_driver::" + MSG_OR_SRV + "::" + template_detect + ">::SharedPtr"

                                    new_line = line.replace(line_struct[0], template_add)
                                    f_contents = f_contents.replace(line, new_line)
                                    print(line_struct[0], "->", template_add, file_path_struct[-1])

                                else:
                                    print("[WARNING]", file_path, template_detect, "not found!")

                    if file_change_flag:
                        file_change_flag = False
                        file_obj.seek(0, 0)
                        file_obj.truncate()
                        ret = file_obj.write(f_contents)
                        if ret != len(f_contents):
                            print("[WRITE ERROR]", file_path)
                            exit("[WRITE ERROR]", file_path)

                sleep(0.02)



