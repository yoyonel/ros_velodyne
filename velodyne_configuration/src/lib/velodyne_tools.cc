#include <ros/ros.h>
#include <velodyne_tools.h>
#include <string>

namespace velodyne_tools
{
    /**
     * @brief exec
     * @param cmd
     * @return
     */
    std::string exec_cmd(const char* cmd) {
        FILE* pipe = popen(cmd, "r");
        if (!pipe) return "ERROR";
        char buffer[128];
        std::string result = "";
        while(!feof(pipe)) {
            if(fgets(buffer, 128, pipe) != NULL)
                result += buffer;
        }
        pclose(pipe);
        return result;
    }

    std::string request_webserver(
            std::string _str_ip_laser,
            WebServerCommands _cmd,
            const float& _max_delay_for_cmd
            )
    {
        // url: http://stackoverflow.com/questions/23936246/error-invalid-operands-of-types-const-char-35-and-const-char-2-to-binar
        std::string cmd_curl = "curl -s http://" + _str_ip_laser + "/cgi/" + _cmd._to_string() + ".json";
        std::string cmd = cmd_curl;
        if(_max_delay_for_cmd != 0) {
            cmd = "timeout " + std::to_string(_max_delay_for_cmd) + "s " + cmd;
        }
        //
        ROS_INFO_STREAM("Commande bash: " << cmd);

        return exec_cmd(cmd.c_str());
    }
} // namespace velodyne_settings
