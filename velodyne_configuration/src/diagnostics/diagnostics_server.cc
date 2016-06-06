//
#include <ros/ros.h>
//
#include "velodyne_configuration/VLP16_DiagnosticsService.h"
#include "velodyne_configuration/VLP16_DiagnosticsRawService.h"
#include <velodyne_tools.h>
//
#include <string>
//#include <math.h>
//
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>


namespace pt = boost::property_tree;


#define hdltop_volts_to_hv(volts)   \
    101.0 * (volts - 5.0)

#define lm20_volts_to_degCel(volts) \
    -1481.96 + sqrt(2.1962E6 + (1.8639 - volts)/3.88E-6)

#define acs17_volts_to_amps(volts)  \
    10.0 * (volts - 2.5);

/**
 * @brief scale_volt_temp
 *  inspired by 'js/diag.js' dump from the VLP-16 webserver
 * @param msg_raw
 * @param msg
 */
void scale_volt_temp(velodyne_configuration::VLP16_DiagnosticsRawMessage    &msg_raw,
                     velodyne_configuration::VLP16_DiagnosticsMessage       &msg)
{
    const float scale_2x_vref = 5.0/4096;

    try
    {
//        volt_temp.top.hv = hdltop_volts_to_hv(volt_temp.top.hv);
//        volt_temp.top.lm20_temp = lm20_volts_to_degCel(volt_temp.top.lm20_temp);
//        volt_temp.top.pwr_5v *= 2.0;
//        volt_temp.top.pwr_5v_raw *= 2.0;
        msg.top_hv = hdltop_volts_to_hv(msg_raw.top_hv * scale_2x_vref);
        msg.top_lm20_temp = lm20_volts_to_degCel(msg_raw.top_lm20_temp * scale_2x_vref);
        msg.top_pwr_5v = msg_raw.top_pwr_5v * 2.0 * scale_2x_vref;
        msg.top_pwr_5v_raw = msg_raw.top_pwr_5v_raw * 2.0 * scale_2x_vref;
        msg.top_ad_temp = msg_raw.top_ad_temp * scale_2x_vref;
        msg.top_pwr_2_5v = msg_raw.top_pwr_2_5v * scale_2x_vref;
        msg.top_pwr_3_3v = msg_raw.top_pwr_3_3v * scale_2x_vref;
        msg.top_pwr_vccint = msg_raw.top_pwr_vccint * scale_2x_vref;

//        volt_temp.bot.i_out = acs17_volts_to_amps(volt_temp.bot.i_out);
//        volt_temp.bot.lm20_temp = lm20_volts_to_degCel(volt_temp.bot.lm20_temp);
//        volt_temp.bot.pwr_5v *= 2.0;
//        volt_temp.bot.pwr_v_in*= 11.0;
        msg.bot_i_out = acs17_volts_to_amps(msg_raw.bot_i_out * scale_2x_vref);
        msg.bot_lm20_temp = lm20_volts_to_degCel(msg_raw.bot_lm20_temp * scale_2x_vref);
        msg.bot_pwr_5v = msg_raw.bot_pwr_5v * 2.0 * scale_2x_vref;
        msg.bot_pwr_v_in = msg_raw.bot_pwr_v_in * 11.0 * scale_2x_vref;
        msg.bot_pwr_2_5v = msg_raw.bot_pwr_2_5v * scale_2x_vref;
        msg.bot_pwr_3_3v = msg_raw.bot_pwr_3_3v * scale_2x_vref;
        msg.bot_pwr_1_25v = msg_raw.bot_pwr_1_25v * scale_2x_vref;
        msg.bot_pwr_1_2v = msg_raw.bot_pwr_1_2v * scale_2x_vref;

        msg.vhv = msg_raw.vhv;
        msg.ixe = msg_raw.ixe;
        msg.adc_nf = msg_raw.adc_nf;
    }
    catch (std::exception const& e)
    {
        ROS_ERROR_STREAM(e.what());
    }
}

/**
 * @brief get_diagnostics_raw
 * @param res
 * @return
 */
bool get_diagnostics_raw(velodyne_configuration::VLP16_DiagnosticsRawServiceResponse &res)
{
    const std::string str_exec_res = velodyne_tools::request_webserver(
                velodyne_tools::str_IP_ADRESS_WEBSERVER_LIDAR,
                velodyne_tools::WebServerCommands::diag,
                velodyne_tools::max_delay_for_cmd
                );
    //
    ROS_INFO_STREAM("response from VLP webserver: " << str_exec_res);

    // ------------------------------------
    // Manual JSON file parsing
    // ------------------------------------
    // urls:
    // - http://zenol.fr/blog/boost-property-tree/en.html
    // - https://gist.github.com/mloskot/1509935
    try
    {
        JSON_INIT(root, str_exec_res);

        JSON_READ_UINT16(root, volt_temp.bot.i_out, res, bot_i_out);
        JSON_READ_UINT16(root, volt_temp.bot.pwr_1_2v, res, bot_pwr_1_2v);
        JSON_READ_UINT16(root, volt_temp.bot.lm20_temp, res, bot_lm20_temp);
        JSON_READ_UINT16(root, volt_temp.bot.pwr_5v, res, bot_pwr_5v);
        JSON_READ_UINT16(root, volt_temp.bot.pwr_2_5v, res, bot_pwr_2_5v);
        JSON_READ_UINT16(root, volt_temp.bot.pwr_3_3v, res, bot_pwr_3_3v);
        JSON_READ_UINT16(root, volt_temp.bot.pwr_v_in, res, bot_pwr_v_in);
        JSON_READ_UINT16(root, volt_temp.bot.pwr_1_25v, res, bot_pwr_1_25v);

        JSON_READ_UINT16(root, volt_temp.top.lm20_temp, res, top_lm20_temp);
        JSON_READ_UINT16(root, volt_temp.top.hv, res, top_hv);
        JSON_READ_UINT16(root, volt_temp.top.ad_temp, res, top_ad_temp);
        JSON_READ_UINT16(root, volt_temp.top.pwr_5v, res, top_pwr_5v);
        JSON_READ_UINT16(root, volt_temp.top.pwr_2_5v, res, top_pwr_2_5v);
        JSON_READ_UINT16(root, volt_temp.top.pwr_3_3v, res, top_pwr_3_3v);
        JSON_READ_UINT16(root, volt_temp.top.pwr_vccint, res, top_pwr_vccint);

        JSON_READ_UINT16(root, vhv, res, vhv);
        JSON_READ_UINT16(root, adc_nf, res, adc_nf);
        JSON_READ_UINT16(root, ixe, res, ixe);
    }
    catch (std::exception const& e)
    {
        ROS_ERROR_STREAM(e.what());
    }
    // ------------------------------------

    return true;
}

/**
 * @brief get_diagnostics
 * @param req
 * @param res
 * @return
 */
bool get_diagnostics(velodyne_configuration::VLP16_DiagnosticsServiceRequest &req,
                     velodyne_configuration::VLP16_DiagnosticsServiceResponse &res)
{
    velodyne_configuration::VLP16_DiagnosticsRawServiceResponse res_raw;
    get_diagnostics_raw(res_raw);

    scale_volt_temp(res_raw.msg, res.msg);

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velodyne_diagnostics_server");
    ros::NodeHandle n("~");

    ros::ServiceServer service = n.advertiseService("get_diagnostics", get_diagnostics);
    ROS_INFO("Ready to get velodyne diagnostics.");
    ros::spin();

    return 0;
}

