
#include "common_inc.h"

/**
 * @file cmd_protocol.cpp
 * @brief 命令协议定义。
 *
 * 此文件定义了与 "reftool" 交互的协议对象树。
 * 它暴露了只读变量、辅助函数和机器人对象供外部工具访问。
 */

/*----------------- 1.Add Your Extern Variables Here (Optional) ------------------*/
extern DummyRobot dummy;

/**
 * @brief 辅助函数类。
 *
 * 包含用于协议交互的静态辅助函数，例如获取芯片温度。
 */
class HelperFunctions
{
public:
    /*--------------- 2.Add Your Helper Functions Helper Here (optional) ----------------*/
    /**
     * @brief 获取芯片温度的辅助函数。
     *
     * @return 芯片温度（摄氏度）。
     */
    float GetTemperatureHelper()
    { return AdcGetChipTemperature(); }

} staticFunctions;


// Define options that intractable with "reftool".
/**
 * @brief 构建对象树。
 *
 * 定义可通过协议访问的成员列表，包括序列号、温度获取函数和机器人对象。
 *
 * @return 协议成员列表。
 */
static inline auto MakeObjTree()
{
    /*--------------- 3.Add Your Protocol Variables & Functions Here ----------------*/
    return make_protocol_member_list(
        // Add Read-Only Variables
        make_protocol_ro_property("serial_number", &serialNumber),
        make_protocol_function("get_temperature", staticFunctions, &HelperFunctions::GetTemperatureHelper),
        make_protocol_object("robot", dummy.MakeProtocolDefinitions())
    );
}


COMMIT_PROTOCOL
