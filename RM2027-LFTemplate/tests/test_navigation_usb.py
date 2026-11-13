from pathlib import Path
import subprocess
root=Path(__file__).resolve().parents[1]
out=root/"build/navigation_host_tests"
out.mkdir(parents=True,exist_ok=True)

def function(source, name):
    start=source.index(name)
    opening=source.index('{',start)
    depth=1; end=opening+1
    while depth:
        depth += (source[end]=='{')-(source[end]=='}'); end+=1
    return source[start:end]

usb=(root/'RM2027-LFCore/2_Device/USB/usb.cpp').read_text(encoding='utf-8')
cdc=(root/'USB_DEVICE/App/usbd_cdc_if.c').read_text(encoding='utf-8')
rc=(root/'RM2027-LFCore/2_Device/RC/remote_control.h').read_text(encoding='utf-8')
rc=rc[rc.index('typedef struct RcControlData'):rc.index('} __attribute__((packed)) RcControl;')+len('} __attribute__((packed)) RcControl;')]
(out/'remote_control.h').write_text('#pragma once\n#include <stdint.h>\n'+rc)
(out/'struct_typedef.h').write_text('#include <stdint.h>\n')
(out/'attribute_typedef.h').write_text('#define __packed__ __attribute__((packed))\n')
(out/'main.h').write_text('#include <stdint.h>\n')
prefix=r"""
#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>
#include <cstddef>
#include "usb_typdef.h"
#include "chassis_config.hpp"
#include "gimbal_config.h"
using std::isfinite;
#include "lf_core.hpp"
#include "CRC8_CRC16.h"
constexpr int USBD_OK=0, USBD_FAIL=2, USBD_STATE_CONFIGURED=3;
struct Device { int dev_state=USBD_STATE_CONFIGURED; void* pClassData=(void*)1; } hUsbDeviceHS;
uint8_t UserRxBufferHS[512];
volatile uint32_t rx_pending_length=0;
uint32_t rx_pending_offset=0, rearmed=0, mask=0;
uint32_t __get_PRIMASK() {return mask;}
void __disable_irq(){mask=1;}
void __set_PRIMASK(uint32_t value){mask=value;}
int USBD_CDC_SetRxBuffer(Device*,uint8_t* buffer){assert(buffer==UserRxBufferHS);return 0;}
int USBD_CDC_ReceivePacket(Device*){++rearmed;return 0;}
uint32_t tick=1234;
uint32_t HAL_GetTick(){return tick;}
namespace LuojiaFox {
SystemStateSnapshot snapshot{};
SystemStateStore system_state;
unsigned robot_cmd_updates=0;
SystemStateSnapshot SystemStateStore::read() const { return snapshot; }
void SystemStateStore::updateRobotCmd(const RobotCmdState& cmd){snapshot.robot_cmd=cmd;++robot_cmd_updates;}
}
constexpr unsigned HEADER_SIZE=4;
enum ChassisModeTypedef { CHASSIS_MODE_DISABLED, CHASSIS_MODE_DIRECT_RC, CHASSIS_MODE_AUTO_VISION };
ChassisModeTypedef chassis_mode=CHASSIS_MODE_AUTO_VISION;
ChassisModeTypedef Chassis_GetMode(){return chassis_mode;}
struct Power {
    uint16_t power_limit_w=80, buffer_energy_j=47;
    uint32_t power_tick=0, buffer_tick=0;
    bool power_received=false, buffer_received=false, online=false;
};
struct Referee {
    Power power;
    Power GetPowerSupplySnapshot(){return power;}
} referee;
namespace board_link {
namespace can3 { constexpr uint32_t kChassisFeedbackTimeoutMs=50; }
struct BigYawFeedback {
    float joint_rad=0;
    bool motor_online=false, ready=false, slow=false;
    uint32_t revision=0, rx_ms=0;
} feedback;
bool getPeerBigYawFeedback(BigYawFeedback& value){value=feedback;return value.motor_online;}
}
std::vector<uint8_t> tx;
uint8_t USB_Transmit(uint8_t* data,uint16_t size){tx.assign(data,data+size);return USBD_OK;}
"""
# Select the callback implementation, after its declaration.
callback=cdc.index('static int8_t CDC_Receive_HS(uint8_t* Buf, uint32_t *Len)')
prefix+=function(cdc[callback:],'static int8_t CDC_Receive_HS')+'\n'
prefix+=function(cdc,'int8_t USB_Receive(uint8_t* Buf, uint32_t *Len)')+'\n'
prefix+=r"""
struct Usb {
    uint8_t rx_buf[256]{}; uint32_t rx_size=0;
    struct {uint32_t NavigationStatus=0;} last_send_time;
    ReceiveDataRobotCmd_s receive_robot_cmd{};
    ReceiveDataPidDebug_s receive_pid_debug{};
    ReceiveDataVirtualRc_s receive_virtual_rc{};
    ReceiveDataSentryCmd_s receive_sentry_cmd{};
    uint32_t latest_rx_timestamp=0;
    uint32_t receive_time=0,robot_cmd_receive_time=0;
    struct {
        struct {float vx,vy,wz;} speed_vector;
        struct {float roll,pitch,yaw,leg_length;} chassis;
        struct {float pitch,yaw;} gimbal;
        struct {bool fire,fric_on;} shoot;
    } robot_cmd{};
    SendDataNavigationStatus_s send_navigation_status{};
    SendDataRobotMotion_s send_robot_motion{};
    SendDataJointState_s send_joint_state{};
    void Receive();
    bool AcceptRobotCmd(const ReceiveDataRobotCmd_s&);
    void UpdateRobotCmd();
    void SendNavigationStatus();
    void SendRobotMotion();
    void SendJointState();
};
"""
for name in ["bool Usb::AcceptRobotCmd", "void Usb::UpdateRobotCmd", "void Usb::Receive",
             "void Usb::SendNavigationStatus", "void Usb::SendRobotMotion", "void Usb::SendJointState"]:
    prefix+=function(usb,name)+'\n'
tests=r"""
using Bytes=std::vector<uint8_t>;
Bytes encode(ReceiveDataRobotCmd_s c){
    c.frame_header={RECEIVE_SOF,sizeof(c)-6,ROBOT_CMD_DATA_RECEIVE_ID,0};
    Bytes v(sizeof(c));memcpy(v.data(),&c,sizeof(c));
    append_CRC8_check_sum(v.data(),4);append_CRC16_check_sum(v.data(),v.size());return v;
}
ReceiveDataRobotCmd_s command(uint32_t timestamp,uint8_t mode=0,uint8_t flags=0,uint32_t session=1){
    ReceiveDataRobotCmd_s c{};
    c.time_stamp=timestamp;c.data.navigation={mode,flags,session,1.5f};return c;
}
void feed(Usb& u,const Bytes& v){
    assert(v.size()<=sizeof(UserRxBufferHS));assert(rx_pending_length==0);
    memcpy(UserRxBufferHS,v.data(),v.size());uint32_t len=v.size();
    const unsigned before=rearmed;
    CDC_Receive_HS(UserRxBufferHS,&len);
    if(len)assert(rearmed==before);
    do{u.Receive();assert(mask==0);}while(rx_pending_length);
    assert(rearmed==before+1);
}
void reset(){
    LuojiaFox::snapshot={};LuojiaFox::robot_cmd_updates=0;
}
void commandTests(){
    using namespace LuojiaFox;
    reset();Usb u;
    auto fold=command(2,ROBOT_CMD_NAV_FOLD,
        ROBOT_CMD_CHASSIS_ENABLED|ROBOT_CMD_HEADING_VALID|ROBOT_CMD_AIM_VALID);
    fold.data.speed_vector={0.3f,-0.4f,0.5f};
    fold.data.gimbal={-0.2f,0.8f};fold.data.shoot.fire=1;
    feed(u,encode(fold));assert(robot_cmd_updates==0);
    feed(u,encode(command(1)));assert(robot_cmd_updates==1);
    auto invalid=fold;invalid.data.navigation.flags &= ~ROBOT_CMD_HEADING_VALID;
    feed(u,encode(invalid));assert(robot_cmd_updates==1);
    feed(u,encode(fold));
    assert(robot_cmd_updates==2&&snapshot.robot_cmd.chassis_enabled&&snapshot.robot_cmd.passage_requested);
    assert(snapshot.robot_cmd.vx==.3f&&snapshot.robot_cmd.vy==-.4f&&snapshot.robot_cmd.wz==.5f);
    assert(snapshot.robot_cmd.heading_valid&&snapshot.robot_cmd.big_yaw_heading_rad==1.5f);
    assert(snapshot.robot_cmd.gimbal_pitch==-.2f&&snapshot.robot_cmd.gimbal_yaw==.8f&&snapshot.robot_cmd.fire);
    assert(snapshot.robot_cmd.gimbal_aim_changed_ms==tick);
    feed(u,encode(command(2,ROBOT_CMD_NAV_UPRIGHT,ROBOT_CMD_CHASSIS_ENABLED)));
    assert(robot_cmd_updates==2&&snapshot.robot_cmd.passage_requested);
    tick+=500;
    u.Receive();assert(snapshot.robot_cmd.passage_requested&&tick-snapshot.robot_cmd.last_update_ms==500);
    feed(u,encode(command(3,ROBOT_CMD_NAV_NORMAL,ROBOT_CMD_CHASSIS_ENABLED)));
    assert(snapshot.robot_cmd.passage_requested&&snapshot.robot_cmd.gimbal_aim_changed_ms==0U);
    feed(u,encode(command(4,ROBOT_CMD_NAV_UPRIGHT,0)));
    assert(snapshot.robot_cmd.passage_requested&&!snapshot.robot_cmd.chassis_enabled);
    feed(u,encode(command(1,ROBOT_CMD_NAV_UPRIGHT,ROBOT_CMD_CHASSIS_ENABLED,2)));
    assert(u.receive_robot_cmd.data.navigation.session==1);
    auto non_neutral=command(1,0,0,2);non_neutral.data.shoot.fire=1;
    feed(u,encode(non_neutral));assert(u.receive_robot_cmd.data.navigation.session==1);
    feed(u,encode(command(1,0,0,2)));
    assert(u.receive_robot_cmd.data.navigation.session==2&&snapshot.robot_cmd.passage_requested);
    feed(u,encode(command(2,ROBOT_CMD_NAV_UPRIGHT,ROBOT_CMD_CHASSIS_ENABLED,2)));
    assert(!snapshot.robot_cmd.passage_requested&&snapshot.robot_cmd.chassis_enabled);
    auto bad=command(3,0,ROBOT_CMD_CHASSIS_ENABLED,2);
    bad.data.speed_vector.vx=std::numeric_limits<float>::quiet_NaN();
    feed(u,encode(bad));assert(u.receive_robot_cmd.time_stamp==2);
    bad=command(3,0,ROBOT_CMD_AIM_VALID,2);
    bad.data.gimbal.pitch=std::numeric_limits<float>::infinity();
    feed(u,encode(bad));assert(u.receive_robot_cmd.time_stamp==2);
    puts("Unified RobotCmd: shared speed, aim and heading, session, replay, fold/restore latch, invalid values passed");
}
Bytes obsolete(uint8_t id,unsigned size){
    Bytes v(size,0);v[0]=RECEIVE_SOF;v[1]=size-6;v[2]=id;
    append_CRC8_check_sum(v.data(),4);append_CRC16_check_sum(v.data(),v.size());return v;
}
void receiveTests(){
    using namespace LuojiaFox;
    static_assert(sizeof(ReceiveDataRobotCmd_s)==58,"actual wire layout");
    const auto frame=encode(command(1));
    for(size_t split=0;split<=frame.size();++split){
        reset();Usb receiver;feed(receiver,Bytes(frame.begin(),frame.begin()+split));
        if(split<frame.size())assert(robot_cmd_updates==0);
        feed(receiver,Bytes(frame.begin()+split,frame.end()));
        assert(robot_cmd_updates==1&&receiver.rx_size==0);
        receiver.Receive();assert(robot_cmd_updates==1);
    }
    reset();Usb bytewise;for(auto b:frame)feed(bytewise,Bytes{b});assert(robot_cmd_updates==1);
    reset();Usb u;
    feed(u,obsolete(0x20,34));feed(u,obsolete(0x01,48));feed(u,obsolete(0x01,49));
    assert(robot_cmd_updates==0);
    Bytes joined;
    for(int i=1;i<=8;++i){auto v=encode(command(i));joined.insert(joined.end(),v.begin(),v.end());}
    feed(u,joined);assert(robot_cmd_updates==8&&u.rx_size==0);
    feed(u,encode(command(8)));assert(robot_cmd_updates==8);
    feed(u,encode(command(9,5)));assert(robot_cmd_updates==8);
    auto bad=encode(command(9));bad[20]^=1;feed(u,bad);assert(robot_cmd_updates==8);
    Bytes garbage{0,0x5A,0,0x11,0x33,0x5A,255,1,0};
    append_CRC8_check_sum(garbage.data()+5,4);
    auto good=encode(command(10));garbage.insert(garbage.end(),good.begin(),good.end());
    feed(u,garbage);assert(robot_cmd_updates==9&&u.receive_robot_cmd.time_stamp==10);
    good=encode(command(11));feed(u,Bytes(good.begin(),good.begin()+25));assert(u.rx_size==25);
    hUsbDeviceHS.dev_state=0;u.Receive();assert(u.rx_size==0&&mask==0);
    hUsbDeviceHS.dev_state=USBD_STATE_CONFIGURED;feed(u,good);assert(robot_cmd_updates==10);
    reset();Usb wrap;feed(wrap,encode(command(0xFFFFFFFE)));feed(wrap,encode(command(1)));
    assert(robot_cmd_updates==2);
    puts("USB stream: every split, bytewise, 464-byte read, old IDs/lengths, CRC, disconnect and timestamp wrap passed");
}
template<class Frame>
Frame transmitted(uint8_t id){
    assert(tx.size()==sizeof(Frame)&&tx[2]==id);
    assert(verify_CRC8_check_sum(tx.data(),4)&&verify_CRC16_check_sum(tx.data(),tx.size()));
    Frame frame{};memcpy(&frame,tx.data(),sizeof(frame));return frame;
}
template<class Frame>
void initHeader(Frame& frame,uint8_t id){
    frame.frame_header={SEND_SOF,sizeof(Frame)-6,id,0};
    append_CRC8_check_sum(reinterpret_cast<uint8_t*>(&frame.frame_header),4);
}
void statusTests(){
    using namespace LuojiaFox;
    reset();Usb u;
    initHeader(u.send_navigation_status,NAVIGATION_STATUS_SEND_ID);
    initHeader(u.send_robot_motion,ROBOT_MOTION_DATA_SEND_ID);
    initHeader(u.send_joint_state,JOINT_STATE_SEND_ID);
    feed(u,encode(command(1)));
    feed(u,encode(command(2,0,ROBOT_CMD_CHASSIS_ENABLED)));
    snapshot.chassis={.5f,-.2f,.7f,true,tick-5};
    snapshot.imu.yaw_deg=90.f;snapshot.imu.last_update_ms=tick-4;
    snapshot.gimbal.online=true;snapshot.gimbal.updated_at_ms=tick-3;
    snapshot.gimbal.transform_state=GimbalTransformState::Preparing;
    snapshot.gimbal.transformed=GimbalShape::Upright;
    snapshot.gimbal.joint_yaw_rad=.6f;snapshot.gimbal.joint_pitch_rad=-.2f;
    snapshot.gimbal.joint_roll_rad=.1f;
    board_link::feedback={-.3f,true,true,true,5,tick-2};
    referee.power={80,47,tick-100,tick-200,true,true,true};
    u.SendNavigationStatus();
    auto status=transmitted<SendDataNavigationStatus_s>(NAVIGATION_STATUS_SEND_ID);
    assert(sizeof(status)==39&&status.frame_header.len==33);
    assert(status.data.transform_state==3&&status.data.transformed==4);
    assert(status.data.session==1&&status.data.command_time_stamp==2);
    assert(status.data.flags==0x1F&&status.data.buffer_energy_j==47);
    assert(status.data.main_imu.time_stamp==tick-4&&fabsf(status.data.main_imu.yaw-1.5707963f)<1e-6f);
    assert(status.data.big_yaw.angle==-.3f&&status.data.big_yaw.time_stamp==tick-2);
    u.SendRobotMotion();
    auto motion=transmitted<SendDataRobotMotion_s>(ROBOT_MOTION_DATA_SEND_ID);
    assert(motion.time_stamp==tick-5&&motion.data.speed_vector.vy==-.2f);
    u.SendJointState();
    auto joints=transmitted<SendDataJointState_s>(JOINT_STATE_SEND_ID);
    assert(joints.time_stamp==tick-3&&joints.data.yaw==.6f&&joints.data.pitch==-.2f);
    // A new fold command may arrive before the gimbal task leaves upright READY.
    snapshot.gimbal.transform_state=GimbalTransformState::Ready;
    feed(u,encode(command(3,ROBOT_CMD_NAV_FOLD,ROBOT_CMD_CHASSIS_ENABLED|ROBOT_CMD_HEADING_VALID)));
    u.SendNavigationStatus();status=transmitted<SendDataNavigationStatus_s>(NAVIGATION_STATUS_SEND_ID);
    assert(status.data.transform_state==3&&status.data.transformed==4);
    snapshot.gimbal.transformed=GimbalShape::Folded;
    u.SendNavigationStatus();status=transmitted<SendDataNavigationStatus_s>(NAVIGATION_STATUS_SEND_ID);
    assert(status.data.transform_state==4&&status.data.transformed==3);
    snapshot.gimbal.joint_roll_rad=NAN;
    u.SendNavigationStatus();status=transmitted<SendDataNavigationStatus_s>(NAVIGATION_STATUS_SEND_ID);
    assert(status.data.transform_state==1);
    snapshot.gimbal.joint_roll_rad=.1f;
    feed(u,encode(command(4,ROBOT_CMD_NAV_UPRIGHT,ROBOT_CMD_CHASSIS_ENABLED)));
    u.SendNavigationStatus();status=transmitted<SendDataNavigationStatus_s>(NAVIGATION_STATUS_SEND_ID);
    assert(status.data.transform_state==5&&status.data.transformed==3);
    snapshot.chassis.valid=false;
    tx.clear();u.SendRobotMotion();assert(tx.empty());
    u.SendNavigationStatus();status=transmitted<SendDataNavigationStatus_s>(NAVIGATION_STATUS_SEND_ID);
    assert(status.data.transform_state==1);
    tick+=100;board_link::feedback={};referee.power.online=false;
    tx.clear();u.SendJointState();assert(tx.empty());
    u.SendNavigationStatus();status=transmitted<SendDataNavigationStatus_s>(NAVIGATION_STATUS_SEND_ID);
    assert(status.data.transform_state==0&&status.data.flags==0&&status.data.buffer_energy_j==0);
    puts("USB feedback: compact status, split motion/joints, original source timestamps, CRC and stale feedback passed");
}
int main(){
    commandTests();
    receiveTests();
    statusTests();
}
"""
(out/'usb_regression.cpp').write_text(prefix+tests,encoding='utf-8')
crc=root/'RM2027-LFCore/1_Middleware/Algorithm/CRC'
exe=out/'usb_regression.exe'
subprocess.run(['g++','-std=c++14','-O0','-g','-Wall','-Wextra','-I'+str(out),'-I'+str(root/'RM2027-LFCore/2_Device/USB'),'-I'+str(crc),'-I'+str(root/'RM2027-LFCore/3_LuojiaFox'),'-I'+str(root/'RM2027-LFCore/4_Application/Chassis'),'-I'+str(root/'RM2027-LFCore/4_Application/Gimbal'),str(out/'usb_regression.cpp'),str(crc/'CRC8_CRC16.c'),'-o',str(exe)],check=True)
subprocess.run([str(exe)],check=True)
