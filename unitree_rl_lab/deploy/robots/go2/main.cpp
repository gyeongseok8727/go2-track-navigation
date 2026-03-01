// main.cpp
#include "FSM/CtrlFSM.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FixStand.h"
#include "FSM/State_RLBase.h"
#include "FSM/State_SitDown.h"

#include "udp_receiver.h"
#include "cmd_buffer.h"   // take_mode_cmd() 사용

std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = nullptr;

// 추가: UDP 숫자 명령이 특정 값인지 체크(1회성 소비)
static bool udp_cmd_is(int want) {
    int m = take_mode_cmd();   // 읽으면 자동으로 0으로 초기화
    return (m == want);
}

void init_fsm_state()
{
    auto lowcmd_sub = std::make_shared<unitree::robot::go2::subscription::LowCmd>();
    usleep(0.2 * 1e6);
    if(!lowcmd_sub->isTimeout())
    {
        spdlog::critical("The other process is using the lowcmd channel, please close it first.");
        unitree::robot::go2::shutdown();
        // exit(0);
    }
    FSMState::lowcmd = std::make_unique<LowCmd_t>();
    FSMState::lowstate = std::make_shared<LowState_t>();

    if (!FSMState::keyboard) {
        FSMState::keyboard = std::make_shared<Keyboard>();
    }

    // UDP 수신 시작 (기존 포트/주소 유지)
    udp_receiver::start("127.0.0.1", 9000);

    spdlog::info("Waiting for connection to robot...");
    FSMState::lowstate->wait_for_connection();
    spdlog::info("Connected to robot.");
}

int main(int argc, char** argv)
{
    // Load parameters
    auto vm = param::helper(argc, argv);

    std::cout << " --- Unitree Robotics --- \n";
    std::cout << "     Go2 Controller \n";

    // Unitree DDS Config
    unitree::robot::ChannelFactory::Instance()->Init(0, vm["network"].as<std::string>());

    init_fsm_state();

    // Initialize FSM
    auto & joy = FSMState::lowstate->joystick; // (유지) 미사용이더라도 기존 코드 변경 최소화
    auto fsm = std::make_unique<CtrlFSM>(new State_Passive(FSMMode::Passive));

    // 기존 조이스틱 조건을 UDP 숫자키로 교체
    // (1) Passive -> FixStand : UDP "1"
    fsm->states.back()->registered_checks.emplace_back(
        std::make_pair(
            [&]()->bool{ return udp_cmd_is(1); },  // 숫자 1 수신 시 전환
            (int)FSMMode::FixStand
        )
    );
    fsm->add(new State_FixStand(FSMMode::FixStand));

    // (2) FixStand -> Velocity(RLBase) : UDP "2"
    fsm->states.back()->registered_checks.emplace_back(
        std::make_pair(
            [&]()->bool{ return udp_cmd_is(2); },  // 숫자 2 수신 시 전환
            (int)FSMMode::Velocity
        )
    );
    fsm->add(new State_RLBase(FSMMode::Velocity, "Velocity"));

    // (3) Velocity -> SitDown : UDP "3"
    fsm->states.back()->registered_checks.emplace_back(
        std::make_pair(
            [&]()->bool{ return udp_cmd_is(3); },  // 숫자 3 수신 시 전환
            (int)FSMMode::SitDown
        )
    );
    fsm->add(new State_SitDown(FSMMode::SitDown));

    std::cout << "UDP 숫자키로 상태 전환합니다.\n";
    std::cout << " 1 -> FixStand, 2 -> Velocity, 3 -> SitDown\n";

    while (true)
    {
        sleep(1);
    }
    
    return 0;
}
