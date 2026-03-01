// cmd_buffer.h
#pragma once
#include <atomic>

struct CmdBuffer {
  std::atomic<float> vx{0.f};
  std::atomic<float> vy{0.f};
  std::atomic<float> wz{0.f};

  // 추가: UDP로 들어오는 상태 전환용 숫자 명령 (1,2,3...)
  std::atomic<int>   mode_cmd{0};
};

inline CmdBuffer& cmd_buffer() {
  static CmdBuffer buf;
  return buf;
}

// 추가: 모드 명령을 1회성으로 소비(edge trigger)하는 헬퍼
inline int take_mode_cmd() {
  return cmd_buffer().mode_cmd.exchange(0);
}
