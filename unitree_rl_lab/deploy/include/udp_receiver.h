// udp_receiver.h
#pragma once
#include <atomic>
#include <thread>
#include <string>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>

#include "cmd_buffer.h"  // cmd_buffer()/vx,vy,wz,mode_cmd 사용

// === UDP Receiver ===
namespace udp_receiver {

    inline std::atomic<bool> running{false};
    inline std::thread th;

    inline void start(const std::string& bind_ip="127.0.0.1", int port=9000) {
        bool expected = false;
        if (!running.compare_exchange_strong(expected, true)) return; // 이미 실행중

        th = std::thread([bind_ip, port]{
            int fd = socket(AF_INET, SOCK_DGRAM, 0);
            if (fd < 0) { perror("socket"); running=false; return; }

            sockaddr_in addr{};
            addr.sin_family = AF_INET;
            addr.sin_port   = htons(port);
            addr.sin_addr.s_addr = inet_addr(bind_ip.c_str());
            if (bind(fd, (sockaddr*)&addr, sizeof(addr)) < 0) {
                perror("bind"); close(fd); running=false; return;
            }

            char buf[256];
            while (running.load()) {
                ssize_t n = recv(fd, buf, sizeof(buf)-1, 0);
                if (n <= 0) continue;
                buf[n] = '\0';

                // 1) 속도 3플로트 "vx vy wz"
                float vx=0.f, vy=0.f, wz=0.f;
                if (sscanf(buf, "%f %f %f", &vx, &vy, &wz) == 3) {
                    cmd_buffer().vx.store(vx);
                    cmd_buffer().vy.store(vy);
                    cmd_buffer().wz.store(wz);
                    continue;
                }

                // 2) 모드 명령: "CMD 2", "mode 3", 혹은 단일 "2"
                int m=0;
                if (sscanf(buf, "CMD %d", &m) == 1 ||
                    sscanf(buf, "cmd %d", &m) == 1 ||
                    sscanf(buf, "mode %d", &m) == 1 ||
                    sscanf(buf, "%d", &m) == 1)
                {
                    cmd_buffer().mode_cmd.store(m);
                    continue;
                }

                // 다른 포맷은 무시
            }
            close(fd);
        });
    }

    inline void stop() {
        bool expected = true;
        if (!running.compare_exchange_strong(expected, false)) return;
        if (th.joinable()) th.join();
    }
}
