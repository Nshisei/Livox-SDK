#include <fstream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <inttypes.h> // PRIu64のため

#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include "cmdline.h"
#include "lds_lidar.h"
/** Cmdline input broadcast code */
static std::vector<std::string> cmdline_broadcast_code;


/** コマンドライン引数の処理関数 */
void SetProgramOption(int argc, const char *argv[]) {
  cmdline::parser cmd;
  cmd.add<std::string>("code", 'c', "Register device broadcast code", false);
  cmd.add("log", 'l', "Save the log file");
  cmd.add("help", 'h', "Show help");
  cmd.parse_check(argc, const_cast<char **>(argv));

  extern std::vector<std::string> cmdline_broadcast_code;
  if (cmd.exist("code")) {
    std::string sn_list = cmd.get<std::string>("code");
    printf("Register broadcast code: %s\n", sn_list.c_str());
    size_t pos = 0;
    cmdline_broadcast_code.clear();
    while ((pos = sn_list.find("&")) != std::string::npos) {
      cmdline_broadcast_code.push_back(sn_list.substr(0, pos));
      sn_list.erase(0, pos + 1);
    }
    cmdline_broadcast_code.push_back(sn_list);
  }
  if (cmd.exist("log")) {
    printf("Save the log file.\n");
    SaveLoggerFile();
  }
}


int main(int argc, const char *argv[]) {
  /** コマンドライン引数の処理 */
  SetProgramOption(argc, argv);

  LdsLidar& read_lidar = LdsLidar::GetInstance();

  int ret = read_lidar.InitLdsLidar(cmdline_broadcast_code);
  if (!ret) {
    printf("Init lds lidar success!\n");
  } else {
    printf("Init lds lidar fail!\n");
    return -1;
  }

  printf("Start discovering device.\n");

  std::this_thread::sleep_for(std::chrono::seconds(5));

  read_lidar.DeInitLdsLidar();
  printf("Livox lidar demo end!\n");

  return 0;
}
