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

/** 設定用変数 */
static std::string camera_position_path;
static std::string save_dir_path;
static size_t max_points = 1000000; // デフォルトの最大表示点数
static size_t screenshot_interval = 1000; // デフォルトのスクリーンショット間隔

/** コマンドライン引数の処理関数 */
void SetProgramOption(int argc, const char *argv[]) {
  cmdline::parser cmd;
  cmd.add<std::string>("code", 'c', "Register device broadcast code", false);
  cmd.add<std::string>("camera", 'p', "Absolute path to camera_position.txt", true); // 必須引数に変更
  cmd.add<std::string>("save_dir", 's', "Absolute path to save data", true); // 必須引数に変更
  cmd.add<size_t>("max_points", 'm', "Max number of points to display", false, 1000000);
  cmd.add<size_t>("interval", 'i', "Screenshot interval (callback count)", false, 100);
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

  // カメラ位置のパス設定（必須）
  if (cmd.exist("camera")) {
    camera_position_path = cmd.get<std::string>("camera");
    printf("Camera position path: %s\n", camera_position_path.c_str());
  } else {
    fprintf(stderr, "Error: Camera position file path must be provided using --camera option.\n");
    exit(EXIT_FAILURE);
  }

  // カメラ位置のパス設定（必須）
  if (cmd.exist("save_dir")) {
    save_dir_path = cmd.get<std::string>("save_dir");
    printf("Save Dir path: %s\n", save_dir_path.c_str());
  } else {
    fprintf(stderr, "Error: Save Data Dir path must be provided using --save_dir option.\n");
    exit(EXIT_FAILURE);
  }

  // MAX_POINTS の設定
  max_points = cmd.get<size_t>("max_points");
  printf("Max points: %zu\n", max_points);

  // スクリーンショットの間隔設定
  screenshot_interval = cmd.get<size_t>("interval");
  printf("Screenshot interval: %zu\n", screenshot_interval);

  if (cmd.exist("log")) {
    printf("Save the log file.\n");
    SaveLoggerFile();
  }
}


int main(int argc, const char *argv[]) {
  /** コマンドライン引数の処理 */
  SetProgramOption(argc, argv);

  LdsLidar& read_lidar = LdsLidar::GetInstance();

   int ret = read_lidar.InitLdsLidar(cmdline_broadcast_code, camera_position_path,save_dir_path, max_points, screenshot_interval);
  if (!ret) {
    printf("Init lds lidar success!\n");
  } else {
    printf("Init lds lidar fail!\n");
    return -1;
  }

  printf("Start discovering device.\n");
  while (true)
  {
    std::this_thread::sleep_for(std::chrono::seconds(5));
  }
  

  read_lidar.DeInitLdsLidar();
  printf("Livox lidar demo end!\n");

  return 0;
}
