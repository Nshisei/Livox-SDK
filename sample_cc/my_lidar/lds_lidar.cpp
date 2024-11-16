//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "lds_lidar.h"
#include <stdio.h>
#include <string.h>
#include <thread>
#include <memory>
#include <sstream>
#include <inttypes.h>
#include <fstream>
#include <ctime>
#include <iomanip>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <deque>
#include <sys/stat.h> // mkdir のため
#include <unistd.h>   // access のため
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
static auto accumulated_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
static size_t callback_count = 0;
/** Const varible ------------------------------------------------------------------------------- */
/** User add broadcast code here */
static const char* local_broadcast_code_list[] = {
  "000000000000001",
};

static std::string global_camera_position_path;
static std::string global_save_dir_path;
static size_t global_max_points = 1000000;  // デフォルト値
static size_t global_screenshot_interval = 100;  // デフォルト値


/** For callback use only */
LdsLidar* g_lidars = nullptr;

/** Lds lidar function ---------------------------------------------------------------------------*/
LdsLidar::LdsLidar() {
  auto_connect_mode_ = true;
  whitelist_count_   = 0;
  is_initialized_    = false;

  lidar_count_       = 0;
  memset(broadcast_code_whitelist_, 0, sizeof(broadcast_code_whitelist_));

  memset(lidars_, 0, sizeof(lidars_));
  for (uint32_t i=0; i<kMaxLidarCount; i++) {
    lidars_[i].handle = kMaxLidarCount; 
    /** Unallocated state */
    lidars_[i].connect_state = kConnectStateOff;
  }
}

LdsLidar::~LdsLidar(){
}

int LdsLidar::InitLdsLidar(std::vector<std::string>& broadcast_code_strs, 
                           const std::string& camera_position_path,
                           const std::string& save_dir_path,
                           size_t max_points, size_t screenshot_interval) {

  if (is_initialized_) {
    printf("LiDAR data source is already inited!\n");
    return -1;
  }

  // Lidar SDKの初期化
  if (!Init()) {
    Uninit();
    printf("Livox-SDK init fail!\n");
    return -1;
  }

  // 必要な設定をクラスメンバー変数に保存
  // グローバル変数に設定を保存
  global_camera_position_path = camera_position_path;
  global_save_dir_path = save_dir_path;
  global_max_points = max_points;
  global_screenshot_interval = screenshot_interval;

  printf("Initialized with:\n");
  printf("  Camera Position Path: %s\n", global_camera_position_path.c_str());
  printf("  Max Points: %zu\n", global_max_points);
  printf("  Screenshot Interval: %zu\n", global_screenshot_interval);


  viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer"); 
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();

  LivoxSdkVersion _sdkversion;
  GetLivoxSdkVersion(&_sdkversion);
  printf("Livox SDK version %d.%d.%d\n", _sdkversion.major, _sdkversion.minor, _sdkversion.patch);

  SetBroadcastCallback(LdsLidar::OnDeviceBroadcast);
  SetDeviceStateUpdateCallback(LdsLidar::OnDeviceChange);

  /** Add commandline input broadcast code */
  for (auto input_str : broadcast_code_strs) {
    LdsLidar::AddBroadcastCodeToWhitelist(input_str.c_str());
  }

  /** Add local broadcast code */
  LdsLidar::AddLocalBroadcastCode();

  if (whitelist_count_) {
    LdsLidar::DisableAutoConnectMode();
    printf("Disable auto connect mode!\n");

    printf("List all broadcast code in whiltelist:\n");
    for (uint32_t i=0; i<whitelist_count_; i++) {
      printf("%s\n", broadcast_code_whitelist_[i]);
    }
  } else {
    LdsLidar::EnableAutoConnectMode();
    printf("No broadcast code was added to whitelist, swith to automatic connection mode!\n");
  }

  /** Start livox sdk to receive lidar data */
  if (!Start()) {
    Uninit();
    printf("Livox-SDK init fail!\n");
    return -1;
  }

  /** Add here, only for callback use */
  if (g_lidars == nullptr) {
    g_lidars = this;
  }
  is_initialized_= true;
  printf("Livox-SDK init success!\n");

  return 0;
}


int LdsLidar::DeInitLdsLidar(void) {

  if (!is_initialized_) {
    printf("LiDAR data source is not exit");
    return -1;
  }

  Uninit();
  printf("Livox SDK Deinit completely!\n");

  return 0;
}

// ディレクトリを作成する関数
void CreateDirectory(const std::string& path) {
    if (access(path.c_str(), F_OK) == -1) { // ディレクトリが存在しない場合
        if (mkdir(path.c_str(), 0755) == -1) { // ディレクトリを作成
            std::cerr << "Failed to create directory " << path
                      << ": " << strerror(errno) << std::endl;
        }
    }
}

// ファイルパスを生成する関数
std::string CreateFilePath(const std::string& base_dir,uint64_t cur_timestamp, const std::string& type) {
    // 現在時刻を取得
    std::time_t t = std::time(nullptr);
    std::tm local_time = *std::localtime(&t);
    

    // 時間部分をフォーマット
    std::ostringstream date_stream, hour_stream, minute_stream;
    date_stream << std::put_time(&local_time, "%Y%m%d");
    hour_stream << std::put_time(&local_time, "%H00");
    minute_stream << std::put_time(&local_time, "%M");

    // yyyymmdd/hh00/mm ディレクトリを構築
    std::string path = global_save_dir_path + base_dir + "/" + type + "/" + date_stream.str() + "/" + hour_stream.str() + "/" + minute_stream.str();

    // ディレクトリを作成
    CreateDirectory(global_save_dir_path + base_dir);
    CreateDirectory(global_save_dir_path + base_dir + "/" + type);
    CreateDirectory(global_save_dir_path + base_dir + "/" + type + "/" + date_stream.str());
    CreateDirectory(global_save_dir_path + base_dir + "/" + type + "/" + date_stream.str() + "/" + hour_stream.str());
    CreateDirectory(path);

    // ファイル名の生成
    std::ostringstream file_name_stream;
    file_name_stream << path << "/"
                     << std::to_string(cur_timestamp); // ミリ秒部分を追加

    return file_name_stream.str();
}
/** Static function in LdsLidar for callback or event process ------------------------------------*/

/** タイムスタンプからファイル名を生成 */
std::string GenerateFileName(uint64_t timestamp) {
  std::time_t time = timestamp / 1000000;
  std::tm* tm_info = std::localtime(&time);
  char buffer[64];
  strftime(buffer, 64, "%Y%m%d_%H%M%S", tm_info);
  std::stringstream ss;
  ss << buffer << "_" << (timestamp % 1000000) << ".csv";
  return ss.str();
}

// 点群データをCSVに保存
void SavePointsToCsv(const std::string& filename,const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    std::ofstream csv_file(filename);
    if (!csv_file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    csv_file << "x,y,z\n";
    for (const auto& point : cloud->points) {
        csv_file << point.x << "," << point.y << "," << point.z << "\n";
    }
    csv_file.close();
    std::cout << "Saved points to " << filename << std::endl;
}


// ファイルからカメラ位置を読み込む
void loadCameraPosition(pcl::visualization::PCLVisualizer::Ptr viewer) {
    std::ifstream file(global_camera_position_path);
    if (!file.is_open()) {
        std::cerr << "Failed to open camera position file." << std::endl;
        return;
    }

    Eigen::Vector3f position, look_at, up_vector;
    file >> position.x() >> position.y() >> position.z();
    file >> look_at.x() >> look_at.y() >> look_at.z();
    file >> up_vector.x() >> up_vector.y() >> up_vector.z();
    file.close();

    // カメラの位置、注視点、上向きベクトルを設定
    viewer->setCameraPosition(position.x(), position.y(), position.z(),
                              look_at.x(), look_at.y(), look_at.z(),
                              up_vector.x(), up_vector.y(), up_vector.z());

    std::cout << "Camera position loaded and set." << std::endl;
}

/** LiDARデータを取得するコールバック関数 */
void LdsLidar::GetLidarDataCb(uint8_t handle, LivoxEthPacket *data,
                              uint32_t data_num, void *client_data) {
  if (!data || !data_num || (handle >= kMaxLidarCount)) {
    return;
  }

  uint64_t cur_timestamp = *((uint64_t *)(data->timestamp));
  uint8_t data_type = data->data_type;

  printf("count cb %zu\n", callback_count);
  switch (data_type) {
    case kExtendCartesian: {
      uint32_t point_count = data_num;
      LivoxExtendRawPoint *p_point_data = (LivoxExtendRawPoint *)data->data;
      auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      cloud->reserve(point_count);
      for (uint32_t i = 0; i < point_count; ++i) {
	    pcl::PointXYZ point;
            point.x = p_point_data[i].x ;
            point.y = p_point_data[i].y ;
            point.z = p_point_data[i].z ;
            cloud->points.push_back(point);
      }
      std::string base_path = CreateFilePath("Lidar",cur_timestamp, "csv") + ".csv";
      SavePointsToCsv(base_path, cloud);

      // 累積点群に新しい点群を追加
      *accumulated_cloud += *cloud;

      // 最大点数を超えた場合、古い点を削除
      if (accumulated_cloud->points.size() > global_max_points) {
           accumulated_cloud->points.erase(
            accumulated_cloud->points.begin(),
             accumulated_cloud->points.begin() + (accumulated_cloud->points.size() - global_max_points)
                );
       }
      if (++callback_count % global_screenshot_interval == 0) {
        accumulated_cloud->width = accumulated_cloud->points.size();
        accumulated_cloud->height = 1;
        accumulated_cloud->is_dense = true;

        viewer->removeAllPointClouds();
	      viewer->addPointCloud<pcl::PointXYZ>(accumulated_cloud, "sample cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	
        viewer->initCameraParameters();
        loadCameraPosition(viewer);
        printf("Total points in queue: %lu\n",accumulated_cloud->points.size());
        
        std::string screenshot_path = CreateFilePath("Lidar",cur_timestamp, "png") + ".png";
        viewer->saveScreenshot(screenshot_path);
        std::cout << "Screenshot saved as " << screenshot_path << std::endl;
      }
       
      break;
    }
    default:
      printf("Unsupported data type: %u\n", data_type);
      break;

  }
}



void LdsLidar::OnDeviceBroadcast(const BroadcastDeviceInfo *info) {
  if (info == nullptr) {
    return;
  }

  if (info->dev_type == kDeviceTypeHub) {
    printf("In lidar mode, couldn't connect a hub : %s\n", info->broadcast_code);
    return;
  }

  if (g_lidars->IsAutoConnectMode()) {
    printf("In automatic connection mode, will connect %s\n", info->broadcast_code);
  } else {
    if (!g_lidars->FindInWhitelist(info->broadcast_code)) {
      printf("Not in the whitelist, please add %s to if want to connect!\n",\
             info->broadcast_code);
      return;
    }
  }

  livox_status result = kStatusFailure;
  uint8_t handle = 0;
  result = AddLidarToConnect(info->broadcast_code, &handle);
  if (result == kStatusSuccess && handle < kMaxLidarCount) {
    SetDataCallback(handle, LdsLidar::GetLidarDataCb, (void *)g_lidars);
    LidarDevice* p_lidar = &(g_lidars->lidars_[handle]);
    p_lidar->handle = handle;
    p_lidar->connect_state = kConnectStateOff;
    p_lidar->config.enable_fan = true;
    p_lidar->config.return_mode = kStrongestReturn;
    p_lidar->config.coordinate = kCoordinateCartesian;
    p_lidar->config.imu_rate = kImuFreq200Hz;
  } else {
    printf("Add lidar to connect is failed : %d %d \n", result, handle);
  }
}

/** Callback function of changing of device state. */
void LdsLidar::OnDeviceChange(const DeviceInfo *info, DeviceEvent type) {
  if (info == nullptr) {
    return;
  }

  uint8_t handle = info->handle;
  if (handle >= kMaxLidarCount) {
    return;
  }

  LidarDevice* p_lidar = &(g_lidars->lidars_[handle]);
  if (type == kEventConnect) {
    QueryDeviceInformation(handle, DeviceInformationCb, g_lidars);
    if (p_lidar->connect_state == kConnectStateOff) {
      p_lidar->connect_state = kConnectStateOn;
      p_lidar->info = *info;
    }
    printf("[WARNING] Lidar sn: [%s] Connect!!!\n", info->broadcast_code);
  } else if (type == kEventDisconnect) {
    p_lidar->connect_state = kConnectStateOff;
    printf("[WARNING] Lidar sn: [%s] Disconnect!!!\n", info->broadcast_code);
  } else if (type == kEventStateChange) {
    p_lidar->info = *info;
    printf("[WARNING] Lidar sn: [%s] StateChange!!!\n", info->broadcast_code);
  }

  if (p_lidar->connect_state == kConnectStateOn) {
    printf("Device Working State %d\n", p_lidar->info.state);
    if (p_lidar->info.state == kLidarStateInit) {
      printf("Device State Change Progress %u\n", p_lidar->info.status.progress);
    } else {
      printf("Device State Error Code 0X%08x\n", p_lidar->info.status.status_code.error_code);
    }
    printf("Device feature %d\n", p_lidar->info.feature);
    SetErrorMessageCallback(handle, LdsLidar::LidarErrorStatusCb);

    /** Config lidar parameter */
    if (p_lidar->info.state == kLidarStateNormal) {
      if (p_lidar->config.coordinate != 0) {
        SetSphericalCoordinate(handle, LdsLidar::SetCoordinateCb, g_lidars);
      } else {
        SetCartesianCoordinate(handle, LdsLidar::SetCoordinateCb, g_lidars);
      }
      p_lidar->config.set_bits |= kConfigCoordinate;

      if (kDeviceTypeLidarMid40 != info->type) {
        LidarSetPointCloudReturnMode(handle, (PointCloudReturnMode)(p_lidar->config.return_mode),\
                                     LdsLidar::SetPointCloudReturnModeCb, g_lidars);
        p_lidar->config.set_bits |= kConfigReturnMode;
      }

      if (kDeviceTypeLidarMid40 != info->type && kDeviceTypeLidarMid70 != info->type) {
        LidarSetImuPushFrequency(handle, (ImuFreq)(p_lidar->config.imu_rate),\
                                 LdsLidar::SetImuRatePushFrequencyCb, g_lidars);
        p_lidar->config.set_bits |= kConfigImuRate;
      }

      p_lidar->connect_state = kConnectStateConfig;
    }
  }
}

/** Query the firmware version of Livox LiDAR. */
void LdsLidar::DeviceInformationCb(livox_status status, uint8_t handle, \
                         DeviceInformationResponse *ack, void *client_data) {
  if (status != kStatusSuccess) {
    printf("Device Query Informations Failed : %d\n", status);
  }
  if (ack) {
    printf("firm ver: %d.%d.%d.%d\n",
           ack->firmware_version[0],
           ack->firmware_version[1],
           ack->firmware_version[2],
           ack->firmware_version[3]);
  }
}

/** Callback function of Lidar error message. */
void LdsLidar::LidarErrorStatusCb(livox_status status, uint8_t handle, ErrorMessage *message) {
  static uint32_t error_message_count = 0;
  if (message != NULL) {
    ++error_message_count;
    if (0 == (error_message_count % 100)) {
      printf("handle: %u\n", handle);
      printf("temp_status : %u\n", message->lidar_error_code.temp_status);
      printf("volt_status : %u\n", message->lidar_error_code.volt_status);
      printf("motor_status : %u\n", message->lidar_error_code.motor_status);
      printf("dirty_warn : %u\n", message->lidar_error_code.dirty_warn);
      printf("firmware_err : %u\n", message->lidar_error_code.firmware_err);
      printf("pps_status : %u\n", message->lidar_error_code.device_status);
      printf("fan_status : %u\n", message->lidar_error_code.fan_status);
      printf("self_heating : %u\n", message->lidar_error_code.self_heating);
      printf("ptp_status : %u\n", message->lidar_error_code.ptp_status);
      printf("time_sync_status : %u\n", message->lidar_error_code.time_sync_status);
      printf("system_status : %u\n", message->lidar_error_code.system_status);
    }
  }
}

void LdsLidar::ControlFanCb(livox_status status, uint8_t handle, \
                           uint8_t response, void *client_data) {

}

void LdsLidar::SetPointCloudReturnModeCb(livox_status status, uint8_t handle, \
                                         uint8_t response, void *client_data) {
  LdsLidar* lds_lidar = static_cast<LdsLidar *>(client_data);

  if (handle >= kMaxLidarCount) {
    return;
  }
  LidarDevice* p_lidar = &(lds_lidar->lidars_[handle]);

  if (status == kStatusSuccess) {
    p_lidar->config.set_bits &= ~((uint32_t)(kConfigReturnMode));
    printf("Set return mode success!\n");

    if (!p_lidar->config.set_bits) {
      LidarStartSampling(handle, LdsLidar::StartSampleCb, lds_lidar);
      p_lidar->connect_state = kConnectStateSampling;
    }
  } else {
    LidarSetPointCloudReturnMode(handle, (PointCloudReturnMode)(p_lidar->config.return_mode),\
                                 LdsLidar::SetPointCloudReturnModeCb, lds_lidar);
    printf("Set return mode fail, try again!\n");
  }
}

void LdsLidar::SetCoordinateCb(livox_status status, uint8_t handle, \
                               uint8_t response, void *client_data) {
  LdsLidar* lds_lidar = static_cast<LdsLidar *>(client_data);

  if (handle >= kMaxLidarCount) {
    return;
  }
  LidarDevice* p_lidar = &(lds_lidar->lidars_[handle]);

  if (status == kStatusSuccess) {
    p_lidar->config.set_bits &= ~((uint32_t)(kConfigCoordinate));
    printf("Set coordinate success!\n");

    if (!p_lidar->config.set_bits) {
       LidarStartSampling(handle, LdsLidar::StartSampleCb, lds_lidar);
       p_lidar->connect_state = kConnectStateSampling;
    }
  } else {
    if (p_lidar->config.coordinate != 0) {
      SetSphericalCoordinate(handle, LdsLidar::SetCoordinateCb, lds_lidar);
    } else {
      SetCartesianCoordinate(handle, LdsLidar::SetCoordinateCb, lds_lidar);
    }

    printf("Set coordinate fail, try again!\n");
  }
}

void LdsLidar::SetImuRatePushFrequencyCb(livox_status status, uint8_t handle, \
                                         uint8_t response, void *client_data) {
  LdsLidar* lds_lidar = static_cast<LdsLidar *>(client_data);

  if (handle >= kMaxLidarCount) {
    return;
  }
  LidarDevice* p_lidar = &(lds_lidar->lidars_[handle]);

  if (status == kStatusSuccess) {
    p_lidar->config.set_bits &= ~((uint32_t)(kConfigImuRate));
    printf("Set imu rate success!\n");

    if (!p_lidar->config.set_bits) {
      LidarStartSampling(handle, LdsLidar::StartSampleCb, lds_lidar);
      p_lidar->connect_state = kConnectStateSampling;
    }

  } else {
    LidarSetImuPushFrequency(handle, (ImuFreq)(p_lidar->config.imu_rate),\
                             LdsLidar::SetImuRatePushFrequencyCb, lds_lidar);
    printf("Set imu rate fail, try again!\n");
  }
}

/** Callback function of starting sampling. */
void LdsLidar::StartSampleCb(livox_status status, uint8_t handle, \
                             uint8_t response, void *client_data) {
  LdsLidar* lds_lidar = static_cast<LdsLidar *>(client_data);

  if (handle >= kMaxLidarCount) {
    return;
  }

  LidarDevice* p_lidar = &(lds_lidar->lidars_[handle]);
  if (status == kStatusSuccess) {
    if (response != 0) {
      p_lidar->connect_state = kConnectStateOn;
      printf("Lidar start sample fail : state[%d] handle[%d] res[%d]\n", \
             status, handle, response);
    } else {
      printf("Lidar start sample success\n");
   }
  } else if (status == kStatusTimeout) {
    p_lidar->connect_state = kConnectStateOn;
    printf("Lidar start sample timeout : state[%d] handle[%d] res[%d]\n", \
           status, handle, response);
  }
}

/** Callback function of stopping sampling. */
void LdsLidar::StopSampleCb(livox_status status, uint8_t handle, \
                            uint8_t response, void *client_data) {
}

/** Add broadcast code to whitelist */
int LdsLidar::AddBroadcastCodeToWhitelist(const char* bd_code) {
  if (!bd_code || (strlen(bd_code) > kBroadcastCodeSize) || \
      (whitelist_count_ >= kMaxLidarCount)) {
    return -1;
  }

  if (LdsLidar::FindInWhitelist(bd_code)) {
    printf("%s is alrealy exist!\n", bd_code);
    return -1;
  }

  strcpy(broadcast_code_whitelist_[whitelist_count_], bd_code);
  ++whitelist_count_;

  return 0;
}

void LdsLidar::AddLocalBroadcastCode(void) {
  for (size_t i=0; i<sizeof(local_broadcast_code_list)/sizeof(intptr_t); ++i) {
    std::string invalid_bd = "000000000";
    printf("Local broadcast code : %s\n", local_broadcast_code_list[i]);
    if ((kBroadcastCodeSize == strlen(local_broadcast_code_list[i]) + 1) && \
        (nullptr == strstr(local_broadcast_code_list[i], invalid_bd.c_str()))) {
      LdsLidar::AddBroadcastCodeToWhitelist(local_broadcast_code_list[i]);
    } else {
      printf("Invalid local broadcast code : %s\n", local_broadcast_code_list[i]);
    }
  }
}

bool LdsLidar::FindInWhitelist(const char* bd_code) {
  if (!bd_code) {
    return false;
  }

  for (uint32_t i=0; i<whitelist_count_; i++) {
    if (strncmp(bd_code, broadcast_code_whitelist_[i], kBroadcastCodeSize) == 0) {
      return true;
    }
  }

  return false;
}


