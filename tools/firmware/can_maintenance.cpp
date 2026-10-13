// UMRR maintenance through the original Smart Access SDK and SocketCAN can0.
// Default mode only reads settings. Update mode waits for a fresh bootloader echo.
#include <CommunicationServicesIface.h>
#include <Instruction.h>
#include <InstructionBatch.h>
#include <nlohmann/json.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <unistd.h>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>

using Json = nlohmann::json;
using namespace std::chrono_literals;
constexpr uint32_t CLIENT = 230739;

Json read_json(const std::string &path) {
  std::ifstream file(path);
  if (!file) throw std::runtime_error("Cannot read " + path);
  return Json::parse(file);
}
void save(const std::string &path, const Json &value) {
  std::ofstream file(path);
  if (!file) throw std::runtime_error("Cannot write " + path);
  file << value.dump(2) << '\n';
}

struct Pending {
  std::mutex mutex;
  std::condition_variable ready;
  bool done = false;
  Json reply;
};

template<class T>
Json read_value(const std::shared_ptr<com::master::InstructionServiceIface> &service,
                const std::string &section, const std::string &name, bool status) {
  std::shared_ptr<com::master::InstructionBatch> batch;
  if (!service->AllocateInstructionBatch(CLIENT, batch))
    throw std::runtime_error("Cannot allocate read batch");
  std::shared_ptr<com::master::InstructionIf> request;
  if (status) request = std::make_shared<com::master::GetStatusRequest<T>>(section, name);
  else request = std::make_shared<com::master::GetParamRequest<T>>(section, name);
  if (!batch->AddRequest(request)) throw std::runtime_error("Cannot add read " + name);
  auto pending = std::make_shared<Pending>();
  auto sent = service->SendInstructionBatch(batch,
    [pending,section,name](auto client, auto response) {
      Json reply = {{"client_id",client}};
      std::vector<std::shared_ptr<com::master::Response<T>>> values;
      if (response && response->template GetResponse<T>(section,name,values) && values.size()==1) {
        reply["response_type"] = values[0]->GetResponseType();
        reply["value"] = values[0]->GetValue();
      }
      {
        std::lock_guard<std::mutex> lock(pending->mutex);
        pending->reply = reply;
        pending->done = true;
      }
      pending->ready.notify_one();
    });
  if (sent != com::types::ERROR_CODE_OK) throw std::runtime_error("Cannot send read " + name);
  bool received;
  Json result;
  {
    std::unique_lock<std::mutex> lock(pending->mutex);
    received = pending->ready.wait_for(lock, 3s, [&]{return pending->done;});
    result = pending->reply;
  }
  service->ReleaseInstructionBatch(batch);
  if (!received || result.value("response_type",0)!=1 || !result.contains("value"))
    throw std::runtime_error("Read failed: " + name + ": " + result.dump());
  return result["value"];
}

Json snapshot(const std::shared_ptr<com::master::InstructionServiceIface> &service,
              const Json &definitions) {
  Json report = {{"transport","SocketCAN can0 / VN1630A CH1"}, {"parameters",Json::object()}, {"statuses",Json::object()}};
  auto get = [&](const std::string &name, const std::string &type, bool status) {
    const std::string section = status ? "auto_interface" : "auto_interface_0dim";
    if (type=="u8") return read_value<uint8_t>(service,section,name,status);
    if (type=="u16") return read_value<uint16_t>(service,section,name,status);
    if (type=="u32") return read_value<uint32_t>(service,section,name,status);
    if (type=="f32") return read_value<float>(service,section,name,status);
    throw std::runtime_error("Unsupported read type " + type);
  };
  report["statuses"]["product_serial"] = get("product_serial","u32",true);
  if (report["statuses"]["product_serial"] != CLIENT)
    throw std::runtime_error("Sensor serial mismatch; refusing maintenance");
  for (auto group : {"statuses","parameters"}) {
    for (const auto &definition : definitions.at(group)) {
      std::string name = definition.at("name");
      if (std::string(group)=="statuses" && name=="product_serial") continue;
      auto value = get(name,definition.at("type"),std::string(group)=="statuses");
      report[group][name] = value;
      std::cout << "READ " << name << "=" << value << std::endl;
    }
  }
  return report;
}

int bootloader_socket() {
  int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd<0) throw std::runtime_error("Cannot open bootloader monitor socket");
  ifreq iface{};
  std::snprintf(iface.ifr_name,sizeof(iface.ifr_name),"can0");
  if (ioctl(fd,SIOCGIFINDEX,&iface)<0) throw std::runtime_error("Cannot find can0");
  can_filter filter{0x320,CAN_SFF_MASK|CAN_EFF_FLAG|CAN_RTR_FLAG};
  if (setsockopt(fd,SOL_CAN_RAW,CAN_RAW_FILTER,&filter,sizeof(filter))<0)
    throw std::runtime_error("Cannot set bootloader filter");
  sockaddr_can address{};
  address.can_family=AF_CAN;
  address.can_ifindex=iface.ifr_ifindex;
  if (bind(fd,reinterpret_cast<sockaddr*>(&address),sizeof(address))<0)
    throw std::runtime_error("Cannot bind bootloader socket");
  return fd;
}

const char *update_status(com::types::SWUpdateStatus status) {
  switch(status) {
    case com::types::RUNNING:return "RUNNING";
    case com::types::READY_SUCCESS:return "READY_SUCCESS";
    case com::types::STOPPED_BY_MASTER:return "STOPPED_BY_MASTER";
    case com::types::STOPPED_BY_SLAVE:return "STOPPED_BY_SLAVE";
    case com::types::STOPPED_BY_ERROR_TIMEOUT:return "STOPPED_BY_ERROR_TIMEOUT";
    case com::types::STOPPED_BY_ERROR_BLOCK_REPEAT:return "STOPPED_BY_ERROR_BLOCK_REPEAT";
    case com::types::STOPPED_BY_ERROR_IMAGE_INVALID:return "STOPPED_BY_ERROR_IMAGE_INVALID";
    default:return "OTHER";
  }
}

int main(int argc, char **argv) {
  if (argc!=4 && argc!=5 && argc!=6) {
    std::cerr << "Usage: can_maintenance DEFINITIONS OUTPUT_DIR snapshot|update|retry-bootloader [ORIGINAL_XML] [PREVIOUS_SNAPSHOT]\n";
    return 1;
  }
  std::string directory=argv[2];
  bool transfer_started=false;
  try {
    auto config_path=std::getenv("SMART_ACCESS_CFG_FILE_PATH");
    if (!config_path) throw std::runtime_error("Private SDK config required");
    auto master=read_json(config_path);
    auto hardware=read_json(master.at("config_path").get<std::string>()+"/hw_inventory.json");
    auto routing=read_json(master.at("config_path").get<std::string>()+"/routing_table.json");
    if (master.at("alive")!=false || hardware.at("hwItems").size()!=1 || routing.at("clients").size()!=1 ||
        hardware["hwItems"][0].at("type")!="can" || hardware["hwItems"][0].at("iface_name")!="can0" ||
        routing["clients"][0].at("client_id")!=CLIENT || routing["clients"][0].at("can_network_id")!=0)
      throw std::runtime_error("Refusing unexpected maintenance route");
    auto definitions=read_json(argv[1]);
    const std::string action=argv[3];
    const bool recovery=action=="retry-bootloader";
    if (!((action=="snapshot" && argc==4) || (action=="update" && argc==5) || (recovery && argc==6)))
      throw std::runtime_error("Invalid maintenance action");
    auto services=com::master::CommunicationServicesIface::Get();
    if (!services->Init()) throw std::runtime_error("SDK Init failed");
    auto instructions=services->GetInstructionService();
    auto before=recovery ? read_json(argv[5]) : snapshot(instructions,definitions);
    if (before.at("statuses").at("product_serial")!=CLIENT)
      throw std::runtime_error("Previous sensor snapshot serial mismatch");
    if (recovery) {
      save(directory+"/snapshot-reference.json",{{"previous_snapshot",argv[5]}, {"snapshot",before}});
      std::cout << "RECOVERY: using verified pre-transfer snapshot; application reads unavailable in bootloader" << std::endl;
    } else save(directory+"/snapshot-before.json",before);
    if (action=="snapshot") return 0;
    if (before["statuses"]["sw_version_major"]!=5 || before["statuses"]["sw_version_minor"]!=2 || before["statuses"]["sw_version_patch"]!=2)
      throw std::runtime_error("Expected current firmware 5.2.2; refusing unexpected upgrade");
    auto update=services->GetUpdateService();
    if (!update) throw std::runtime_error("SDK update service unavailable");
    int monitor=bootloader_socket();
    std::cout << (recovery ? "ARMED: waiting up to 5 seconds for three fresh idle bootloader echoes; no update request sent yet"
                          : "ARMED: waiting up to 10 minutes for a fresh CAN 0x320 bootloader echo; no update request sent yet") << std::endl;
    auto deadline=std::chrono::steady_clock::now()+(recovery ? 5s : 600s);
    bool bootloader_seen=false;
    int idle_echoes=0;
    const unsigned char idle_echo[8]={0,0,0x40,0,0,0,0,0};
    while (std::chrono::steady_clock::now()<deadline) {
      fd_set ready;FD_ZERO(&ready);FD_SET(monitor,&ready);timeval timeout{1,0};
      if (select(monitor+1,&ready,nullptr,nullptr,&timeout)>0) {
        can_frame frame{};
        if (read(monitor,&frame,sizeof(frame))==sizeof(frame) && frame.can_id==0x320 && frame.can_dlc==8) {
          if (!recovery || std::memcmp(frame.data,idle_echo,8)==0) {
            if (++idle_echoes>=(recovery ? 3 : 1)) {
              bootloader_seen=true;
              break;
            }
          } else idle_echoes=0;
        }
      }
    }
    close(monitor);
    if (!bootloader_seen) throw std::runtime_error("No bootloader echo; no update attempted");
    std::cout << "BOOTLOADER_SEEN: starting original vendor XML update" << std::endl;
    // Once transfer starts, accidental terminal signals must not terminate the host.
    std::signal(SIGINT,SIG_IGN);
    std::signal(SIGTERM,SIG_IGN);
    auto pending=std::make_shared<Pending>();
    std::string image=argv[4];
    auto started=std::chrono::steady_clock::now();
    auto result=update->SoftwareUpdate(image,CLIENT,[pending,directory](auto &info){
      Json progress={{"status",update_status(info.GetUpdateStatus())},
                     {"status_code",info.GetUpdateStatus()},
                     {"segment",info.GetCurrentSegmentNumber()},
                     {"downloaded_bytes",info.GetCurrentDownloadedBytes()}};
      {
        std::lock_guard<std::mutex> lock(pending->mutex);
        pending->reply=progress;
        pending->done=info.GetUpdateStatus()!=com::types::RUNNING && info.GetUpdateStatus()!=com::types::NOT_VALID;
        try {
          save(directory+"/update-progress.json",progress);
        } catch (const std::exception &error) {
          // Logging must never interrupt the SDK's transfer state machine.
          std::cerr << "Progress file warning: " << error.what() << std::endl;
        }
      }
      std::cout << "UPDATE " << progress.dump() << std::endl;
      pending->ready.notify_one();
    });
    transfer_started=result==com::types::ERROR_CODE_OK;
    if (!transfer_started) throw std::runtime_error("SDK rejected update start: "+std::to_string(result));
    Json terminal;
    {
      std::unique_lock<std::mutex> lock(pending->mutex);
      // The vendor state machine controls its own protocol timeouts. Do not impose an outer abort.
      while (!pending->done) pending->ready.wait_for(lock,30s);
      terminal=pending->reply;
    }
    terminal["elapsed_s"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
    save(directory+"/update-terminal.json",terminal);
    std::cout << "TERMINAL " << terminal.dump() << std::endl;
    if (terminal["status_code"]!=com::types::READY_SUCCESS) return 3;
    std::this_thread::sleep_for(3s);
    for (int attempt=0;attempt<12;attempt++) {
      try {
        auto after=snapshot(instructions,definitions);
        save(directory+"/snapshot-after.json",after);
        if (after["statuses"]["sw_version_major"]!=5 || after["statuses"]["sw_version_minor"]!=2 || after["statuses"]["sw_version_patch"]!=4)
          throw std::runtime_error("Application readback did not confirm firmware 5.2.4");
        std::cout << "POST_UPDATE_SNAPSHOT_COMPLETE" << std::endl;
        return 0;
      } catch (const std::exception &error) {
        std::cout << "Waiting for application readback: " << error.what() << std::endl;
        std::this_thread::sleep_for(2s);
      }
    }
    throw std::runtime_error("SDK reported success but application readback did not return; leave powered");
  } catch (const std::exception &error) {
    save(directory+"/maintenance-error.json",{{"error",error.what()},{"transfer_started",transfer_started}});
    std::cerr << error.what() << std::endl;
    return 2;
  }
}
