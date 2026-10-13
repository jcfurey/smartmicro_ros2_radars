// Read-only post-update verification using the SDK's unmodified port serializer.
#define main unused_can_maintenance_main
#include "can_maintenance.cpp"
#undef main

int main(int argc, char **argv) {
  if (argc!=3) return 1;
  try {
    const auto path=std::getenv("SMART_ACCESS_CFG_FILE_PATH");
    if (!path) throw std::runtime_error("Private SDK config required");
    const auto config=read_json(path);
    const auto directory=config.at("config_path").get<std::string>();
    const auto hardware=read_json(directory+"/hw_inventory.json");
    const auto routing=read_json(directory+"/routing_table.json");
    if (config.at("alive")!=false || config.at("instruction_serialization_type")!="port_based" ||
        hardware.at("hwItems").size()!=1 || routing.at("clients").size()!=1)
      throw std::runtime_error("Unexpected verification configuration");
    const auto iface=hardware["hwItems"][0];
    const auto peer=routing["clients"][0];
    if (iface.at("type")!="eth" || iface.at("iface_name")!="enp68s0f0" ||
        iface.at("ip_address")!="192.168.11.17" || iface.at("port")!=55558 ||
        peer.at("link_type")!="eth" || peer.at("ip")!="192.168.11.11" ||
        peer.at("port")!=55555 || peer.at("client_id")!=CLIENT ||
        peer.at("instruction_serialization_type")!="port_based")
      throw std::runtime_error("Unexpected verification destination");
    const auto services=com::master::CommunicationServicesIface::Get();
    if (!services->Init()) throw std::runtime_error("SDK Init failed");
    auto report=snapshot(services->GetInstructionService(),read_json(argv[1]));
    report["transport"]="Ethernet port_based / unmodified SDK";
    save(std::string(argv[2])+"/snapshot-ethernet-port.json",report);
    if (report["statuses"]["sw_version_major"]!=5 || report["statuses"]["sw_version_minor"]!=2 ||
        report["statuses"]["sw_version_patch"]!=4)
      throw std::runtime_error("Expected firmware 5.2.4");
    std::cout << "PORT_READBACK_COMPLETE" << std::endl;
    return 0;
  } catch (const std::exception &error) {
    save(std::string(argv[2])+"/port-readback-error.json",{{"error",error.what()}});
    std::cerr << error.what() << std::endl;
    return 2;
  }
}
