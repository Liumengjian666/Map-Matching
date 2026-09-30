// Offline sensor decoding only: pinned official RawData, no state/TF/deskew.
#include <velodyne_pointcloud/rawdata.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <array>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <cstdio>
#include <memory>
#include <dlfcn.h>

std::string verifiedDriverLibrary() {
  // Pinned GCC/Itanium ABI symbol identities. Check the actual global binding,
  // not merely whether the intended library appears in /proc/self/maps.
  std::string library;
  for(const char* symbol:{
      "p6_a3a_driver_source_sha256", "p6_a3a_calibration_source_sha256",
      "_ZN16velodyne_rawdata7RawDataC1Ev",
      "_ZN16velodyne_rawdata7RawData12setupOfflineENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_dd",
      "_ZN16velodyne_rawdata7RawData6unpackERKN13velodyne_msgs15VelodynePacket_ISaIvEEERNS_17DataContainerBaseERKN3ros4TimeE",
      "_ZN16velodyne_rawdata7RawData12unpack_vlp16ERKN13velodyne_msgs15VelodynePacket_ISaIvEEERNS_17DataContainerBaseERKN3ros4TimeE",
      "_ZN16velodyne_rawdata7RawData12buildTimingsEv",
      "_ZN16velodyne_rawdata7RawData13setParametersEdddd",
      "_ZN19velodyne_pointcloud11Calibration4readERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE"}) {
    Dl_info info{}; void* address=dlsym(RTLD_DEFAULT,symbol);
    if(!address || !dladdr(address,&info) || !info.dli_fname)
      throw std::runtime_error("runtime_decoder_symbol_unidentified");
    const std::string provider=info.dli_fname;
    if(provider.find("/libp6_a3a_pinned_velodyne_rawdata.so")==std::string::npos ||
        (!library.empty() && provider!=library))
      throw std::runtime_error("runtime_decoder_symbol_interposition_rejected");
    library=provider;
  }
  return library;
}
const char* loadedSourceIdentity(const char* symbol) {
  const auto function=reinterpret_cast<const char*(*)()>(dlsym(RTLD_DEFAULT,symbol));
  if(!function) throw std::runtime_error("runtime_decoder_source_identity_missing");
  return function();
}

struct PointRecord { double x,y,z,intensity; std::uint64_t stamp; };
static_assert(sizeof(PointRecord)==40,"raw timed binary layout");
class RawCollector : public velodyne_rawdata::DataContainerBase {
 public:
  RawCollector():DataContainerBase(200,.1,"","",0,1,true,384,0) {}
  std::uint64_t packet_stamp=0;
  std::vector<PointRecord> points;
  void newLine() override {}
  void addPoint(float x,float y,float z,std::uint16_t,std::uint16_t,
      float distance,float intensity,float time) override {
    if (!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)||
        !std::isfinite(intensity)||!std::isfinite(time)||time<0)
      throw std::runtime_error("invalid_decoded_sensor_point");
    if(distance<.1f || distance>200.f) return;
    // unpack is called with scan_start_time=packet.stamp. Thus time is the
    // official firing offset in seconds, not a large epoch float.
    const auto offset=std::llround(static_cast<double>(time)*1e9);
    if (offset<0 || offset>1306368) throw std::runtime_error("invalid_firing_offset");
    points.push_back({x,y,z,intensity,packet_stamp+static_cast<std::uint64_t>(offset)});
  }
};
int main(int argc,char** argv) { try {
  const auto library=verifiedDriverLibrary();
  if(argc==2 && std::string(argv[1])=="--identity") {
    std::cout<<"decoder_source_sha256="<<A3A_DECODER_SOURCE_SHA<<'\n'
      <<"driver_source_sha256="<<loadedSourceIdentity("p6_a3a_driver_source_sha256")<<'\n'
      <<"calibration_source_sha256="<<loadedSourceIdentity("p6_a3a_calibration_source_sha256")<<'\n';
    std::cout<<"driver_loaded_library="<<library<<'\n'; return 0;
  }
  if(argc!=5) throw std::runtime_error("usage: decoder bag calibration points.bin catalog.csv");
  const std::uint16_t endian=1;
  if(*reinterpret_cast<const unsigned char*>(&endian)!=1) throw std::runtime_error("little_endian_required");
  ros::Time::init();
  velodyne_rawdata::RawData decoder;
  if(decoder.setupOffline(argv[2],"VLP16",200,.1)!=0) throw std::runtime_error("calibration_failed");
  decoder.setParameters(.1,200,0,2*M_PI);
  rosbag::Bag bag(argv[1],rosbag::bagmode::Read);
  rosbag::View view(bag,rosbag::TopicQuery("/velodyne_packets"));
  const auto close_file=[](FILE* file){if(file) std::fclose(file);};
  std::unique_ptr<FILE,decltype(close_file)> binary(std::fopen(argv[3],"wbx"),close_file);
  std::unique_ptr<FILE,decltype(close_file)> catalog(std::fopen(argv[4],"wx"),close_file);
  if(!binary||!catalog) throw std::runtime_error("output_open_failed");
  std::fprintf(catalog.get(),"transaction_id,scan_start_ns,scan_end_ns,byte_offset,point_count,provenance\n");
  std::uint64_t tx=0,offset=0,last_end=0;
  RawCollector data;
  for(const auto& item:view) {
    auto scan=item.instantiate<velodyne_msgs::VelodyneScan>();
    if(!scan || scan->packets.empty() || scan->header.frame_id!="cmu_rc2_velodyne")
      throw std::runtime_error("unexpected_raw_sensor_definition");
    if(scan->header.stamp!=scan->packets.front().stamp) throw std::runtime_error("scan_origin_mismatch");
    data.points.clear(); std::uint64_t previous=0;
    for(const auto& packet:scan->packets) {
      const auto stamp=packet.stamp.toNSec();
      if(!stamp || stamp<=previous || packet.data[1205]!=34 || packet.data[1204]!=55)
        throw std::runtime_error("unsupported_packet_time_or_model_or_return_mode");
      previous=stamp;
      for(int block=0;block<12;++block)
        if(packet.data[100*block]!=255 || packet.data[100*block+1]!=238)
          throw std::runtime_error("invalid_packet_block");
      data.packet_stamp=stamp;
      decoder.unpack(packet,data,packet.stamp);
    }
    if(data.points.empty()) throw std::runtime_error("empty_decoded_scan");
    // Terminal is the last scheduled firing in the last packet, even when
    // its return has no valid range: (11*2+1)*55296+15*2304 ns.
    const std::uint64_t end=previous+1306368;
    std::uint64_t start=std::numeric_limits<std::uint64_t>::max();
    for(const auto& p:data.points) {start=std::min(start,p.stamp);if(p.stamp>end) throw std::runtime_error("point_after_terminal");}
    if(start>=end || end<=last_end) throw std::runtime_error("invalid_raw_scan_sequence");
    ++tx;
    if(std::fwrite(data.points.data(),40,data.points.size(),binary.get())!=data.points.size() ||
       std::fprintf(catalog.get(),"%llu,%llu,%llu,%llu,%zu,RAW_TIMED_SENSOR\n",
        static_cast<unsigned long long>(tx),static_cast<unsigned long long>(start),
        static_cast<unsigned long long>(end),static_cast<unsigned long long>(offset),data.points.size())<0)
      throw std::runtime_error("raw_export_write_failed");
    offset+=data.points.size()*40; last_end=end;
  }
  if(!tx) throw std::runtime_error("no_raw_scans");
  if(std::fflush(binary.get())!=0 || std::fflush(catalog.get())!=0)
    throw std::runtime_error("raw_export_flush_failed");
  std::cerr<<"RAW_TIMED_SENSOR scans="<<tx<<" points="<<offset/40<<" bytes="<<offset<<'\n';
  return 0;
} catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;} }
