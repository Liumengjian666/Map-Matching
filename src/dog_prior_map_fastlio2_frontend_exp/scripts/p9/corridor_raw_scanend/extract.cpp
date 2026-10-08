// Prospective raw extraction only: no adapter, TF, deskew, NDT, labels or GT.
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/serialization.h>
#include <sensor_msgs/Imu.h>
#include <velodyne_pointcloud/rawdata.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <sys/stat.h>

namespace {
void require(bool yes, const char* message) {
  if (!yes) throw std::runtime_error(message);
}
uint32_t firingNs(float seconds) {
  require(std::isfinite(seconds) && seconds >= 0, "invalid point firing time");
  const auto ns = std::llround(double(seconds) * 1e9);
  require(ns >= 0 && ns <= 1306368, "unsupported VLP16 firing time");
  // Pinned single-return VLP16: 24 firings, 16 lasers per firing.
  const auto firing = ns / 55296, laser = (ns % 55296) / 2304;
  require(firing < 24 && laser < 16 && ns == firing * 55296 + laser * 2304,
          "timing does not match pinned single-return firing schedule");
  return static_cast<uint32_t>(ns);
}
struct Point { float x,y,z; uint32_t offset; };
static_assert(sizeof(Point) == 16, "P7 packed point layout");
struct Container : velodyne_rawdata::DataContainerBase {
  Container() : DataContainerBase(1000., 0., "", "", 0, 1, true, 384, 0) {}
  std::vector<Point> points;
  uint64_t packet_offset = 0, emitted = 0, invalid = 0, zero = 0;
  void newLine() override {}
  void addPoint(float x,float y,float z,uint16_t ring,uint16_t,
                float distance,float intensity,float time) override {
    ++emitted;
    const uint64_t offset = packet_offset + firingNs(time);
    require(offset <= UINT32_MAX && ring < 16, "point time/ring outside protocol");
    if (!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)||
        !std::isfinite(distance)||!std::isfinite(intensity)) { ++invalid; return; }
    if (distance <= 0) { ++zero; return; }
    points.push_back({x,y,z,static_cast<uint32_t>(offset)});
  }
};
std::ofstream output(const std::string& dir,const std::string& name,bool binary=false) {
  std::ofstream f(dir+"/"+name,std::ios::out|(binary?std::ios::binary:std::ios::openmode(0)));
  require(bool(f),"cannot create output");
  f.exceptions(std::ios::badbit|std::ios::failbit); f<<std::setprecision(17); return f;
}
uint16_t u16(const uint8_t* p) { return uint16_t(p[0]) | uint16_t(p[1])<<8; }
uint32_t u32(const uint8_t* p) {
  return uint32_t(p[0]) | uint32_t(p[1])<<8 | uint32_t(p[2])<<16 | uint32_t(p[3])<<24;
}
void selfTest() {
  for(int f=0;f<24;++f) for(int d=0;d<16;++d) {
    const auto expected=f*55296+d*2304;
    require(firingNs(float((55.296e-6*f)+(2.304e-6*d)))==uint32_t(expected),"firing self-test");
  }
  for(float bad:{-1.f,1.f,.000001f,std::numeric_limits<float>::quiet_NaN()}) {
    bool rejected=false;try {firingNs(bad);}catch(const std::exception&){rejected=true;}
    require(rejected,"bad firing accepted");
  }
  Container c;c.packet_offset=2000000;c.addPoint(1,2,3,0,0,4,5,float(1306368e-9));
  require(c.points.at(0).offset==3306368,"packet-relative timing lost");
  c.addPoint(0,0,0,0,0,0,0,0);require(c.zero==1&&c.points.size()==1,"zero return");
  const Point p{1,2,3,123};unsigned char bytes[16];std::memcpy(bytes,&p,16);
  require(bytes[3]==63&&u32(bytes+12)==123,"little endian layout required");
  std::cout<<"RAW_TIMING_SELF_TEST=PASS\n";
}
void packetTest(const std::string& calibration) {
  velodyne_rawdata::RawData parser;
  require(parser.setupOffline(calibration,"VLP16",1000.,0.)==0,"test calibration");
  parser.setParameters(0.,1000.,0.,2*M_PI);
  velodyne_msgs::VelodynePacket pkt;pkt.stamp.fromNSec(1517157219000000123ULL);
  std::fill(pkt.data.begin(),pkt.data.end(),0);
  for(int b=0;b<12;++b){
    pkt.data[b*100]=0xff;pkt.data[b*100+1]=0xee;
    for(int j=0;j<32;++j){pkt.data[b*100+4+j*3]=0x88;pkt.data[b*100+5+j*3]=0x13;}
  }
  Container c;c.packet_offset=1327100;parser.unpack(pkt,c,pkt.stamp);
  require(c.points.size()==384,"synthetic packet point count");
  for(int j=0;j<384;++j){
    const auto& p=c.points[j];const int f=j/16,d=j%16;
    require(p.offset==uint32_t(1327100+f*55296+d*2304),"parser packet time origin");
    require(std::abs(std::sqrt(p.x*p.x+p.y*p.y+p.z*p.z)-10.)<1e-5,"parser metric distance");
  }
  require(std::abs(c.points[0].x-10*std::cos(M_PI/12))<1e-5&&
          std::abs(c.points[0].y)<1e-5&&std::abs(c.points[0].z+10*std::sin(M_PI/12))<1e-5,
          "parser ROS XYZ convention");
  std::cout<<"PINNED_PACKET_TEST=PASS\n";
}
void extract(const std::string& bag_path,const std::string& calibration,const std::string& dir) {
  const uint32_t one=1;require(*reinterpret_cast<const uint8_t*>(&one)==1,"little endian host required");
  // Exclusive directory is the one-shot guard. Never truncate an existing extraction.
  require(mkdir(dir.c_str(),0755)==0,"destination exists/unwritable; refusing repeated extraction");
  auto started=output(dir,"EXTRACTION_STARTED");started<<"P9_CORRIDOR01_RAW_SCANEND_V1\n";started.close();
  velodyne_rawdata::RawData parser;
  require(parser.setupOffline(calibration,"VLP16",1000.,0.)==0,"calibration failed");
  parser.setParameters(0.,1000.,0.,2*M_PI); // Decode all valid ranges; P9 filters later.
  Container data;
  auto points=output(dir,"raw_timed_points.bin",true), index=output(dir,"raw_timed_scan_index.csv");
  auto filter=output(dir,"filter_scans.csv"), imu=output(dir,"imu.csv");
  auto imu_payload=output(dir,"raw_imu_serialized.bin",true), imu_index=output(dir,"raw_imu_index.csv");
  auto diag=output(dir,"conversion_diagnostics.csv");
  index<<"transaction_id,scan_start_ns,scan_end_ns,cloud_byte_offset,cloud_point_count\n";
  filter<<"transaction_id,stamp_ns\n";imu<<"stamp_ns,ax,ay,az,gx,gy,gz\n";
  imu_index<<"message_index,header_seq,stamp_ns,bag_record_ns,frame_id,byte_offset,byte_count\n";
  diag<<"transaction_id,header_seq,header_stamp_ns,bag_record_ns,frame_id,packets,scan_start_ns,scan_end_ns,point_count,emitted,invalid,zero,not_emitted,packet_clock_max_error_ns,previous_scan_overlap_ns,point_min_offset_ns,point_max_offset_ns,packet_interval_min_ns,packet_interval_max_ns\n";
  rosbag::Bag bag(bag_path,rosbag::bagmode::Read);
  const uint64_t expected_scans=rosbag::View(bag,rosbag::TopicQuery("/velodyne_packets")).size();
  const uint64_t expected_imu=rosbag::View(bag,rosbag::TopicQuery("/imu/data")).size();
  require(expected_scans>0&&expected_imu>0,"required topic empty/missing");
  rosbag::View view(bag,rosbag::TopicQuery(std::vector<std::string>{"/velodyne_packets","/imu/data"}));
  uint64_t tx=0,ni=0,byte_offset=0,imu_byte_offset=0,previous_start=0,previous_end=0,previous_imu=0;
  for(const auto& msg:view) {
    if(msg.getTopic()=="/imu/data") {
      const auto m=msg.instantiate<sensor_msgs::Imu>();require(bool(m),"IMU type mismatch");
      const auto stamp=m->header.stamp.toNSec();
      require(stamp>previous_imu&&m->header.frame_id=="epson","IMU header contract");previous_imu=stamp;
      const double fields[]={m->linear_acceleration.x,m->linear_acceleration.y,m->linear_acceleration.z,
        m->angular_velocity.x,m->angular_velocity.y,m->angular_velocity.z};
      imu<<stamp;for(double v:fields){require(std::isfinite(v),"nonfinite IMU");imu<<','<<v;}imu<<'\n';
      // Preserve full raw payload (including orientation/covariances) without using it for state init.
      const auto len=ros::serialization::serializationLength(*m);std::vector<uint8_t> buffer(len);
      ros::serialization::OStream stream(buffer.data(),len);ros::serialization::serialize(stream,*m);
      imu_payload.write(reinterpret_cast<const char*>(buffer.data()),len);
      imu_index<<++ni<<','<<m->header.seq<<','<<stamp<<','<<msg.getTime().toNSec()<<','<<m->header.frame_id<<','<<imu_byte_offset<<','<<len<<'\n';
      imu_byte_offset+=len;continue;
    }
    const auto scan=msg.instantiate<velodyne_msgs::VelodyneScan>();require(bool(scan),"scan type mismatch");
    require(!scan->packets.empty()&&scan->header.frame_id=="cmu_rc2_velodyne","scan header contract");
    const auto start=scan->packets.front().stamp.toNSec();
    require(start==scan->header.stamp.toNSec()&&start>previous_start,"scan start not first sensor packet");
    data.points.clear();data.emitted=data.invalid=data.zero=0;
    uint64_t previous_packet=0,max_clock_error=0,min_interval=UINT64_MAX,max_interval=0;
    for(const auto& packet:scan->packets) {
      const auto stamp=packet.stamp.toNSec();require(stamp>=start&&stamp>previous_packet,"packet timestamp order");
      if(previous_packet){const auto dt=stamp-previous_packet;min_interval=std::min(min_interval,dt);max_interval=std::max(max_interval,dt);}
      previous_packet=stamp;
      require(packet.data[1205]==0x22&&packet.data[1204]==0x37,"only VLP16 single strongest return supported");
      for(int b=0;b<12;++b)require(u16(packet.data.data()+100*b)==0xeeff&&u16(packet.data.data()+100*b+2)<36000,"invalid packet block");
      const int64_t hour=3600000000000LL;
      int64_t gap=int64_t(stamp%hour)-int64_t(u32(packet.data.data()+1200))*1000;
      if(gap>hour/2)gap-=hour;
      if(gap< -hour/2)gap+=hour;
      max_clock_error=std::max(max_clock_error,uint64_t(std::llabs(gap)));
      require(std::llabs(gap)<=1000,"packet header does not match embedded sensor microsecond clock");
      data.packet_offset=stamp-start;
      // Local packet origin avoids converting a ~100ms offset through a float.
      parser.unpack(packet,data,packet.stamp);
    }
    require(!data.points.empty(),"empty scan must not silently disappear");
    uint32_t lo=UINT32_MAX,hi=0;
    for(const auto& p:data.points){lo=std::min(lo,p.offset);hi=std::max(hi,p.offset);}
    // Actual last scheduled firing, even if the last return has no usable XYZ.
    const uint64_t end=previous_packet+1306368;
    require(end>start&&end>previous_end&&uint64_t(hi)<=end-start,"scan end contract");
    ++tx;index<<tx<<','<<start<<','<<end<<','<<byte_offset<<','<<data.points.size()<<'\n';filter<<tx<<','<<end<<'\n';
    points.write(reinterpret_cast<const char*>(data.points.data()),data.points.size()*sizeof(Point));
    diag<<tx<<','<<scan->header.seq<<','<<scan->header.stamp.toNSec()<<','<<msg.getTime().toNSec()<<','<<scan->header.frame_id
      <<','<<scan->packets.size()<<','<<start<<','<<end<<','<<data.points.size()<<','<<data.emitted<<','<<data.invalid<<','<<data.zero
      <<','<<scan->packets.size()*384-data.emitted<<','<<max_clock_error<<','<<(previous_end>start?previous_end-start:0)
      <<','<<lo<<','<<hi<<','<<(min_interval==UINT64_MAX?0:min_interval)<<','<<max_interval<<'\n';
    byte_offset+=data.points.size()*sizeof(Point);previous_start=start;previous_end=end;
    if(tx%250==0)std::cerr<<"EXTRACTED scans="<<tx<<" IMU="<<ni<<'\n';
  }
  require(tx==expected_scans&&ni==expected_imu,"message accounting mismatch");
  points.close();index.close();filter.close();imu.close();imu_payload.close();imu_index.close();diag.close();
  auto done=output(dir,"EXTRACTION_COMPLETE");done<<"scans="<<tx<<" imu="<<ni<<" raw_bytes="<<byte_offset<<'\n';done.close();
  std::cout<<"RAW_EXTRACTION_COMPLETE scans="<<tx<<" imu="<<ni<<" raw_bytes="<<byte_offset<<'\n';
}
}
int main(int argc,char** argv) {
  try {
    ros::Time::init();
    if(argc==2&&std::string(argv[1])=="--self-test")selfTest();
    else if(argc==3&&std::string(argv[1])=="--packet-test")packetTest(argv[2]);
    else if(argc==4)extract(argv[1],argv[2],argv[3]);
    else throw std::runtime_error("usage: RAW_BAG CALIBRATION NEW_OUTPUT_DIR | --self-test");
    return 0;
  }catch(const std::exception& e){std::cerr<<"RAW_EXTRACTION_FAIL="<<e.what()<<'\n';return 1;}
}
