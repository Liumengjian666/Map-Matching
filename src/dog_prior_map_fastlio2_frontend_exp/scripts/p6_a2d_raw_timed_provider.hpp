// Included inside p6_i1, after the shared V2/V3 producer.
struct RawTimedAsset {
  std::uint64_t transaction_id=0,start_ns=0,end_ns=0,offset=0,count=0;
  fixed_lag::LidarCloudProvenance provenance=fixed_lag::LidarCloudProvenance::RAW_TIMED_SENSOR;
};
std::vector<RawTimedAsset> readRawTimedCatalog(const std::string& path) {
  std::ifstream input(path); std::string row;
  if(!input || !std::getline(input,row) || row!=
      "transaction_id,scan_start_ns,scan_end_ns,byte_offset,point_count,provenance")
    throw std::runtime_error("RAW_POINT_TIME_UNAVAILABLE:invalid_raw_timed_catalog_header");
  std::vector<RawTimedAsset> output;
  std::set<std::uint64_t> ids;
  while(std::getline(input,row)) {
    if(row.empty()) continue;
    const auto fields=split(row,',');
    if(fields.size()!=6) throw std::runtime_error("invalid_raw_timed_catalog_row");
    RawTimedAsset a;
    a.transaction_id=std::stoull(fields[0]); a.start_ns=std::stoull(fields[1]);
    a.end_ns=std::stoull(fields[2]); a.offset=std::stoull(fields[3]); a.count=std::stoull(fields[4]);
    bool known=false;
    for(auto p:{fixed_lag::LidarCloudProvenance::RAW_TIMED_SENSOR,
                fixed_lag::LidarCloudProvenance::SENSOR_LOCAL_ROTATION_ONLY,
                fixed_lag::LidarCloudProvenance::WINDOW_OWNED_SE3_DESKEW,
                fixed_lag::LidarCloudProvenance::LEGACY_STATE_DERIVED_SE3_DESKEW})
      if(fields[5]==fixed_lag::toString(p)) {a.provenance=p;known=true;}
    if(!known || !a.transaction_id || !a.start_ns || a.end_ns<=a.start_ns || !a.count ||
        !ids.insert(a.transaction_id).second || (!output.empty() && a.end_ns<=output.back().end_ns))
      throw std::runtime_error("invalid_raw_timed_catalog_identity");
    output.push_back(a);
  }
  return output;
}
bool loadRawTimedScan(const std::string& path,const RawTimedAsset& a,
    fixed_lag::RawTimedScan* output,std::string* reason) {
  auto fail=[&](const char* text){if(reason)*reason=text;return false;};
  if(!output) return fail("null_raw_timed_scan_output");
  if(!fixed_lag::rawDeskewInputAllowed(a.provenance,reason)) return false;
  const std::uint16_t endian=1;
  if(*reinterpret_cast<const unsigned char*>(&endian)!=1)
    return fail("raw_timed_format_requires_little_endian_decoder");
  std::ifstream input(path,std::ios::binary|std::ios::ate);
  if(!input) return fail("raw_timed_binary_open_failed");
  const auto size=input.tellg();
  constexpr std::uint64_t record_bytes=40; // x,y,z,intensity float64 + uint64 sensor ns.
  if(size<0 || a.offset>static_cast<std::uint64_t>(size) ||
      a.count>(static_cast<std::uint64_t>(size)-a.offset)/record_bytes ||
      a.count>std::numeric_limits<std::size_t>::max()/sizeof(TimedLidarPoint))
    return fail("raw_timed_scan_byte_range_invalid");
  input.seekg(a.offset);
  fixed_lag::RawTimedScan pending;
  pending.transaction_id=a.transaction_id; pending.scan_start_ns=a.start_ns;
  pending.scan_end_ns=a.end_ns; pending.provenance=a.provenance;
  pending.points.reserve(a.count);
  for(std::uint64_t i=0;i<a.count;++i) {
    double xyz_intensity[4]; std::uint64_t stamp;
    input.read(reinterpret_cast<char*>(xyz_intensity),32);
    input.read(reinterpret_cast<char*>(&stamp),8);
    if(!input) return fail("raw_timed_scan_truncated");
    if(!stamp) return fail("RAW_POINT_TIME_UNAVAILABLE");
    TimedLidarPoint p;
    p.position=Eigen::Vector3d(xyz_intensity[0],xyz_intensity[1],xyz_intensity[2]);
    p.intensity=xyz_intensity[3]; p.stamp_ns=stamp;
    if(stamp<a.start_ns || stamp>a.end_ns || !p.position.allFinite() || !std::isfinite(p.intensity))
      return fail("raw_timed_point_invalid");
    pending.points.push_back(p);
  }
  *output=std::move(pending); return true;
}
std::vector<fixed_lag::VisualMeasurementProvenance> readVisualProvenance(
    const std::string& path,const std::vector<VisualMeasurement>& visual) {
  using P=fixed_lag::VisualMeasurementProvenance;
  std::map<std::pair<std::uint64_t,std::uint64_t>,P> metadata;
  if(path!="NONE") {
    std::ifstream input(path); std::string row;
    if(!input || !std::getline(input,row) || row!="ref_ns,cur_ns,provenance")
      throw std::runtime_error("invalid_visual_provenance_header");
    while(std::getline(input,row)) {
      if(row.empty()) continue;
      const auto fields=split(row,',');
      if(fields.size()!=3) throw std::runtime_error("invalid_visual_provenance_row");
      P p=P::UNKNOWN; bool known=false;
      for(P value:{P::RAW_SENSOR_LOCAL_DEPTH,P::WINDOW_OWNED_DEPTH,P::LEGACY_STATE_DERIVED_DEPTH,P::UNKNOWN})
        if(fields[2]==fixed_lag::toString(value)){p=value;known=true;}
      if(!known || !metadata.emplace(std::make_pair(std::stoull(fields[0]),std::stoull(fields[1])),p).second)
        throw std::runtime_error("duplicate_or_invalid_visual_provenance");
    }
  }
  std::vector<P> result;
  for(const auto& v:visual) {
    const auto it=metadata.find({v.ref_ns,v.cur_ns});
    result.push_back(it==metadata.end()?P::UNKNOWN:it->second);
  }
  return result;
}
void runWindowOwnedExperimentalMode(const p4_i2::Inputs& inputs,
    const std::vector<ScanAsset>& assets,const std::string& raw_path,
    const std::string& map_path,const std::string& params_path,
    const std::string& trajectory_path,const std::string& diagnostics_path,
    const std::string& runtime_path,const std::vector<VisualMeasurement>& visual,
    std::uint64_t init_stamp,const std::string& profile,R2Policy policy,
    const std::string& catalog_path,const std::string& visual_provenance_path) {
  const auto catalog=readRawTimedCatalog(catalog_path);
  if(catalog.size()<assets.size()) throw std::runtime_error("raw_timed_catalog_too_short");
  WindowOwnedProducerInput owned;
  std::map<std::uint64_t,RawTimedAsset> index;
  for(std::size_t i=0;i<assets.size();++i) {
    const auto& a=catalog[i];
    if(a.transaction_id!=assets[i].transaction_id || a.end_ns!=assets[i].stamp_ns)
      throw std::runtime_error("raw_timed_vs_filter_scan_identity_mismatch");
    std::string reason;
    if(!fixed_lag::rawDeskewInputAllowed(a.provenance,&reason)) throw std::runtime_error(reason);
    index.emplace(a.transaction_id,a); owned.scan_start_ns.push_back(a.start_ns);
  }
  owned.provider=[&](const ScanAsset& a,fixed_lag::RawTimedScan* scan,std::string* reason) {
    return loadRawTimedScan(raw_path,index.at(a.transaction_id),scan,reason);
  };
  owned.visual_provenance=readVisualProvenance(visual_provenance_path,visual);
  if(profile=="floor01") p5_i1::requireFrozenMapSha256(map_path);
  else if(profile!="corridor01" || p5_i1::sha256File(map_path)!=
      "103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f")
    throw std::runtime_error("window_owned_map_identity_mismatch");
  Pose3d initial,extrinsic;
  const auto parameters=p4_i2::readParameters(params_path,&initial,&extrinsic);
  const auto target=loadTarget(map_path);
  if(profile=="floor01" && target->size()!=549606) throw std::runtime_error("V3_target_count_mismatch");
  std::ofstream trajectory(trajectory_path),diagnostics(diagnostics_path),runtime(runtime_path);
  if(!trajectory||!diagnostics||!runtime) throw std::runtime_error("V3_output_open_failed");
  const auto result=runFixedLagProducer(inputs,assets,parameters,initial,extrinsic,target,visual,
      [](const ScanAsset&)->Cloud::Ptr {throw std::runtime_error("V3_LEGACY_SOURCE_PROVIDER_FORBIDDEN");},
      trajectory,diagnostics,runtime,init_stamp,policy,{}, {},&owned);
  std::cout<<"FULL_FIXED_LAG_V3_EXPERIMENTAL_COMPLETE window_deskews="<<result.window_deskew_count
      <<" ndt_calls="<<result.ndt_calls<<" raw_timed_sha256="<<p5_i1::sha256File(raw_path)
      <<" catalog_sha256="<<p5_i1::sha256File(catalog_path)<<'\n';
}

void runRawTimedFormatFixture() {
  // Private, explicitly synthetic file; no prepared real asset is rewritten.
  const auto serial=std::chrono::steady_clock::now().time_since_epoch().count();
  const std::string path="/tmp/p6_a2d_raw_timed_fixture_"+std::to_string(serial)+".bin";
  const std::string catalog_path=path+".csv";
  const std::string visual_path=path+".visual.csv";
  auto writeBinary=[&](std::uint64_t stamp,bool truncated=false) {
    std::ofstream f(path,std::ios::binary|std::ios::trunc);
    const double values[4]={1,2,3,.4}; f.write(reinterpret_cast<const char*>(values),32);
    if(!truncated) f.write(reinterpret_cast<const char*>(&stamp),8);
    if(!f) throw std::runtime_error("raw_timed_fixture_write_failed");
  };
  try {
    writeBinary(1000000007);
    {std::ofstream f(catalog_path); f<<"transaction_id,scan_start_ns,scan_end_ns,byte_offset,point_count,provenance\n"
        <<"1,1000000000,1000000020,0,1,RAW_TIMED_SENSOR\n";}
    const auto assets=readRawTimedCatalog(catalog_path);
    if(assets.size()!=1) throw std::runtime_error("raw_timed_catalog_parse_failed");
    fixed_lag::RawTimedScan raw; std::string reason;
    if(!loadRawTimedScan(path,assets.front(),&raw,&reason) || raw.points.size()!=1 ||
        raw.points[0].stamp_ns!=1000000007 || raw.points[0].position.x()!=1)
      throw std::runtime_error("raw_timed_real_record_fields_not_preserved:"+reason);
    const auto before=raw;
    writeBinary(0);
    if(loadRawTimedScan(path,assets.front(),&raw,&reason) || reason!="RAW_POINT_TIME_UNAVAILABLE" ||
        raw.points[0].stamp_ns!=before.points[0].stamp_ns)
      throw std::runtime_error("raw_timed_missing_timestamp_or_atomicity_failed");
    writeBinary(1000000021);
    if(loadRawTimedScan(path,assets.front(),&raw,&reason) || reason!="raw_timed_point_invalid")
      throw std::runtime_error("raw_timed_interval_gate_failed");
    writeBinary(1000000007,true);
    if(loadRawTimedScan(path,assets.front(),&raw,&reason)) throw std::runtime_error("raw_timed_truncation_gate_failed");
    {std::ofstream f(visual_path);f<<"ref_ns,cur_ns,provenance\n1,2,LEGACY_STATE_DERIVED_DEPTH\n";}
    VisualMeasurement v;v.ref_ns=1;v.cur_ns=2;
    const auto p=readVisualProvenance(visual_path,{v});
    if(p[0]!=fixed_lag::VisualMeasurementProvenance::LEGACY_STATE_DERIVED_DEPTH ||
        readVisualProvenance("NONE",{v})[0]!=fixed_lag::VisualMeasurementProvenance::UNKNOWN)
      throw std::runtime_error("visual_metadata_parse_failed");
    std::cout<<"A2D_RAW_TIMED_FILE_FORMAT_AND_VISUAL_METADATA_PASS\n";
  } catch(...) {
    std::remove(path.c_str());std::remove(catalog_path.c_str());std::remove(visual_path.c_str());throw;
  }
  std::remove(path.c_str());std::remove(catalog_path.c_str());std::remove(visual_path.c_str());
}
