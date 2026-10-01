// Included inside p6_i1, after the shared V2/V3 producer.
// V3 has a real no-vision ablation. Keep the historical V1/V2 parser strict:
// only the V3 caller may pass the explicit sentinel "NONE".
std::vector<VisualMeasurement> readV3VisualMeasurements(const std::string& path) {
  if (path == "NONE") return {};
  return readI6dVisual(path);
}

struct RawTimedAsset {
  std::uint64_t transaction_id=0,start_ns=0,end_ns=0,offset=0,count=0;
  fixed_lag::LidarCloudProvenance provenance=fixed_lag::LidarCloudProvenance::RAW_TIMED_SENSOR;
};
std::vector<RawTimedAsset> readRawTimedCatalog(const std::string& path);
const std::map<std::string,std::string>& corridorRawTimeContract() {
  static const std::map<std::string,std::string> fields={
    {"point_time_field","packets[].stamp + official VLP16 firing offset"},
    {"point_time_datatype","ROS time(sec,nsec) + driver float32 seconds firing offset"},
    {"point_time_unit","absolute_sensor_nanoseconds"},
    {"point_time_reference","packet.stamp; packet-local scan_start_time"},
    {"point_time_conversion_formula","packet.stamp.toNSec()+llround(unpack(packet,scan_start_time=packet.stamp).time*1e9)"},
    {"packet_clock_provenance","recorded ROS packet.stamp; acquisition driver mode not recorded; host receive or GPS origin not independently proven"},
    {"hardware_clock_or_sync_accuracy_proven","false"},
    {"scan_start_semantic","minimum timestamp of decoded valid range returns"},
    {"scan_end_semantic","last_packet.stamp_ns + 1306368 ns; last scheduled VLP16 firing (11*2+1)*55296+15*2304"}};
  return fields;
}
std::uint64_t strictRawUnsigned(const std::string& value) {
  if(value.empty() || value.find_first_not_of("0123456789")!=std::string::npos)
    throw std::runtime_error("V3_RAW_INPUT_INVALID_UNSIGNED_VALUE");
  return std::stoull(value);
}
void requireRawTimedInputManifest(const std::string& raw_path,
    const std::string& catalog_path,const std::string& filter_path,
    const std::string& manifest_path) {
  std::ifstream input(manifest_path); std::string line;
  if(!input) throw std::runtime_error("V3_RAW_INPUT_MANIFEST_MISSING");
  std::map<std::string,std::string> fields;
  while(std::getline(input,line)) {
    const auto separator=line.find('=');
    if(separator==std::string::npos || !fields.emplace(line.substr(0,separator),line.substr(separator+1)).second)
      throw std::runtime_error("V3_RAW_INPUT_MANIFEST_INVALID");
  }
  for(const char* key:{"dataset","original_bag_absolute_path","original_bag_sha256", "raw_lidar_topic",
      "raw_lidar_message_type","sensor_frame_id","point_time_field","point_time_datatype",
      "point_time_unit","point_time_reference","point_time_conversion_formula","scan_start_semantic",
      "scan_end_semantic","scan_count","point_count","first_scan_start_ns","last_scan_end_ns",
      "calibration_path","calibration_sha256","export_script_git_sha","export_script_sha256",
      "driver_git_sha","driver_rawdata_sha256","driver_library_sha256","decoder_source_sha256",
      "decoder_binary_sha256","packet_clock_provenance","hardware_clock_or_sync_accuracy_proven"})
    if(fields[key].empty()) throw std::runtime_error(std::string("V3_RAW_INPUT_MANIFEST_INCOMPLETE:")+key);
  const std::map<std::string,std::string> pinned={
    {"dataset","SuperLoc Corridor01"},{"provenance","RAW_TIMED_SENSOR"},
    {"original_bag_sha256","c9e127402463eebee25c54bc967eb377e026bbbdfc979eae853a5af134180811"},
    {"calibration_sha256","171e5fbf3c17256ca1fdc4098a3d190f668d414b1212a8f90bbf0fe6956a11bc"},
    {"driver_git_sha","29abd0e1361cb7f5eda451d2b51c35eeca45e0d5"},
    {"raw_lidar_topic","/velodyne_packets"},{"raw_lidar_message_type","velodyne_msgs/VelodyneScan"},
    {"sensor_frame_id","cmu_rc2_velodyne"},
    {"GT_USED","false"},{"LEGACY_STATE_USED","false"},{"WINDOW_STATE_USED","false"},{"DESKEW_PERFORMED","false"}};
  for(const auto& field:pinned) if(fields[field.first]!=field.second)
    throw std::runtime_error("V3_RAW_INPUT_MANIFEST_IDENTITY_MISMATCH:"+field.first);
  for(const auto& field:corridorRawTimeContract()) if(fields[field.first]!=field.second)
    throw std::runtime_error("V3_RAW_INPUT_TIME_SEMANTICS_MISMATCH:"+field.first);
  for(const auto& field:fields) {
    const bool digest=field.first.size()>=7 && field.first.substr(field.first.size()-7)=="_sha256";
    const bool revision=field.first=="export_script_git_sha" || field.first=="driver_git_sha";
    if((digest || revision) && (field.second.size()!=(digest?64:40) ||
        field.second.find_first_not_of("0123456789abcdef")!=std::string::npos))
      throw std::runtime_error("V3_RAW_INPUT_INVALID_IDENTITY_HASH:"+field.first);
  }
  for(const auto& file:std::vector<std::pair<std::string,std::string>>{
      {"raw_timed_points",raw_path},{"raw_timed_catalog",catalog_path},{"filter_scans",filter_path}}) {
    const auto slash=file.second.find_last_of('/');
    const auto filename=file.second.substr(slash==std::string::npos?0:slash+1);
    if(fields[file.first+"_file"]!=filename || fields[file.first+"_sha256"]!=p5_i1::sha256File(file.second))
      throw std::runtime_error("V3_RAW_INPUT_MANIFEST_SHA_MISMATCH:"+file.first);
  }
  const auto catalog=readRawTimedCatalog(catalog_path);
  std::uint64_t points=0,bytes=0;
  std::ifstream schedule(filter_path); std::string row;
  if(!std::getline(schedule,row) || row!="transaction_id,stamp_ns")
    throw std::runtime_error("V3_RAW_SENSOR_SCHEDULE_INVALID");
  for(std::size_t i=0;i<catalog.size();++i) {
    const auto& a=catalog[i];
    if(a.transaction_id!=i+1 || a.offset!=bytes ||
        a.provenance!=fixed_lag::LidarCloudProvenance::RAW_TIMED_SENSOR ||
        a.count>(std::numeric_limits<std::uint64_t>::max()-bytes)/40)
      throw std::runtime_error("V3_RAW_INPUT_CATALOG_COUNTS_INVALID");
    points+=a.count; bytes+=40*a.count;
    if(!std::getline(schedule,row)) throw std::runtime_error("V3_RAW_SENSOR_SCHEDULE_TOO_SHORT");
    const auto values=split(row,',');
    if(values.size()!=2 || strictRawUnsigned(values[0])!=a.transaction_id || strictRawUnsigned(values[1])!=a.end_ns)
      throw std::runtime_error("V3_RAW_SENSOR_SCHEDULE_TIMESTAMP_MISMATCH");
  }
  if(std::getline(schedule,row)) throw std::runtime_error("V3_RAW_SENSOR_SCHEDULE_TOO_LONG");
  std::ifstream binary(raw_path,std::ios::binary|std::ios::ate);
  if(catalog.empty() || strictRawUnsigned(fields["scan_count"])!=catalog.size() ||
      strictRawUnsigned(fields["point_count"])!=points ||
      strictRawUnsigned(fields["first_scan_start_ns"])!=catalog.front().start_ns ||
      strictRawUnsigned(fields["last_scan_end_ns"])!=catalog.back().end_ns ||
      binary.tellg()<0 || static_cast<std::uint64_t>(binary.tellg())!=bytes)
    throw std::runtime_error("V3_RAW_INPUT_MANIFEST_COUNTS_OR_BOUNDS_MISMATCH");
}
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
    a.transaction_id=strictRawUnsigned(fields[0]); a.start_ns=strictRawUnsigned(fields[1]);
    a.end_ns=strictRawUnsigned(fields[2]); a.offset=strictRawUnsigned(fields[3]); a.count=strictRawUnsigned(fields[4]);
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
std::vector<ScanAsset> readRawSensorScanAssets(const std::string& catalog_path) {
  const auto catalog=readRawTimedCatalog(catalog_path);
  if(catalog.empty()) throw std::runtime_error("empty_raw_sensor_schedule");
  std::vector<ScanAsset> assets;
  for(const auto& raw:catalog) {
    if(raw.transaction_id!=assets.size()+1)
      throw std::runtime_error("raw_sensor_transaction_sequence_invalid");
    std::string reason;
    if(!fixed_lag::rawDeskewInputAllowed(raw.provenance,&reason)) throw std::runtime_error(reason);
    ScanAsset asset;
    asset.transaction_id=raw.transaction_id; asset.stamp_ns=raw.end_ns;
    asset.time_s=static_cast<double>(raw.end_ns-catalog.front().start_ns)*1e-9;
    asset.cloud_byte_offset=raw.offset; asset.cloud_point_count=raw.count;
    asset.expected_source_hash_available=false;
    assets.push_back(asset);
  }
  return assets;
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
  std::uint64_t minimum_stamp=std::numeric_limits<std::uint64_t>::max();
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
    minimum_stamp=std::min(minimum_stamp,stamp);
    pending.points.push_back(p);
  }
  if(minimum_stamp!=a.start_ns) return fail("raw_scan_start_not_min_point_time");
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
std::ofstream openV3ExclusiveOutput(const std::string& path) {
  // Linux runner: O_EXCL via fopen("wx") reserves a new inode. The stream
  // opens that inode through its fd, never a replaced pathname. Existing
  // inputs, symlinks, hardlinks and earlier outputs cannot be truncated.
  FILE* reservation=std::fopen(path.c_str(),"wx");
  if(!reservation) throw std::runtime_error("V3_OUTPUT_EXISTS_OR_OPEN_FAILED:"+path);
  const std::string fd_path="/proc/self/fd/"+std::to_string(fileno(reservation));
  std::ofstream stream(fd_path);
  std::fclose(reservation);
  if(!stream) throw std::runtime_error("V3_OUTPUT_FD_OPEN_FAILED:"+path);
  return stream;
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
  auto trajectory=openV3ExclusiveOutput(trajectory_path);
  auto diagnostics=openV3ExclusiveOutput(diagnostics_path);
  auto runtime=openV3ExclusiveOutput(runtime_path);
  auto deskew_evidence=openV3ExclusiveOutput(trajectory_path+".deskew_evidence.csv");
  const char* diagnostic_environment=std::getenv("P6_A3B_R1_DIAGNOSTICS");
  const bool diagnostic_enabled=diagnostic_environment &&
      std::string(diagnostic_environment)=="1";
  fixed_lag::FixedLagOptions options;
  // Explicit V3 estimator contract, not an environment-selected backend.
  options.marginalization_backend=fixed_lag::MarginalizationBackend::SQUARE_ROOT_QR;
  std::cout << "marginalization_backend=SQUARE_ROOT_QR\n";
  options.marginal_covariance_backend=fixed_lag::MarginalCovarianceBackend::SQUARE_ROOT_QR;
  std::cout << "marginal_covariance_backend=SQUARE_ROOT_QR\n";
  const char* covariance_environment=std::getenv("P6_A3F_R1_COVARIANCE_DIAGNOSTICS");
  options.capture_covariance_shadow=covariance_environment && std::string(covariance_environment)=="1";
  std::ofstream covariance_requests,covariance_comparisons;
  if (options.capture_covariance_shadow) {
    covariance_requests=openV3ExclusiveOutput(trajectory_path+".a3f_r1_covariance.csv");
    covariance_comparisons=openV3ExclusiveOutput(trajectory_path+".a3f_r1_comparison.csv");
  }
  options.capture_optimizer_trace=diagnostic_enabled;
  const char* marginalization_environment =
      std::getenv("P6_A3C_R1_MARGINALIZATION_DIAGNOSTICS");
  const bool marginalization_diagnostic_enabled = marginalization_environment &&
      std::string(marginalization_environment) == "1";
  options.capture_marginalization_diagnostics =
      marginalization_diagnostic_enabled;
  std::ofstream preopt_capsule,optimizer_trace,directional_derivative,
      damping_sweep,optimizer_failure_summary,marginalization_trace,
      marginalization_failure_summary;
  if (diagnostic_enabled) {
    preopt_capsule=openV3ExclusiveOutput(trajectory_path+".r1_preopt_capsule.csv");
    optimizer_trace=openV3ExclusiveOutput(trajectory_path+".r1_optimizer_trace.csv");
    directional_derivative=openV3ExclusiveOutput(trajectory_path+".r1_directional_derivative.csv");
    damping_sweep=openV3ExclusiveOutput(trajectory_path+".r1_damping_sweep.csv");
    optimizer_failure_summary=openV3ExclusiveOutput(trajectory_path+".r1_failure_summary.txt");
  }
  const std::string marginalization_failure_capsule_path =
      trajectory_path + ".a3c_r1_failure_capsule.bin";
  if (marginalization_diagnostic_enabled) {
    marginalization_trace=openV3ExclusiveOutput(
        trajectory_path+".a3c_r1_marginalization_trace.csv");
    marginalization_failure_summary=openV3ExclusiveOutput(
        trajectory_path+".a3c_r1_failure_summary.txt");
  }
  const auto result=runFixedLagProducer(inputs,assets,parameters,initial,extrinsic,target,visual,
      [](const ScanAsset&)->Cloud::Ptr {throw std::runtime_error("V3_LEGACY_SOURCE_PROVIDER_FORBIDDEN");},
      trajectory,diagnostics,runtime,init_stamp,policy,{}, options,&owned,
      fixed_lag::LidarCloudProvenance::WINDOW_OWNED_SE3_DESKEW,&deskew_evidence,
      diagnostic_enabled?&preopt_capsule:nullptr,
      diagnostic_enabled?&optimizer_trace:nullptr,
      diagnostic_enabled?&directional_derivative:nullptr,
      diagnostic_enabled?&damping_sweep:nullptr,
      diagnostic_enabled?&optimizer_failure_summary:nullptr,
      marginalization_diagnostic_enabled?&marginalization_trace:nullptr,
      marginalization_diagnostic_enabled?&marginalization_failure_summary:nullptr,
      marginalization_diagnostic_enabled?marginalization_failure_capsule_path:
          std::string(),
      options.capture_covariance_shadow?&covariance_requests:nullptr,
      options.capture_covariance_shadow?&covariance_comparisons:nullptr);
  trajectory.flush(); diagnostics.flush(); runtime.flush(); deskew_evidence.flush();
  if (marginalization_diagnostic_enabled) marginalization_trace.flush();
  std::cout<<"FULL_FIXED_LAG_V3_EXPERIMENTAL_COMPLETE window_deskews="<<result.window_deskew_count
      <<" raw_scans_before_handoff="<<result.raw_scans_before_handoff
      <<" ndt_calls="<<result.ndt_calls<<" raw_timed_sha256="<<p5_i1::sha256File(raw_path)
      <<" catalog_sha256="<<p5_i1::sha256File(catalog_path)<<'\n';
}

void runRawTimedFormatFixture() {
  // Private, explicitly synthetic file; no prepared real asset is rewritten.
  const auto serial=std::chrono::steady_clock::now().time_since_epoch().count();
  const std::string path="/tmp/p6_a2d_raw_timed_fixture_"+std::to_string(serial)+".bin";
  const std::string catalog_path=path+".csv";
  const std::string visual_path=path+".visual.csv";
  const std::string filter_path=path+".filter.csv";
  const std::string manifest_path=path+".manifest.txt";
  auto writeBinary=[&](std::uint64_t stamp,bool truncated=false) {
    std::ofstream f(path,std::ios::binary|std::ios::trunc);
    const double values[4]={1,2,3,.4}; f.write(reinterpret_cast<const char*>(values),32);
    if(!truncated) f.write(reinterpret_cast<const char*>(&stamp),8);
    if(!f) throw std::runtime_error("raw_timed_fixture_write_failed");
  };
  try {
    writeBinary(1000000000);
    {std::ofstream f(catalog_path); f<<"transaction_id,scan_start_ns,scan_end_ns,byte_offset,point_count,provenance\n"
        <<"1,1000000000,1000000020,0,1,RAW_TIMED_SENSOR\n";}
    const auto assets=readRawTimedCatalog(catalog_path);
    if(assets.size()!=1) throw std::runtime_error("raw_timed_catalog_parse_failed");
    fixed_lag::RawTimedScan raw; std::string reason;
    if(!loadRawTimedScan(path,assets.front(),&raw,&reason) || raw.points.size()!=1 ||
        raw.points[0].stamp_ns!=1000000000 || raw.points[0].position.x()!=1)
      throw std::runtime_error("raw_timed_real_record_fields_not_preserved:"+reason);
    auto early_start=assets.front(); --early_start.start_ns;
    if(loadRawTimedScan(path,early_start,&raw,&reason) || reason!="raw_scan_start_not_min_point_time")
      throw std::runtime_error("raw_timed_false_scan_start_semantic_accepted");
    {
      std::ofstream f(path,std::ios::binary|std::ios::trunc);
      const double values[4]={1,2,3,.4};
      for(std::uint64_t stamp:{1000000000ULL,1000000003ULL,1000000019ULL}) {
        f.write(reinterpret_cast<const char*>(values),32);
        f.write(reinterpret_cast<const char*>(&stamp),8);
      }
    }
    auto nonuniform=assets.front(); nonuniform.count=3;
    if(!loadRawTimedScan(path,nonuniform,&raw,&reason) || raw.points.size()!=3 ||
        raw.points[1].stamp_ns-raw.points[0].stamp_ns!=3 ||
        raw.points[2].stamp_ns-raw.points[1].stamp_ns!=16)
      throw std::runtime_error("nonuniform_point_times_not_preserved");
    nonuniform.provenance=fixed_lag::LidarCloudProvenance::LEGACY_STATE_DERIVED_SE3_DESKEW;
    if(loadRawTimedScan(path,nonuniform,&raw,&reason)) throw std::runtime_error("raw_reader_accepted_legacy_provenance");
    writeBinary(1000000000);
    if(!loadRawTimedScan(path,assets.front(),&raw,&reason)) throw std::runtime_error(reason);
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
    VisualMeasurement missing=v; missing.cur_ns=3;
    if(readVisualProvenance(visual_path,{v,missing})[1]!=fixed_lag::VisualMeasurementProvenance::UNKNOWN)
      throw std::runtime_error("missing_sidecar_row_auto_upgraded");
    const auto raw_schedule=readRawSensorScanAssets(catalog_path);
    if(raw_schedule[0].stamp_ns!=1000000020 || raw_schedule[0].expected_source_hash_available)
      throw std::runtime_error("raw_schedule_not_sensor_owned");
    writeBinary(1000000000);
    {std::ofstream f(filter_path);f<<"transaction_id,stamp_ns\n1,1000000020\n";}
    auto writeManifest=[&](const std::map<std::string,std::string>& overrides) {
      std::ofstream f(manifest_path);
      f<<"dataset=SuperLoc Corridor01\nprovenance=RAW_TIMED_SENSOR\n"
       <<"original_bag_sha256=c9e127402463eebee25c54bc967eb377e026bbbdfc979eae853a5af134180811\n"
       <<"calibration_sha256=171e5fbf3c17256ca1fdc4098a3d190f668d414b1212a8f90bbf0fe6956a11bc\n"
       <<"driver_git_sha=29abd0e1361cb7f5eda451d2b51c35eeca45e0d5\n";
      f<<"raw_lidar_topic=/velodyne_packets\nraw_lidar_message_type=velodyne_msgs/VelodyneScan\nsensor_frame_id=cmu_rc2_velodyne\n";
      auto metadata=corridorRawTimeContract();
      metadata["original_bag_absolute_path"]="SYNTHETIC_FIXTURE_NOT_REAL_SENSOR_EVIDENCE";
      metadata["calibration_path"]="SYNTHETIC_FIXTURE_NOT_REAL_SENSOR_EVIDENCE";
      metadata["scan_count"]="1"; metadata["point_count"]="1";
      metadata["first_scan_start_ns"]="1000000000"; metadata["last_scan_end_ns"]="1000000020";
      metadata["export_script_git_sha"]=std::string(40,'0');
      for(const char* key:{"export_script_sha256","driver_rawdata_sha256","driver_library_sha256",
          "decoder_source_sha256","decoder_binary_sha256"}) metadata[key]=std::string(64,'0');
      for(const auto& field:overrides) metadata[field.first]=field.second;
      for(const auto& field:metadata) f<<field.first<<'='<<field.second<<'\n';
      for(const char* key:{"GT_USED","LEGACY_STATE_USED","WINDOW_STATE_USED","DESKEW_PERFORMED"}) f<<key<<"=false\n";
      for(const auto& file:std::vector<std::pair<std::string,std::string>>{
          {"raw_timed_points",path},{"raw_timed_catalog",catalog_path},{"filter_scans",filter_path}})
        f<<file.first<<"_file="<<file.second.substr(file.second.find_last_of('/')+1)<<'\n'
         <<file.first<<"_sha256="<<p5_i1::sha256File(file.second)<<'\n';
    };
    writeManifest({});
    requireRawTimedInputManifest(path,catalog_path,filter_path,manifest_path);
    for(const auto& change:std::vector<std::pair<std::string,std::string>>{
        {"scan_count","2"},{"point_count","2"},{"last_scan_end_ns","1000000021"},
        {"hardware_clock_or_sync_accuracy_proven","true"},{"point_time_unit","guessed_seconds"},
        {"export_script_git_sha","not_a_git_sha"}}) {
      writeManifest({change}); bool caught=false;
      try {requireRawTimedInputManifest(path,catalog_path,filter_path,manifest_path);}
      catch(const std::runtime_error&) {caught=true;}
      if(!caught) throw std::runtime_error("V3_manifest_semantic_corruption_not_rejected:"+change.first);
    }
    writeManifest({});
    const auto original_sha=p5_i1::sha256File(path); bool overwrite_rejected=false;
    try {auto stream=openV3ExclusiveOutput(path);}
    catch(const std::runtime_error&) {overwrite_rejected=true;}
    if(!overwrite_rejected || p5_i1::sha256File(path)!=original_sha)
      throw std::runtime_error("V3_output_overwrote_admitted_input");
    {std::fstream f(path,std::ios::in|std::ios::out|std::ios::binary); const double finite_modified_x=9;
      f.write(reinterpret_cast<const char*>(&finite_modified_x),8);}
    bool rejected=false;
    try {requireRawTimedInputManifest(path,catalog_path,filter_path,manifest_path);}
    catch(const std::runtime_error& e) {rejected=std::string(e.what()).find("SHA_MISMATCH")!=std::string::npos;}
    if(!rejected) throw std::runtime_error("V3_manifest_did_not_reject_finite_modified_raw_geometry");
    std::cout<<"A2D_RAW_TIMED_FILE_FORMAT_AND_VISUAL_METADATA_PASS\n";
  } catch(...) {
    std::remove(filter_path.c_str());std::remove(manifest_path.c_str());
    std::remove(path.c_str());std::remove(catalog_path.c_str());std::remove(visual_path.c_str());throw;
  }
  std::remove(path.c_str());std::remove(catalog_path.c_str());std::remove(visual_path.c_str());
  std::remove(filter_path.c_str());std::remove(manifest_path.c_str());
}
