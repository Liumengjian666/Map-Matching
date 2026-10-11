// P15 orchestration only. Author algorithms and P14 compatibility helpers unchanged.
#include "../p14_inlier_baseline/real_probe.hpp"
#include <map>

int main(int argc, char** argv) {
  try {
    if (argc != 2) throw std::runtime_error("Usage: inlier_causal_probe FROZEN_N_DIRECTORY");
    const std::string root = argv[1];
    if (std::ifstream(root+"/queries_result.csv").good())
      throw std::runtime_error("Do not repeat or overwrite frozen real query results");
    const auto refs = p14::readInputs(root+"/references.csv");
    const auto queries = p14::readInputs(root+"/queries.csv");
    const std::vector<int64_t> expected{16,45,708,1891,1994,4691};
    if (queries.size() != expected.size() || refs.empty() || refs.size() > 389)
      throw std::runtime_error("Frozen P15 input count mismatch");
    for (size_t i=0; i<expected.size(); ++i)
      if (queries[i].id != expected[i]) throw std::runtime_error("Fixed six query IDs mismatch");
    inlier::InLiERConfig grid;
    inlier::Encoder encoder(grid);
    inlier::Matcher matcher(grid);
    matcher.Reserve(refs.size());
    std::map<int64_t,p14::Encoded> db;  // preserve sparse original reference IDs
    auto rout=p14::output(root+"/reference_encoding.csv");
    rout << "id,stamp," << p14::frameHeader() << '\n';
    const long before=p14::rssKiB();
    const auto dbtick=p14::Clock::now();
    for (const auto& r:refs) {
      auto f=p14::encode(r.path,encoder);
      rout << r.id << ',' << r.stamp << ','; p14::frameFields(rout,f); rout << '\n';
      matcher.Add(r.id,f.encoded.tokens.token_id);
      db.emplace(r.id,std::move(f.encoded));
      if(db.size()%50==0) std::cout << "REFERENCE " << db.size() << std::endl;
    }
    matcher.Finalize();
    auto resources=p14::output(root+"/resources.csv");
    resources << "database_build_ms,rss_before_kib,rss_after_kib,peak_rss_kib\n"
              << p14::ms(dbtick) << ',' << before << ',' << p14::rssKiB() << ',' << p14::peakRssKiB() << '\n';
    auto out=p14::output(root+"/queries_result.csv");
    out << "id,stamp," << p14::frameHeader()
        << ",mint_count,mint_positive,beam_count,beam_positive,evidence_positive,verify_calls,ransac_entered,verify_success,legal_relative_poses,mint_ms,beam_ms,verify_ms,end_to_end_ms,cpu_ms,peak_rss_kib\n";
    auto candidates=p14::output(root+"/candidates.csv");
    candidates << "query_id,rank,reference_id,mint_score,beam_score,yaw_radians,azimuth_shift,correspondences,ransac_entered,fail_stage,ransac_inliers,keypoint_inliers,verify_success,legal_pose,inlier_rmse,verify_ms";
    for(int i=0;i<16;++i) candidates << ",T" << i;
    candidates << '\n';
    for(const auto& input:queries) {
      auto tick=p14::Clock::now(); const auto cpu=std::clock();
      auto f=p14::encode(input.path,encoder);
      auto phase=p14::Clock::now();
      auto mint=matcher.Shortlist(f.encoded.tokens.token_id,p14::mintConfig(),-1,-1);
      double mint_ms=p14::ms(phase); phase=p14::Clock::now();
      auto beam=matcher.BeamScore(f.encoded.tokens.token_id,mint.ids,p14::beamConfig(),-1,-1);
      double beam_ms=p14::ms(phase),verify_ms=0;
      int success=0,legal=0,entered=0,evidence=0;
      for(size_t i=0;i<beam.ids.size();++i) {
        const auto it=std::find(mint.ids.begin(),mint.ids.end(),beam.ids[i]);
        if(it==mint.ids.end()) throw std::runtime_error("BEAM ID missing from MINT pool");
        double m=mint.scores.at(static_cast<size_t>(it-mint.ids.begin()));
        if(!std::isfinite(m)||!std::isfinite(beam.scores[i])) throw std::runtime_error("Invalid retrieval score");
        evidence+=(m>0&&beam.scores[i]>0);
        // Keep author's top20 verification pipeline even for padded zero IDs;
        // zero IDs are never reported as positive retrieval evidence.
        const auto& r=db.at(beam.ids[i]); phase=p14::Clock::now();
        auto v=inlier::Verify(f.encoded.tokens.token_id,f.encoded.kp,r.tokens.token_id,r.kp,
                              beam.best_shifts[i],grid,p14::verifyConfig());
        double elapsed=p14::ms(phase); verify_ms+=elapsed;
        bool rigid=v.success&&p14::rigid(v.T_sensor)&&std::isfinite(v.inlier_rmse);
        if(v.success&&!rigid) throw std::runtime_error("Invalid successful relative pose");
        bool ransac=v.n_correspondences>=32;
        success+=v.success; legal+=rigid; entered+=ransac;
        candidates << input.id << ',' << i+1 << ',' << beam.ids[i] << ',' << m << ',' << beam.scores[i]
                   << ',' << beam.yaw_estimates[i] << ',' << beam.best_shifts[i] << ',' << v.n_correspondences
                   << ',' << ransac << ',' << v.fail_stage << ',' << v.ransac_inliers_found << ',' << v.n_keypoint_inliers
                   << ',' << v.success << ',' << rigid << ',' << v.inlier_rmse << ',' << elapsed;
        for(int row=0;row<4;++row) for(int col=0;col<4;++col) candidates << ',' << v.T_sensor(row,col);
        candidates << '\n';
      }
      double total=p14::ms(tick),cpu_ms=1000.*(std::clock()-cpu)/CLOCKS_PER_SEC;
      out << input.id << ',' << input.stamp << ','; p14::frameFields(out,f);
      out << ',' << mint.ids.size() << ',' << p14::positive(mint.scores) << ',' << beam.ids.size() << ','
          << p14::positive(beam.scores) << ',' << evidence << ',' << beam.ids.size() << ',' << entered << ','
          << success << ',' << legal << ',' << mint_ms << ',' << beam_ms << ',' << verify_ms << ','
          << total << ',' << cpu_ms << ',' << p14::peakRssKiB() << '\n';
      out.flush(); candidates.flush();
      std::cout << "QUERY " << input.id << " tokens=" << f.encoded.tokens.token_id.size()
                << " positive=" << evidence << " ransac=" << entered << " success=" << success << std::endl;
    }
    return 0;
  } catch(const std::exception& e) { std::cerr << "P15_FAIL " << e.what() << '\n'; return 1; }
}
