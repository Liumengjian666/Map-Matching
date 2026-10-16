// Auxiliary metadata-size report reusing the exact P54 makeDescriptor implementation.
#define main p54_budgeted_submap_probe_main
#include "p54_budgeted_submap_probe.cpp"
#undef main

int main(int argc, char **argv)
{
  try
  {
    if (argc != 5)
      throw std::runtime_error("usage: p54_tile_descriptor_profile SAMPLE_MANIFEST TILE_CATALOG OUTPUT_CSV OUTPUT_MANIFEST_JSON");
    const std::string sample_path = argv[1], catalog_path = argv[2], csv_path = argv[3], json_path = argv[4];
    const CsvTable samples = readCsv(sample_path);
    const CsvTable catalog_csv = readCsv(catalog_path);
    const auto catalog = readTileCatalog(catalog_csv);
    std::set<std::string> active_union;
    for (const auto &row : samples.rows)
      for (const auto &id : activeIds(samples, row)) active_union.insert(id);
    if (samples.rows.size() != 154 || active_union.size() != 51)
      throw std::runtime_error("frozen P54 expected 154 samples and the existing 51-Tile active union");
    std::ofstream csv(csv_path, std::ios::out | std::ios::binary);
    if (!csv) throw std::runtime_error("cannot create descriptor CSV: " + csv_path);
    csv << "tile_id,point_count,occupied_centroids,planar_surfels,descriptor_payload_bytes,sha256,descriptor_ms\n";
    std::size_t payload_sum = 0;
    const auto all_start = Clock::now();
    for (const auto &id : active_union)
    {
      const auto it = catalog.find(id);
      if (it == catalog.end()) throw std::runtime_error("active Tile missing from P47 catalog: " + id);
      const auto start = Clock::now();
      TileDescriptor d = makeDescriptor(id, it->second);
      const double elapsed = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
      payload_sum += d.payload_bytes;
      csv << id << ',' << d.point_count << ',' << d.occupied->size() << ',' << d.surfels->size() << ','
          << d.payload_bytes << ',' << it->second.sha256 << ',' << std::setprecision(10) << elapsed << '\n';
    }
    csv.close();
    const double total_ms = std::chrono::duration<double, std::milli>(Clock::now() - all_start).count();
    std::ofstream json(json_path, std::ios::out | std::ios::binary);
    if (!json) throw std::runtime_error("cannot create descriptor profile manifest: " + json_path);
    json << "{\n  \"protocol\": \"P54_TILE_DESCRIPTOR_PROFILE_V1\",\n"
         << "  \"gt_used\": false,\n  \"ndt_align_calls\": 0,\n"
         << "  \"tile_count\": " << active_union.size() << ",\n"
         << "  \"descriptor_payload_bytes_sum\": " << payload_sum << ",\n"
         << "  \"descriptor_build_total_ms\": " << std::setprecision(12) << total_ms << ",\n"
         << "  \"csv\": \"" << csv_path << "\"\n}\n";
    std::cout << "P54_TILE_DESCRIPTOR_PROFILE_PASS tiles=" << active_union.size()
              << " payload_bytes=" << payload_sum << " total_ms=" << total_ms << '\n';
    return 0;
  }
  catch (const std::exception &e)
  {
    std::cerr << "P54_TILE_DESCRIPTOR_PROFILE_ERROR: " << e.what() << '\n';
    return 1;
  }
}
