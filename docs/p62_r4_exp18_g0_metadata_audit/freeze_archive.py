"""Validate recorded identities and freeze small R4 artifacts; no localization."""
import csv
import datetime
import hashlib
import json
import platform
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from assets import DOC,BASE,guard


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    guard()
    before=json.loads((DOC/'asset_identity_before.json').read_text())
    after=json.loads((DOC/'asset_identity_after.json').read_text())
    if before!=after or len(before['assets'])!=5:raise RuntimeError('ASSET_SNAPSHOT_CONFLICT')
    bag=json.loads((DOC/'bag_summary.json').read_text());e57=json.loads((DOC/'e57_summary.json').read_text())
    derived=json.loads((DOC/'derived_metadata.json').read_text())
    if not bag['completed'] or not e57['completed']:raise RuntimeError('INCOMPLETE_PARSING')
    if (bag['messages_deserialized'],bag['scans'],bag['imu_messages'],e57['decoded_points'])!=(66633,1094,43656,285455240):
        raise RuntimeError('COUNT_CONFLICT')
    with (DOC/'topic_inventory.csv').open() as handle:inventory=list(csv.DictReader(handle))
    if sum(int(r['message_count']) for r in inventory)!=bag['messages_deserialized']:
        raise RuntimeError('TOPIC_COUNT_CONFLICT')
    env=json.loads((DOC/'parser_environment.json').read_text())
    software_after=[]
    for row in env['pye57_files']:
        path=Path(row['path']);actual=digest(path)
        if actual!=row['sha256'] or path.stat().st_size!=row['bytes']:
            raise RuntimeError('PARSER_FILE_CHANGED:'+str(path))
        software_after.append({'path':str(path),'bytes':path.stat().st_size,'sha256':actual,'status':'UNCHANGED'})
    for row in env['sources']:
        if row.get('status')=='UNAVAILABLE':continue
        path=Path(row['path'])
        if digest(path)!=row['sha256']:raise RuntimeError('SOURCE_SNAPSHOT_CHANGED')
    platform_page=BASE/'p62_r1_source_cache/hilti_page.html'
    platform_sha=digest(platform_page)
    if platform_sha!='9dad17cabe011d5014987f3150154e0b806210eb011ee26fdbbad1e4b74497bc':
        raise RuntimeError('R1_FROZEN_PLATFORM_PAGE_CHANGED')
    platform_text=' '.join(platform_page.read_text().split())
    if 'synchronized via PTP' not in platform_text or 'aligned to within 1 ms' not in platform_text:
        raise RuntimeError('PLATFORM_SYNCHRONIZATION_TEXT_NOT_PRESENT')
    platform_evidence={'path':str(platform_page),'bytes':platform_page.stat().st_size,
        'sha256':platform_sha,'source':'https://hilti-challenge.com/dataset-2022',
        'frozen_identity_source':'R1 source_manifest.json hilti_page.html',
        'short_verbatim_fragments':['synchronized via PTP','aligned to within 1 ms'],
        'qualification':'Official platform-level description; not an Exp18 session synchronization log or measured error certificate'}
    (DOC/'platform_sync_evidence.json').write_text(json.dumps(platform_evidence,indent=2)+'\n')
    post_sha=digest(DOC/'summarize_metadata.py')
    if post_sha!='96fd708a37231d289bea558ae3bd1e3e9a1c88f0650d36763f5a783dc796877c':
        raise RuntimeError('POSTPROCESSOR_SOURCE_CHANGED')
    (DOC/'parser_identity_after.json').write_text(json.dumps({'files':software_after},indent=2)+'\n')
    storage={'startup_free_bytes':34101174272,'startup_required_bytes':30*1024**3,
        'runtime_required_bytes':20*1024**3,'runtime_receipt_margin_bytes':1024**2,
        'final_free_bytes':shutil.disk_usage(BASE).free,'conversion_export_bytes':0,
        'new_dependencies_installed':False,'new_venv_created':False,
        'hard_deadline_utc':'2026-10-19T14:14:00+00:00',
        'first_confirmed_clock_utc':'2026-10-19T12:15:22+00:00',
        'archive_clock_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'exact_overall_wall_seconds':'NOT_RECORDED; timestamps show completion well within 120 minutes',
        'bag_parse_wall_s':bag['wall_s'],'e57_parse_wall_s':e57['wall_s'],
        'bag_getrusage_maxrss_KiB':bag['peak_RSS_KiB'],'e57_getrusage_maxrss_KiB':e57['peak_RSS_KiB'],
        'memory_not_summed':True,'not_a_localization_realtime_benchmark':True}
    (DOC/'storage_audit.json').write_text(json.dumps(storage,indent=2)+'\n')
    ros_package=Path('/opt/ros/noetic/share/rosbag/package.xml')
    runtime={'python':sys.version,'platform':platform.platform(),'rosbag_package_version':ET.parse(ros_package).getroot().findtext('version'),
        'rosbag_package_xml_sha256':digest(ros_package),'libe57_reader_library_id':'pye57-0.4.5',
        'e57_file_writer_library_id':json.loads((DOC/'e57_structure.json').read_text())['e57LibraryVersion'],
        'reader_and_writer_not_conflated':True,'parser_software_files_before_after_unchanged':True}
    (DOC/'parser_runtime_details.json').write_text(json.dumps(runtime,indent=2)+'\n')
    results={'task':'P62-R4-EXP18-G0-METADATA-AUDIT','result':'G0_BLOCKED_COORDINATE_CONTRACT',
        'primary_blocker':'T_M_W and published GT I/base attitude semantics are not established',
        'coexisting_blockers':['G0_BLOCKED_CALIBRATION_CONTRACT','G0_BLOCKED_TIME_CONTRACT'],
        'asset_identity_status':'PASS_5_INPUTS_BEFORE_AFTER_UNCHANGED',
        'bag_parseability_status':'PASS_ALL_66633_MESSAGES',
        'e57_parseability_status':'PASS_ALL_285455240_POINTS_ALL_7_FIELDS_STREAMED',
        'coordinate_contract_status':'COORDINATE_CONTRACT_NOT_ESTABLISHED',
        'calibration_contract_status':'OFFICIAL_YAML_PRESENT_BUT_CONSUMER_SEMANTICS_NOT_ESTABLISHED',
        'time_contract_status':'TIME_CONTRACT_NOT_ESTABLISHED',
        'seconds_encoding_evidence':'SUPPORTED_BY_FIXED_OFFICIAL_FORMAT_AND_FULL_DATA; recording configuration NOT_CERTIFIED',
        'hypothetical_imu_coverage':derived['hypothetical_coverage_counts'],
        'first_scan_missing_imu_start_ns_if_seconds':derived['first_scan_missing_start_ns_if_absolute_seconds'],
        'reference_independence_status':{'dense':'CONDITIONAL_REGISTERED_REFERENCE',
            'sparse':'SURVEY_DERIVED_POSITION_ONLY_WITH_REFERENCE_POINT_CONTRACT_OPEN',
            'sparse_bag_supported':3,'sparse_dense_supported':2,'independent_dense_6dof_truth':False},
        'initialization_status':'INITIALIZATION_NOT_ESTABLISHED','storage_status':'PASS',
        'git':{'branch':'research/p62-r4-exp18-g0-metadata-audit',
            'start_sha':'1b53e7a4164c1e2871c077a4a6f1362e8c228a23',
            'actual_bag_e57_parser_code_sha':'954c49741f7c20b31a2dcb2459825bbb5ed70e97',
            'postprocessor_source_sha256':post_sha,'postprocessor_uncommitted_when_run':True,
            'archive_end_sha':'Containing final local archive commit; supplied as full SHA in final handoff',
            'queried_R3_remote_head':'1b53e7a4164c1e2871c077a4a6f1362e8c228a23',
            'queried_R4_remote_ref':'NOT_FOUND','push_executed':False},
        'unique_GT_files_semantically_read':2,'unit_tests_passed':11,
        'boundary':{'rosbag_play':0,'ros_localization_nodes':0,'ndt_align':0,'ekf_runs':0,
            'localization_trajectory_generation':0,'ate_rpe_or_pose_error_metrics':0,
            'clock_offset_search':0,'gyro_bias_fit':0,'GT_coordinate_fit':0,'production_code_changes':0,
            'hikvision_access':0,'external_model_cli':0},
        'next':'Stop. Request a narrowly scoped official export/consumer contract clarification before any localization.'}
    (DOC/'results.json').write_text(json.dumps(results,indent=2)+'\n')
    papers=[]
    for name in ('hilti2022.pdf','hilti2022.txt'):
        path=BASE/'p62_r1_source_cache'/name
        papers.append({'path':str(path),'bytes':path.stat().st_size,'sha256':digest(path),
            'source':'https://arxiv.org/abs/2208.09825','role':'Existing official-paper cache; read only for reference methodology'})
    if papers[0]['sha256']!='be8adc1b551661fefa0d8c6478e304e43d730d6f0affbef63729c648f7712367':
        raise RuntimeError('R2_FROZEN_OFFICIAL_PAPER_IDENTITY_MISMATCH')
    files=[p for p in sorted(DOC.rglob('*')) if p.is_file() and '__pycache__' not in p.parts
           and p.name not in ('analysis_manifest.json','checksums.sha256')]
    manifest={'scope':'P62-R4 metadata only','fixed_dataset_revision':'e62017f907007fdc5ab8c721842e4ae7359d7f49',
        'fixed_inputs':before['assets'],'source_snapshots':env['sources'],'paper_cache_identities':papers,
        'platform_page_identity':platform_evidence,
        'output_files':[{'path':str(p.relative_to(DOC.parents[1])),'bytes':p.stat().st_size,'sha256':digest(p)} for p in files],
        'scripts_identified_by_content':True,'parser_runtime_commit':results['git']['actual_bag_e57_parser_code_sha'],
        'raw_sensor_and_reference_files_unchanged':True,'no_G0_pass_inferred_from_parser_success':True}
    (DOC/'analysis_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    files.append(DOC/'analysis_manifest.json')
    guard();(DOC/'checksums.sha256').write_text(''.join(digest(p)+'  '+str(p.relative_to(DOC.parents[1]))+'\n' for p in sorted(files)))
    print(json.dumps({'result':results['result'],'archive_files':len(files),'storage':storage}))


if __name__=='__main__':main()
