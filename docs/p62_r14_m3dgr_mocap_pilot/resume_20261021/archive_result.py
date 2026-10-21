#!/usr/bin/env python3
"""Seal the existing access failure, without another network request or experiment."""
import csv
import hashlib
import json
from pathlib import Path
import runpy
import shutil
import subprocess

DOC=Path(__file__).resolve().parent
REPO=DOC.parents[2]

def write(name,value):
    with (DOC/name).open('x') as f:
        json.dump(value,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')

def textfile(name,text):
    with (DOC/name).open('x') as f:f.write(text)

def main():
    if (DOC/'results.json').exists():raise RuntimeError('Do not overwrite an existing continuation result')
    source=runpy.run_path(str(DOC/'acquire.py'),run_name='offline_protection_reader')
    start=json.loads((DOC/'ACCESS_STARTED.json').read_text())
    receipt=json.loads((DOC/'OFFICIAL_ACCESS_RECEIPT.json').read_text())
    stage=json.loads((DOC/'RESUME_STAGE_RESULT.json').read_text())
    if receipt['status']!='ASSET_ACCESS_REQUIRES_USER_ACTION':raise RuntimeError('Only finalize the actual access-block outcome')
    before=json.loads((DOC/'PRESERVED_ASSETS_BEFORE.json').read_text())
    if before!=json.loads((DOC/'PRESERVED_ASSETS_AFTER.json').read_text()) or before!=source['snapshot']():
        raise RuntimeError('Production or historical R14 protection failed')
    write('FINAL_PROTECTION_RECEIPT.json',dict(old_R14_and_stable_unchanged=True,
          old_file_count=len(before['old_R14_files']),method='SHA of all old R14 tracked artifacts and complete stable source/Git snapshot; no historical runner'))
    data=source['DATA'];free=shutil.disk_usage(data).free
    payload_files=[str(p) for p in data.rglob('*') if p.is_file()]
    if payload_files:raise RuntimeError('New local payload appeared; preserve and verify before claiming no local file')
    used={key:sum(p.stat().st_size for p in (data/folder).rglob('*') if p.is_file())
          for key,folder in [('bag','bag'),('gt_aux','gt'),('map_temp','map_temp'),('run_results','run_results')]}
    used['run_results']+=sum(p.stat().st_size for p in DOC.rglob('*') if p.is_file() and '__pycache__' not in p.parts)
    if free<source['RESERVE'] or any(used[k]>=v-source['NEAR'] for k,v in source['LIMITS'].items()):
        raise RuntimeError('Budget/reserve safety failed')
    write('STORAGE_BUDGET.json',dict(status='PASS_FOR_DECLARED_BOUNDED_BUDGET',before_available_bytes=start['free_bytes'],
          after_available_bytes=free,reserve_bytes=source['RESERVE'],limits_bytes=source['LIMITS'],
          used_bytes_at_seal_start=used,total_authorized_new_bytes=sum(source['LIMITS'].values()),
          near_limit_margin_bytes=source['NEAR'],actual_target=str(data),actual_device=data.stat().st_dev,
          note='New report bytes increase slightly during sealing; all remain far below4GiB results budget'))
    result=dict(task='P62-R14-ASSET-RESUME-AND-INDEPENDENT-EVALUATION',
          status='ASSET_ACCESS_REQUIRES_USER_ACTION',only_actual_blocker='No real official bag download endpoint obtained; local Alipan TLS EOF',
          source_CODE_SHA=start['CODE_SHA'],base_sha=source['BASE'],official_revision=source['REV'],
          storage_status='PASS_FOR_DECLARED_BOUNDED_BUDGET',expected_bag_sha256=source['EXPECTED'],
          actual_bag_sha256=None,actual_bag_size_bytes=None,bag_payload_downloaded_bytes=receipt['bag_payload_bytes'],
          GT_source='Official Wheel-float01 GT share; not acquired or qualified',GT_pose_files_read=0,
          coordinate_contract='NOT_RUN_AFTER_ASSET_ACCESS_BLOCK',time_contract='NOT_RUN_AFTER_ASSET_ACCESS_BLOCK',
          prediction_contract='NOT_RUN_AFTER_ASSET_ACCESS_BLOCK',map_builds=0,map_test_coverage=None,
          registered_test_frames=None,new_NDT_align=0,EKF_runs=0,rosbag_play=0,production_changes=0,
          error_statistics=None,natural_events=None,distinct_candidates=None,accepted_recoveries=None,
          five_degree_false_rejection=None,algorithm_wall_seconds=None,algorithm_peak_RSS_bytes=None,
          access_wall_seconds=receipt['wall_seconds'],acquisition_process_peak_RSS_bytes=receipt['acquisition_process_peak_rss_bytes'],
          runtime_scope=receipt['memory_scope'],old_preflight_runs=0,old_OneDrive_requests=0,
          Alipan_initial_GET_attempts=1,safety_tests_passed=15,preserved_assets_unchanged=True,
          S1='NOT_EVALUABLE',S2='NOT_EVALUABLE',S3='NOT_EVALUABLE',S4='NOT_EVALUABLE',S5='NOT_EVALUABLE',S6='NOT_EVALUABLE',
          novelty='NOT_ESTABLISHED',push_executed=False,
          unique_NEXT='User obtains this same official Wheel-float01 bag and GT via browser or supplies legitimate real payload URLs; then verify identity/contracts before any experiment')
    write('results.json',result)
    write('MAP_MANIFEST.json',dict(status='NOT_BUILT_AFTER_ASSET_ACCESS_BLOCK',map_path=None,sha256=None,points=None,
          planned_type='Oracle-Built Prior Map; first60% only; M=W',GT_build_range=None,builds_executed=0))
    write('TEST_FRAME_PRE_REGISTRATION.json',dict(status='NOT_SELECTED_AFTER_ASSET_ACCESS_BLOCK',frames=[],count=None,
          planned_rule='20 time-equally-spaced frames in65%-95% whole-sequence time; no result-dependent selection'))
    for name,header in {
      'NDT_COMPARISON.csv':'frame_index,stamp_ns,method,converged,iterations,fitness,overlap_0_5,status',
      'ACCEPTANCE_AND_EVENTS.csv':'frame_index,stamp_ns,natural_event,event_reason,admissible,extra_align,final_observation_status',
      'GT_ERROR_COMPARISON.csv':'frame_index,stamp_ns,method,GT_stamp_ns,match_error_ns,translation_error_m,rotation_error_deg',
      'RUNTIME_AND_MEMORY.csv':'stage,wall_seconds,peak_rss_bytes,scope,status'
    }.items():textfile(name,header+'\n')
    with (DOC/'ACCESS_RUNTIME.csv').open('x',newline='') as f:
        w=csv.writer(f);w.writerow(['stage','wall_seconds','peak_rss_bytes','scope'])
        w.writerow(['official_access',receipt['wall_seconds'],receipt['acquisition_process_peak_rss_bytes'],receipt['memory_scope']])
    textfile('REPORT.md',f'''# R14授权续行：ASSET_ACCESS_REQUIRES_USER_ACTION

唯一实际阻塞：没有取得真实官方bag下载地址。存储阻塞已经解除，后续坐标、时间及算法阶段未运行，不额外虚构其他失败。

固定基线 `{source['BASE']}`；实际获取程序CODE_SHA `{start['CODE_SHA']}`；同一R14分支及工作区，旧失败归档保持原样。原分支开始时本地与远端均为基线SHA，工作树干净。

## 已验证事实

- 下载目标 `{data}` 与研究工作区同文件系统。启动可用 `{start['free_bytes']}` bytes；预留20GiB，授权bag6/GT辅助1/地图临时20/日志结果4GiB共31GiB，预算门通过。64MiB提前停止余量固定，不根据结果改变。
- 限定3个目录的序列标识文件名搜索没有发现候选载荷。未声称对所有改名文件作穷尽哈希搜索。
- 复用官方固定版本 `{source['REV']}` 的序列行与摘要记录；预期bagSHA `{source['EXPECTED']}` 仅为预期身份，不是已验证本地文件。
- 只做一次本机Alipan初始GET：`{source['URL']}`。没有再次请求旧OneDrive链接，没有运行旧preflight.py。
- 本机错误：`{receipt.get('error','NOT_AVAILABLE')}`；HTTP状态未知，未收到HTTP响应或有效载荷。耗时 `{receipt['wall_seconds']:.6f}` 秒。它不证明官方链接全球失效，也不证明必须登录。
- 浏览工具另对官方固定说明及Alipan入口进行了有限读取；分享页在该工具中不可读。这不是下载成功或认证结论。
- bag下载0 bytes，本地bag大小/实际SHA未知。GT未请求本机下载或解析，来源与独立性仍未验收。

## 后续全部NOT_RUN

没有建立T_W_B/T_B_L/T_O_B或时间关系；没有地图、测试帧、wheel预测、NDT、EKF或误差评价。S1–S6均NOT_EVALUABLE，不能回答5°误拒或预测/NDT谁更准确。算法时延与RSS未测。

获取进程峰值RSS `{receipt['acquisition_process_peak_rss_bytes']}` bytes，包含只读保护快照，不是NDT运行内存。实际请求耗时另记ACCESS_RUNTIME.csv，算法RUNTIME_AND_MEMORY.csv仅表头。

15项下载守卫合成测试通过；初次反证发现的总超时、循环/默认端口重定向问题已修正。没有把测试通过写成真实数据或算法通过。真实生产与全部旧R14文件SHA/Git快照前后相同。无清理、无删除、无新数据集、无R15。

## 唯一NEXT

请用户通过官方网页取得同一Wheel-float01 bag及GT，或提供合法可访问的真实载荷URL。入口、容量限制、落盘路径和所需操作详见USER_ACTION_REQUIRED.md。收到材料后仍必须先核验SHA/格式及参考、外参、时间和预测合同；本轮不自动重试或运行实验。
''')
    textfile('DATASET_IDENTITY.md',f'''# 唯一资产身份

M3DGR / Wheel-float01，官方固定版本 `{source['REV']}`。官方摘要和发布入口见上级evidence目录；旧文件不改写。

预期bagSHA：`{source['EXPECTED']}`。实际bag文件大小和SHA均NOT_AVAILABLE，不能认证载荷身份。GT未下载，文件名、大小、SHA、字段、刚体坐标和时间尚未实测。TLS失败回执见OFFICIAL_ACCESS_RECEIPT.json。
''')
    for name,title in [('GT_COORDINATE_CONTRACT.md','GT与坐标'),('TIME_ALIGNMENT_CONTRACT.md','时间'),
                       ('PREDICTION_CONTRACT.md','非Oracle预测'),('MAP_BUILD_CONTRACT.md','Oracle建图'),
                       ('RESULTS_AND_LIMITATIONS.md','科学评价限制')]:
        textfile(name,f'# {title}：NOT_RUN_AFTER_ASSET_ACCESS_BLOCK\n\n真实载荷未取得，未进入此项检查或实验。沿用原批准合同，不以官方平台概述代替实际会话认证，不通过GT拟合外参或时差，不将测试GT加入地图或预测。后续未运行指标均为null，而非测得零误差。\n')
    textfile('TESTS_AND_REVIEW.md','''# 测试与反证复核

15项合成测试通过，终端回执：Ran15 tests in0.030s，OK。覆盖空间/硬容量边界、HTML拒绝、有限页面样本、SHA不符不发布、已有partial保护、全局慢流超时、重定向重复/循环/默认端口/降级与合成匹配摘要发布。合成数据不是ROSbag遍历或真实优化器测试。

本地独立反证共3个有界周期：修复全局超时遗漏、循环重复请求、默认HTTPS端口差异问题，最后未发现剩余实质问题。交叉模型意见已非阻塞提供选择，未运行外部CLI。实际访问仅执行一次，不为修复数学阈值重跑算法。

离线复核：python3 test_acquire.py；sha256sum -c checksums.sha256。不要再次执行acquire.py或原preflight.py。执行标记拒绝重跑。
''')
    with (DOC/'checksums.sha256').open('x') as f:
        for p in sorted(p for p in DOC.rglob('*') if p.is_file() and '__pycache__' not in p.parts):
            if p.name=='checksums.sha256':continue
            f.write(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+str(p.relative_to(DOC))+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
