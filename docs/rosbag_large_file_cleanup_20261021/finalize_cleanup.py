#!/usr/bin/env python3
"""Verify retained assets, measure df net change, and write factual cleanup outputs."""
import csv
import hashlib
import json
from pathlib import Path
import shutil
import sys
from audit import OUT, ROOT, PROTECTED, command_file, csvout, hash_record, protections, signature, write

GIB=1024**3

def mdtable(rows):
    lines=['| 完整路径 | 逻辑GiB | 实占GiB |','|---|---:|---:|']
    for r in rows[:20]:
        lines.append('| `'+r['path']+'` | '+f"{r['size']/GIB:.3f}"+' | '+f"{r['allocated']/GIB:.3f}"+' |')
    return '\n'.join(lines)

def main():
    if not (OUT/'DELETE_FINISHED.json').exists():raise RuntimeError('Deletion not completed; do not claim pass')
    if (OUT/'CLEANUP_REPORT.md').exists():raise RuntimeError('Do not overwrite prior report')
    before=json.loads((OUT/'INVENTORY_BEFORE.json').read_text())
    planned=json.loads((OUT/'APPROVED_DUPLICATES.json').read_text())
    hashes={r['path']:r for r in json.loads((OUT/'HASHES_BEFORE.json').read_text())}
    with (OUT/'DELETED_FILES.csv').open() as f:deleted=list(csv.DictReader(f))
    if {r['path'] for r in deleted}!={r['path'] for r in planned} or len(deleted)!=len(planned):
        raise RuntimeError('Deletion evidence differs from plan')
    for r in deleted:
        if Path(r['path']).exists():raise RuntimeError('Deleted file path unexpectedly exists')
    # Recompute full content hashes, including the retained 19.7GB original.
    retained=[]
    for p in sorted(set(PROTECTED)|{r['retained_copy'] for r in planned}):
        print('VERIFY_RETAINED '+p,flush=True)
        actual=hash_record(p)
        if actual['sha256']!=hashes[p]['sha256'] or actual['signature']!=hashes[p]['signature']:
            raise RuntimeError('Retained content/stat changed: '+p)
        retained.append(actual)
    write('RETAINED_ASSET_VERIFICATION.json',retained)
    remaining=[];deleted_paths={r['path'] for r in deleted}
    for r in before:
        if r['path'] in deleted_paths:continue
        if signature(r['path'])!={k:r[k] for k in signature(r['path'])}:
            raise RuntimeError('Unplanned large file change: '+r['path'])
        remaining.append(r)
    remaining.sort(key=lambda r:(-r['allocated'],-r['size'],r['path']))
    write('INVENTORY_AFTER.json',remaining)
    after=protections();initial=json.loads((OUT/'PROTECTION_BEFORE.json').read_text())
    write('PROTECTION_AFTER.json',after)
    if after!=initial:raise RuntimeError('Production/research archive drift; report failure rather than PASS')
    command_file('DF_AFTER.txt',['df','-B1',str(ROOT)])
    command_file('DF_AFTER_HUMAN.txt',['df','-h',str(ROOT)])
    command_file('DU_AFTER.txt',['du','-xhd','2',str(ROOT)])
    df_before=int((OUT/'DF_BEFORE.txt').read_text().splitlines()[-1].split()[3])
    df_after=int((OUT/'DF_AFTER.txt').read_text().splitlines()[-1].split()[3])
    delta=df_after-df_before
    storage=dict(path=str(ROOT),before_available_bytes=df_before,after_available_bytes=df_after,
                 actual_df_available_increase_bytes=delta,actual_df_available_increase_GiB=delta/GIB,
                 deleted_files=len(deleted),deleted_bag_files=sum(r['path'].endswith('.bag') for r in deleted),
                 deleted_pcd_files=sum(r['path'].endswith('.pcd') for r in deleted),
                 deleted_cropped_bags=0,deleted_duplicate_dataset_directories=0,
                 duplicate_bag_content_groups=2,duplicate_pcd_content_groups=4,
                 protected_assets_unchanged=True,large_files_remaining=len(remaining),
                 note='df net available-byte increase, not logical-size sum; includes concurrent filesystem activity and audit-output allocation')
    csvout('STORAGE_AFTER.csv',[storage],list(storage))
    counts=[]
    for unit,base in [('GB',1000**3),('GiB',1024**3)]:
        for threshold in [1,5,10]:
            counts.append(dict(unit=unit,threshold=threshold,before_count=sum(r['size']>threshold*base for r in before),
                               after_count=sum(r['size']>threshold*base for r in remaining)))
    csvout('SIZE_COUNTS_DETAILED.csv',counts,list(counts[0]))
    with (OUT/'CLEANUP_PLAN.csv').open() as f:plan=list(csv.DictReader(f))
    uncertain=[r for r in plan if r['classification']=='UNCERTAIN']
    bagrows=[r for r in planned if r['path'].endswith('.bag')]
    result=dict(task='ROSBAG-LARGE-FILE-CLEANUP',status='SAFE_DUPLICATE_CLEANUP_COMPLETED',**storage,
        large_files_before=len(before),safety_tests_passed=18,archive_reference_matches=4562,
        new_NDT_align=0,GT_evaluation=0,EKF_runs=0,downloads=0,production_changes=0,
        R14_automatically_resumed=False,git_changes=False,git_commit=None,
        R14_storage_assessment='CAPACITY_HEADROOM_IMPROVED_ONLY; precise asset/peak/download and coordinate/time gates still required')
    write('results.json',result)
    report=f'''# ROSBAG-LARGE-FILE-CLEANUP

结果：SAFE_DUPLICATE_CLEANUP_COMPLETED。仅删除明确完整内容重复副本；没有删除唯一原始数据、裁剪数据或整数据集目录。

## 实际释放空间

- 清理前df可用：{df_before:,} bytes（{df_before/GIB:.3f} GiB）。
- 清理后df可用：{df_after:,} bytes（{df_after/GIB:.3f} GiB）。
- **df可用空间实际增加：{delta/GIB:.3f} GiB（{delta:,} bytes）**。
- 删除{len(deleted)}个大文件：4份完整重复bag（其中3份同一loop2旧恢复副本）及6份重复PCD；涉及2组bag内容、4组PCD内容。
- 删除裁剪bag：0；删除完整数据集目录：0；原始唯一bag删除：0。
- df为前后净值，包含同期其他磁盘活动和审计产物，不把逻辑大小之和冒充实际释放量。完整原始df/du回执已保存。

## 盘点和判定

全目录同设备普通文件中，大于300MiB的文件共{len(before)}个，清理后{len(remaining)}个。GB/GiB两种门槛统计见SIZE_COUNTS_DETAILED.csv；扩展名、magic、mtime、st_blocks实占、nlink与初步来源见ROSBAG_LARGE_FILES.csv。

对相同大小组完整顺序计算SHA256；仅精确SHA匹配才作为重复证据。两份同大小loop3 result/orig bag的SHA不同，均保留。没有从名字推断数据完全相同。FASTLIVO2-dogmapping.zip、唯一历史结果、.bag.active及日志未获得可靠重复或完整再生合同，保留为UNCERTAIN。

引用范围：R6–R14实际研究工作区，以及稳定/轻量/论文工作区的源码、配置、脚本、manifest和报告。第二轮扫描记录4562条引用、286个文件，无超限跳过。10个拟删文件的精确路径及特定父目录令牌无匹配；指定原始bag有200条引用。原始引用与历史提及分开保留在REFERENCE_AUDIT.json。这是限定静态证据，不能证明任意外部脚本永远不会使用旧路径。

所有删除按A类证据：保留副本完整SHA及长度匹配、独立inode、进程占用检查通过、非符号链接/挂载点或跨设备路径、文件版本不变。删除前再次完整哈希目标与保留源，逐条INTENT/UNLINKED/VERIFIED_DELETED持久日志；没有通配符递归删除，也没有移到同盘回收站。没有根据B类可重建性推测删除唯一派生文件。

## 实际删除的bag

'''+ '\n'.join('- `'+r['path']+'`；保留 `'+r['retained_copy']+'`。' for r in bagrows)+f'''

其余6份PCD完整路径及保留副本见DELETED_FILES.csv。数据内容可按RECONSTRUCT_EXACT_COPIES.sh.txt中的明确cp命令重新生成；本轮未执行恢复。不保证恢复原inode/mtime，只保证可重建同SHA载荷。旧报告和路径记录没有改写。

## 清理前最大的20个文件（按实占排序）

{mdtable(before)}

## 清理后最大的20个文件（按实占排序）

{mdtable(remaining)}

## 不确定而保留的文件

共{len(uncertain)}项。均未删除，具体原因见CLEANUP_PLAN.csv：

'''+ '\n'.join('- `'+r['path']+'`：'+r['reason']+'。' for r in uncertain)+'''

## 冻结科研依赖与工程保护

三个指定路径保持存在且完整SHA正确：

- loop2/raw_restore_20260914_stage0/loop2_raw.bag：d17bee3f8697dcb01cb4b9f6ea2483aa42a1a31ae277659c8a7869d9c69f13ae。
- loop2/loop2mapping/pcd/loop2_simtime_rebuild_2026_08_12_001_all_downsampled_points.pcd：016b7ae3ae494ad5d462a66d888a96b96dda5fe29d6058059aa2fa6bdf6b09ca。
- loop2/final_delivery_run_A_20260922/ndt_determinism.csv：e5f749cbbc7fc514326cc61626945f4ac6347722e4ad769e676984e1d214102a。

R13/R14全部归档校验项、HEAD和工作树状态前后相同。稳定工程HEAD、refs、reflog、index、状态和快照文件SHA相同。所有未删大文件版本签名保持不变。唯一GT/标定等小文件不在任何删除清单中，未读取内容或修改。未触碰R10保护备份、Git历史、源码或参数。

18项安全测试通过；首次测试兼容性失败、第一次引用扫描中断和工具选项错误均保留，不伪造首轮通过。独立本地反证审阅共3个有界周期，发现问题已修正；最后未发现新增实质问题。用户选择不调用外部模型。守卫不能保证防御任意非合作写入者在系统调用间隙修改文件，检查范围和残余竞态见SAFETY_REVIEW.md。

## R14与唯一NEXT

当前容量显著改善，足以重新考虑同一Wheel-float01的容量准入，但**没有因此认证精确下载/处理峰值、资产访问、坐标或时间合同**。唯一NEXT：将本清理回执交科研总控，由其决定是否批准恢复同一R14资产准入；本轮不自动下载或运行R14。

本轮审计产物位于独立目录，不修改任何研究分支，未形成本轮Git commit，也未push。没有可诚实提供的本轮分支上传命令；不得用旧研究分支上传无关修改。
'''
    with (OUT/'CLEANUP_REPORT.md').open('x') as f:f.write(report)
    files=sorted(p for p in OUT.rglob('*') if p.is_file() and '__pycache__' not in p.parts)
    with (OUT/'checksums.sha256').open('x') as f:
        for p in files:f.write(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+str(p.relative_to(OUT))+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
