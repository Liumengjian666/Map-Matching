#!/usr/bin/env python3
"""Materialize reviewed duplicate choices; all other large files remain untouched."""
import json
from pathlib import Path
import shlex
from audit import OUT, PROTECTED, csvout, signature, write
from safe_delete import unused, validate_pair

# Retention choices are explicit, not inferred by filename during execution.
KEEPERS={
 'd17bee3f8697dcb01cb4b9f6ea2483aa42a1a31ae277659c8a7869d9c69f13ae':
 '/home/jian/rosbag/loop2/raw_restore_20260914_stage0/loop2_raw.bag',
 '5d8b7ea6632f2a2230113b1a5041321c6e6bf47539ff725fa108030e071dfc5e':
 '/home/jian/rosbag/FASTLIVO2-dogmapping/realtime_map_match/bag/realtime_map_match_20261007_143634.bag',
 '34eec3debb8b3523e35a32e5b32a9550795eaddb17e3265945164aa7596436f8':
 '/home/jian/rosbag/fastlio_location_tests/loop4_on_degen_map_2026_08_07_18_57_58/pcd/loop4_on_degen_map_2026_08_07_18_57_58/loop4_on_degen_map_2026_08_07_18_57_58_2026_08_07_19_09_07_all_raw_points.pcd',
 'b0aec1c29883af0eb57c915f17e92ad64fedfe5849d2a5697e978285aea8d88d':
 '/home/jian/rosbag/fastlivo2_degen_tests/loop4_degen_2026_08_07_17_56_01/pcd/loop4_degen_2026_08_07_17_56_01/loop4_degen_2026_08_07_17_56_01_2026_08_07_18_07_09_all_raw_points.pcd',
 'bbbfb1550b5bcfb75b0c650bd29d0a18d0486a6569fe7011a221678c665f1280':
 '/home/jian/rosbag/fastlivo2_degen_tests/loop4_degen_guard_2026_08_07_18_25_02/pcd/loop4_degen_guard_2026_08_07_18_25_02/loop4_degen_guard_2026_08_07_18_25_02_2026_08_07_18_36_07_all_raw_points.pcd',
 '16af451f94016bcf84d1212eea2b32142e7fabcc148654f2907fbe09eb39aa39':
 '/home/jian/rosbag/fastlivo2_original_vs_improved_loop4_2026_08_07_21_25_03/loop1_raw_official_vs_improved_2026_08_07_22_26_58/improved_degen_guard/output/pcd/loop1_improved_degen/loop1_improved_degen_2026_08_07_22_57_21_all_raw_points.pcd',
}

def main():
    inventory=json.loads((OUT/'INVENTORY_BEFORE.json').read_text())
    hashes={r['path']:r for r in json.loads((OUT/'HASHES_BEFORE.json').read_text())}
    refs=json.loads((OUT/'REFERENCE_AUDIT.json').read_text())
    if refs['skipped_oversize']:raise RuntimeError('Incomplete dependency audit')
    plan=[];approved=[];large=[];checks=[]
    # Also explicitly list the two protected files below the large-file cutoff.
    rows=inventory+[dict(path=p,**signature(p),type=Path(p).suffix,mtime_utc='SEE_PROTECTED_MANIFEST',magic='NOT_NEEDED_FOR_EXPLICIT_PROTECTION')
                    for p in PROTECTED if p not in {r['path'] for r in inventory}]
    for r in rows:
        p=r['path'];record=hashes.get(p);digest=record['sha256'] if record else 'NOT_COMPUTED_UNIQUE_SIZE'
        links=[m for m in refs['matches'] if m['asset']==p]
        copies=[v['path'] for v in hashes.values() if record and v['sha256']==digest and v['path']!=p]
        item=dict(path=p,classification='UNCERTAIN',basis='NO_VERIFIED_DUPLICATE_OR_COMPLETE_REGENERATION_CONTRACT',
                  sha256=digest,retained_copy='',reference_count=len(links),reason='Unique/undetermined content; historical mention alone is not a preservation or deletion decision')
        if p in PROTECTED:
            item.update(classification='PROTECTED',basis='EXPLICIT_FROZEN_RESEARCH_INPUT',reason='User-specified path and frozen SHA must remain')
        elif digest in KEEPERS and p==KEEPERS[digest]:
            item.update(classification='PROTECTED',basis='RETAINED_VERIFIED_PAYLOAD',reason='At least one exact payload and generating-output location is retained')
        elif digest in KEEPERS:
            keeper=KEEPERS[digest]
            if links:
                item.update(reason='Reference match requires separate assessment; do not delete automatically')
            else:
                validate_pair(p,keeper,digest,hashes)
                try:checks.append(dict(path=p,retained_copy=keeper,checks=unused([p,keeper])))
                except Exception as exc:item.update(reason='Open-file check uncertainty: '+repr(exc))
                else:
                    item.update(classification='SAFE_TO_DELETE',basis='A_EXACT_SHA_DUPLICATE',retained_copy=keeper,
                       reason='Full SHA and length identical; independent retained inode; no matches in R6-R14/current source/config reference scan; no other open process observed')
                    approved.append(item.copy())
        elif record and not copies:
            item.update(classification='PROTECTED',basis='HASHED_DISTINCT_CONTENT',reason='Same-size comparator has different SHA; unique original/runtime content not proven regenerable')
        plan.append(item)
        if r['size']>300*1024**2:
            large.append(dict(path=p,allocated_bytes=r['allocated'],logical_bytes=r['size'],file_type=r['type'],magic=r['magic'],
                mtime_utc=r['mtime_utc'],original_data='CONFIRMED' if p in PROTECTED else 'UNKNOWN',
                cropped_data='NOT_ESTABLISHED',recovery_copy='VERIFIED_IDENTICAL_RECOVERY_COPY' if 'stage' in p and digest in KEEPERS and digest.startswith('d17bee') else 'NOT_ESTABLISHED',
                identical_files=';'.join(copies) if copies else 'NOT_FOUND_BY_SIZE_GROUP_SCAN',
                current_research_reference='CONFIRMED_FROZEN_INPUT' if p in PROTECTED else ('MENTION_FOUND_USE_UNCONFIRMED' if links else 'NO_MATCH_IN_BOUNDED_SEARCH'),
                sha256=digest,classification=item['classification'],evidence='REFERENCE_AUDIT.json; HASHES_BEFORE.json; CLEANUP_PLAN.csv',nlink=r['nlink']))
    large.sort(key=lambda r:(-r['allocated_bytes'],-r['logical_bytes'],r['path']))
    plan.sort(key=lambda r:(r['classification'],r['path']))
    approved.sort(key=lambda r:(-hashes[r['path']]['signature']['allocated'],r['path']))
    if len(approved)!=10:raise RuntimeError('Unexpected plan cardinality; review before deleting: '+str(len(approved)))
    write('APPROVED_DUPLICATES.json',approved);write('OPEN_FILE_CHECKS_PLAN.json',checks)
    csvout('CLEANUP_PLAN.csv',plan,list(plan[0]))
    csvout('ROSBAG_LARGE_FILES.csv',large,list(large[0]))
    csvout('PROTECTED_FILES.csv',[r for r in plan if r['classification']=='PROTECTED'],list(plan[0]))
    with (OUT/'RECONSTRUCT_EXACT_COPIES.sh.txt').open('x') as f:
        f.write('# Not executed. Recreates exact SHA-identified payloads from retained copies, without localization.\n')
        for r in approved:f.write('cp --reflink=auto -- '+shlex.quote(r['retained_copy'])+' '+shlex.quote(r['path'])+'\n')
    print(json.dumps(dict(safe_files=len(approved),safe_allocated_GiB=sum(hashes[r['path']]['signature']['allocated'] for r in approved)/1024**3,
       protected=sum(r['classification']=='PROTECTED' for r in plan),uncertain=sum(r['classification']=='UNCERTAIN' for r in plan)),indent=2))

if __name__=='__main__':main()
