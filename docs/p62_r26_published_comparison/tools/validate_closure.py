#!/usr/bin/env python3
"""Check source-backed facts, document completeness and scoped preservation only."""
from pathlib import Path
import hashlib,importlib.util,json,re,subprocess,sys
sys.dont_write_bytecode = True
D=Path(__file__).resolve().parents[1];R=D.parents[1]
S=Path('/home/jian/livox_ws/p62_r26_published_baseline_sources')
checks={}
def check(name,condition):
 checks[name]=bool(condition)
 if not condition:raise AssertionError(name)
required=['REPORT.md','PUBLISHED_BASELINE_RESULTS.md','PAPER_COMPARISON_TABLES.md','COMPARABILITY_AUDIT.md','RELATED_WORK_COMPARISON.md','PAPER_FINAL_DRAFT.md','SUBMISSION_READINESS.md']
check('required_documents_exist',all((D/p).is_file() for p in required))
records=json.loads((D/'PUBLICATION_METADATA.json').read_text())
check('seven_published_works_one_software',len(records)==8 and len([r for r in records if r['doi']!='NR'])==7)
check('all_requested_metadata_fields_populated',all(len(r)==23 and all(str(v).strip() for v in r.values()) for r in records))
# Numbers must be present in the actual table pages, not an abstract or graph.
for name,page,tokens in [('2fast_v3',18,['Table 7.','0.378','0.360','0.304','0.262','0.128']),('dlio',5,['TABLE I','0.3775','0.3606','42.86','35.74']),('steam_author_dynsys',10,['TABLE II','0.3042','74ms']),('singer_publisher',19,['Table III.','0.3020']),('2fast_v3',21,['Table 12.','0.17','0.32','2.24','41 ms','61 ms'])]:
 text=(S/(name+'.txt')).read_text().split('\f')[page-1]
 check(name+'_page_'+str(page),all(t in text for t in tokens))
public=json.loads((D/'PUBLIC_TRAJECTORY_ASSESSMENT.json').read_text())
check('public_trajectories_timestamp_identity',len(public)==2 and all(p['rows']==15301 and p['all_timestamps_equal_local_pcd_order'] for p in public))
for p,h in json.loads((D/'FROZEN_INPUT_SHA256.json').read_text()).items():
 check('frozen_input_'+p,hashlib.sha256((R/p).read_bytes()).hexdigest()==h)
source=(R/'docs/p62_r25_results_to_paper_closure/ALGORITHM_ABLATION.md').read_text()
for token in ['7.986422','13.052649','0.209943','2.984149','11.245245','57/3','60+8']:
 # Source table writes nominal/extra with slash, so check documented equivalents.
 source_token='60/8' if token=='60+8' else token
 check('internal_fact_'+token,source_token in source and token in (D/'PAPER_COMPARISON_TABLES.md').read_text())
paper=(D/'PAPER_FINAL_DRAFT.md').read_text()
for heading in ['Abstract','Introduction','Related Work','Methodology','Experimental Setup','Results','Discussion','Conclusion','References']:
 check('paper_section_'+heading,bool(re.search(r'^## (?:\d+\. )?'+heading+r'\s*$',paper,re.M)))
check('full_paper_not_revision_notes',len(paper.split())>5500)
check('separate_A_B_C_tables',all('Table '+s+'.' in paper for s in ['A','B','C']))
check('no_placeholder_author_results',not any(s in paper for s in ['TODO','TBD','INSERT_RESULT','0.000000']))
for rel in re.findall(r'!\[[^]]*\]\(([^)]+)\)',paper):
 check('figure_'+rel,(D/rel).is_file())
check('frozen_algorithm_no_diff',subprocess.check_output(['git','diff','6aeecf2adbc77c233ee0febaed192643a20c2f54','--','src']).decode()=='')
modified=subprocess.check_output(['git','diff','--name-only','6aeecf2adbc77c233ee0febaed192643a20c2f54']).decode().splitlines()
check('tracked_changes_R26_only',all(p.startswith('docs/p62_r26_published_comparison/') for p in modified))
for path in (D/'tools').glob('*.py'):
 compile(path.read_text(),str(path),'exec')
check('documentation_tools_syntax',True)
spec=importlib.util.spec_from_file_location('r23_preservation_reader',R/'docs/p62_r23_real_imu_closure/tools/prepare_inputs.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
after=m.stable_snapshot();(D/'STABLE_AFTER.json').write_text(json.dumps(after,indent=2)+'\n')
check('stable_workstation_unchanged',after==json.loads((D/'STABLE_BEFORE.json').read_text()))
# No self-referential final Git SHA is fabricated; exact END is supplied by Git/handoff.
result={'status':'PASS','checks':checks,'count':len(checks),'new_native_alignments':0,'external_estimator_runs':0,'scope':'facts/documents/scoped Git and small protected-file identity; no experiment'}
(D/'FINAL_VALIDATION.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
