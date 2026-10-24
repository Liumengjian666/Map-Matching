#!/usr/bin/env python3
"""Necessary typesetting, frozen-result and scoped-preservation checks only."""
from pathlib import Path
import re,json,hashlib,subprocess,importlib.util,sys
import fitz
from PIL import Image
sys.dont_write_bytecode=True
D=Path(__file__).resolve().parents[1];R=D.parents[1];checks={}
def check(name,condition):
 checks[name]=bool(condition)
 if not condition:raise AssertionError(name)
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
needed=['PAPER_SUBMISSION_DRAFT.tex','PAPER_SUBMISSION_PDF.pdf','COVER_LETTER_DRAFT.md','COVER_LETTER_DRAFT.docx','HIGHLIGHTS.md','FIGURE_CAPTIONS.md','REFERENCES.bib','REFERENCES.md','SUBMISSION_CHECKLIST.md','NOVELTY_POSITIONING.md','ADVISOR_REVIEW_SUMMARY.md','ADVISOR_REVIEW_SUMMARY.pdf','SUPPLEMENTARY_MATERIAL.tex','SUPPLEMENTARY_MATERIAL.pdf']
check('all_requested_deliverables',all((D/p).is_file() for p in needed))
s=(D/'PAPER_SUBMISSION_DRAFT.tex').read_text();a=s.split('\\begin{abstract}')[1].split('\\end{abstract}')[0]
check('abstract_150_to_250_words',150<=len(a.split())<=250)
check('abstract_single_paragraph_no_citations_equations','\n\n' not in a.strip() and '\\cite' not in a and '$' not in a)
keys=s.split('\\begin{IEEEkeywords}')[1].split('\\end{IEEEkeywords}')[0];check('three_to_five_keywords',3<=len(keys.split(','))<=5)
contrib=s.split('The contributions are limited')[1].split('\\end{enumerate}')[0];check('at_most_three_contributions',contrib.count('\\item')==3)
check('IEEE_journal_double_column_source','\\documentclass[journal,10pt,letterpaper]{IEEEtran}' in s)
check('four_protocol_separated_tables',s.count('\\begin{table*}')==4)
check('no_internal_stage_numbers_in_main_body',not re.search(r'\b(?:R(?:19|20|21|22|23|24|25|26)|P45)\b',s.split('\\begin{document}')[1]))
check('explicit_author_information_placeholders',all(x in s for x in ['Author names','Affiliations','Corresponding author','Funding agencies']))
for name in ['PAPER_SUBMISSION_PDF','SUPPLEMENTARY_MATERIAL','ADVISOR_REVIEW_SUMMARY']:
 doc=fitz.open(D/(name+'.pdf'));text='\n'.join(p.get_text() for p in doc)
 check(name+'_nonempty_pages',all(len(p.get_text().strip())>100 for p in doc))
 check(name+'_no_unresolved_labels','??' not in text and '\ufffd' not in text)
 check(name+'_no_page_clipping',all(span['bbox'][0]>=0 and span['bbox'][2]<=p.rect.width+1 and span['bbox'][1]>=0 and span['bbox'][3]<=p.rect.height+1 for p in doc for b in p.get_text('dict')['blocks'] if 'lines' in b for l in b['lines'] for span in l['spans']))
 if name=='PAPER_SUBMISSION_PDF':
  check('main_seven_pages_within_normal_eight',len(doc)==7)
  check('US_Letter_pages',all(abs(p.rect.width-612)<1 and abs(p.rect.height-792)<1 for p in doc))
  for token in ['7.986422','13.052649','0.209943','2.984149','57/60','11.245245']:
   check('PDF_frozen_value_'+token,token in text)
  check('all_four_table_numbers',all('TABLE '+n in text for n in ['I','II','III','IV']))
  check('main_three_figure_captions',all('Fig. '+str(n)+'.' in text for n in [1,2,3]))
 elif name=='ADVISOR_REVIEW_SUMMARY':check('Chinese_memo_at_most_two_pages',len(doc)<=2 and '导师' in text)
 else:check('supplement_four_pages',len(doc)==4)
for name in ['PAPER_SUBMISSION_DRAFT','SUPPLEMENTARY_MATERIAL']:
 log=(D/'build'/(name+'_build.log')).read_text()
 check(name+'_build_no_errors_missing_refs_or_overfull',not any(t in log for t in ['error:','Overfull','undefined','Missing character']))
fonts=subprocess.check_output(['pdffonts',str(D/'PAPER_SUBMISSION_PDF.pdf')]).decode().splitlines()[2:]
check('all_main_pdf_fonts_embedded',bool(fonts) and all(re.search(r'\byes\s+yes\s+(?:yes|no)\b',line) for line in fonts))
bbl=(D/'REFERENCES_COMPILED.bbl').read_text();bib=(D/'REFERENCES.bib').read_text()
check('fourteen_resolved_reference_entries',len(re.findall(r'\\bibitem\{',bbl))==14)
refkeys=set(re.findall(r'\\bibitem\{([^}]+)\}',bbl));cites=set(k for x in re.findall(r'\\cite\{([^}]+)\}',s) for k in x.split(','))
check('all_cited_keys_resolve',cites==refkeys)
check('all_eight_paper_DOIs_printed',len(re.findall(r'doi=\{([^}]+)\}',bib))==8 and all(d in re.sub(r'\s+','',bbl) for d in re.findall(r'doi=\{([^}]+)\}',bib)))
im=Image.open(D/'figures/gagraphic.png')
check('GA_exact_660x295',im.size==(660,295));check('GA_300dpi_metadata',all(abs(n-300)<1 for n in im.info['dpi']));check('GA_recommended_under45KB',(D/'figures/gagraphic.png').stat().st_size<45000)
check('copied_error_figure_unchanged',sha(D/'figures/ablation_errors.pdf')==sha(R/'docs/p62_r25_results_to_paper_closure/figures/ablation_errors.pdf'))
check('copied_quality_figure_unchanged',sha(D/'figures/coverage_admission.pdf')==sha(R/'docs/p62_r25_results_to_paper_closure/figures/coverage_admission.pdf'))
for p,h in json.loads((D/'INPUT_IDENTITIES.json').read_text()).items():check('frozen_input_'+p,sha(R/p)==h)
check('algorithm_source_unchanged',subprocess.check_output(['git','diff','79944df600e7cadefcdca3fae8d98838fa44116c','--','src'],cwd=R)==b'')
for p in (D/'tools').glob('*.py'):compile(p.read_text(),str(p),'exec')
check('documentation_tools_syntax',True)
spec=importlib.util.spec_from_file_location('stable_reader',R/'docs/p62_r23_real_imu_closure/tools/prepare_inputs.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
after=m.stable_snapshot();(D/'STABLE_AFTER.json').write_text(json.dumps(after,indent=2)+'\n')
check('stable_engineering_unchanged',after==json.loads((D/'STABLE_BEFORE.json').read_text()))
receipt={'status':'PASS','count':len(checks),'checks':checks,'pages':{'main':7,'supplement':4,'Chinese_memo':2},'abstract_words':len(a.split()),'references':14,'new_native_NDT_calls':0,'external_matching_algorithm_runs':0,'submission_performed':False,'scope':'publication/typesetting and selected preservation checks, not scientific novelty or acceptance certification'}
(D/'PACKAGE_VALIDATION.json').write_text(json.dumps(receipt,indent=2,ensure_ascii=False)+'\n')
print(json.dumps(receipt,indent=2,ensure_ascii=False))
