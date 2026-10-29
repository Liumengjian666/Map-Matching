#!/usr/bin/env python3
"""Post-seal evaluation only: reference never modifies the frozen shadow orders."""
import csv, hashlib, json, math, subprocess
from collections import Counter
from pathlib import Path
import numpy as np
from scipy.stats import spearmanr

P = Path(__file__).resolve().parents[1]
ROOT = P.parents[1]
START = '9f8b82710499e48e63efb09e74c86159846b3605'
SEAL_COMMIT = '6d1cc0d576bc8c4aa0ca4298725bb0e1c66fa933'
LABELS = ['MATERIAL_HARM', 'MATERIAL_BENEFIT', 'MIXED_MATERIAL', 'SMALL_OR_AMBIGUOUS']

def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def rows(p):
    with p.open() as f:
        return list(csv.DictReader(f))

def write(p, data):
    assert data
    with p.open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(data[0]), lineterminator='\n')
        w.writeheader(); w.writerows(data)

def dump(p, obj):
    p.write_text(json.dumps(obj, indent=2, allow_nan=False) + '\n')

def classify(dt, dr):
    harm = dt >= 0.10 or dr >= 2.0
    benefit = dt <= -0.10 or dr <= -2.0
    return ('MIXED_MATERIAL' if harm and benefit else 'MATERIAL_HARM' if harm
            else 'MATERIAL_BENEFIT' if benefit else 'SMALL_OR_AMBIGUOUS')

def auc(harm, benefit):
    if not harm or not benefit:
        return None
    # Pairwise probability, including half credit for equal original risk scores.
    return sum((h > b) + 0.5 * (h == b) for h in harm for b in benefit) / (len(harm) * len(benefit))

def corr(x, y):
    if len(x) < 2 or len(set(x)) < 2 or len(set(y)) < 2:
        return None
    return float(spearmanr(x, y).statistic if hasattr(spearmanr(x, y), 'statistic') else spearmanr(x, y).correlation)

def metric(ordered, labels):
    n = len(ordered); k = math.ceil(n / 10)
    counts = Counter(labels[int(r['raw_ordinal'])]['label'] for r in ordered)
    top = ordered[:k]
    tc = Counter(labels[int(r['raw_ordinal'])]['label'] for r in top)
    risks = [float(r['oriented_risk']) for r in ordered]
    hs = [float(r['oriented_risk']) for r in ordered if labels[int(r['raw_ordinal'])]['label'] == 'MATERIAL_HARM']
    bs = [float(r['oriented_risk']) for r in ordered if labels[int(r['raw_ordinal'])]['label'] == 'MATERIAL_BENEFIT']
    av = auc(hs, bs)
    hp = tc['MATERIAL_HARM'] / k if k else None
    bp = tc['MATERIAL_BENEFIT'] / k if k else None
    base = counts['MATERIAL_HARM'] / n if n else None
    sufficient = len(hs) >= 10 and len(bs) >= 10
    passes = sufficient and av >= .75 and hp >= 2 * base and bp <= .10
    return dict(eligible_n=n, top_k=k, **{x.lower() + '_n': counts[x] for x in LABELS},
                top_harm_n=tc['MATERIAL_HARM'], top_benefit_n=tc['MATERIAL_BENEFIT'],
                top_mixed_n=tc['MIXED_MATERIAL'], top_small_n=tc['SMALL_OR_AMBIGUOUS'],
                harm_base_fraction=base, top_harm_fraction=hp, top_benefit_fraction=bp,
                harm_recall_at_top10=tc['MATERIAL_HARM'] / len(hs) if hs else None,
                harm_enrichment=hp / base if base else None, harm_vs_benefit_auroc=av,
                auroc_unavailable_reason='' if av is not None else 'MISSING_HARM_OR_BENEFIT_CLASS',
                spearman_delta_t=corr(risks, [labels[int(r['raw_ordinal'])]['delta_e_t_m'] for r in ordered]),
                spearman_delta_R=corr(risks, [labels[int(r['raw_ordinal'])]['delta_e_R_deg'] for r in ordered]),
                enough_harm_and_benefit=sufficient, passes_exploratory_criteria=passes)

def self_test():
    assert classify(.10, 0) == 'MATERIAL_HARM'
    assert classify(0, 2) == 'MATERIAL_HARM'
    assert classify(-.10, 0) == 'MATERIAL_BENEFIT'
    assert classify(0, -2) == 'MATERIAL_BENEFIT'
    assert classify(.10, -2) == classify(-.10, 2) == 'MIXED_MATERIAL'
    assert classify(.0999, -1.9999) == 'SMALL_OR_AMBIGUOUS'
    assert auc([], [1]) is None and auc([2], [1]) == 1 and auc([1], [2]) == 0
    assert auc([1], [1]) == .5 and auc([1, 2], [1, 3]) == .375
    ls = {i: {'label': 'MATERIAL_HARM' if i == 2 else 'MATERIAL_BENEFIT',
              'delta_e_t_m': float(i), 'delta_e_R_deg': -float(i)} for i in range(11)}
    order = [{'raw_ordinal': i, 'oriented_risk': 11-i} for i in range(11)]
    m = metric(order, ls)
    assert m['top_k'] == 2 and m['top_harm_n'] == 0 and m['top_benefit_fraction'] == 1
    assert corr([1, 1], [1, 2]) is None and corr([1, 2], [1, 2]) > .999
    print('PASS inclusive labels, mixed priority, AUROC ties/missing, ceil Top10 and correlation semantics')

def main():
    self_test()
    assert not (P / 'REFERENCE_LABELS.csv').exists(), 'Do not overwrite formal evaluation'
    seal = json.loads((P / 'SHADOW_SEAL.json').read_text())
    for name, expected in seal['files'].items():
        assert sha(P / name) == expected, name
        committed = subprocess.check_output(['git', 'show', SEAL_COMMIT + ':' + str((P / name).relative_to(ROOT))], cwd=ROOT)
        assert hashlib.sha256(committed).hexdigest() == expected, name
    features = {int(r['raw_ordinal']): r for r in rows(P / 'SHADOW_FEATURES.csv')}
    rankings = rows(P / 'SHADOW_RANKINGS.csv')
    poses = json.loads((P / 'ONLINE_POSES.json').read_text())
    assert len(features) == len(poses) == 120 and len(rankings) == 2640
    # First reference access occurs only below, after committed shadow identities pass.
    frozen_path = 'docs/p62_r29_full_sequence_benchmark/tools/evaluate_batch.py'
    source = subprocess.check_output(['git', 'show', START + ':' + frozen_path], cwd=ROOT)
    ns = {'__name__': 'frozen_reference_helper', '__file__': str(ROOT / frozen_path)}
    exec(compile(source, frozen_path, 'exec'), ns)
    meta = json.loads((ROOT / 'docs/p63_r10_hdl_imu_full_hard/REFERENCE_METADATA.json').read_text())
    reference = ns['Reference'](meta)
    labels = []
    for p in poses:
        truth = reference.at(p['stamp_ns'])
        r = dict(window=p['window'], raw_ordinal=p['raw_ordinal'], stamp_ns=p['stamp_ns'], reference_valid=truth is not None)
        if truth is None:
            r.update(e_pred_t_m=None, e_pred_R_deg=None, e_ndt_t_m=None, e_ndt_R_deg=None,
                     delta_e_t_m=None, delta_e_R_deg=None, harm_t=None, harm_R=None, benefit_t=None,
                     benefit_R=None, label='REFERENCE_UNAVAILABLE', prediction_group='REFERENCE_UNAVAILABLE')
        else:
            pt, pr = ns['error'](np.array(p['prediction']), truth)
            nt, nr = ns['error'](np.array(p['ndt']), truth)
            dt, dr = nt-pt, nr-pr
            r.update(e_pred_t_m=pt, e_pred_R_deg=pr, e_ndt_t_m=nt, e_ndt_R_deg=nr,
                     delta_e_t_m=dt, delta_e_R_deg=dr, harm_t=dt >= .10, harm_R=dr >= 2,
                     benefit_t=dt <= -.10, benefit_R=dr <= -2, label=classify(dt, dr),
                     prediction_group='NOT_LOST' if pt <= .5 and pr <= 5 else 'ALREADY_LOST')
        labels.append(r)
    write(P / 'REFERENCE_LABELS.csv', labels)
    byraw = {r['raw_ordinal']: r for r in labels}
    metrics, benefit_rows, groups = [], [], []
    for scope in ['GLOBAL', 'D', 'E', 'F', 'G']:
        scope_labels = [r for r in labels if scope == 'GLOBAL' or r['window'] == scope]
        for group in ['ALL', 'NOT_LOST', 'ALREADY_LOST']:
            cohort = [r for r in scope_labels if r['reference_valid'] and (group == 'ALL' or r['prediction_group'] == group)]
            ids = {r['raw_ordinal'] for r in cohort}
            groups.append(dict(scope=scope, prediction_group=group, n=len(cohort), labels=dict(Counter(r['label'] for r in cohort))))
            for spec in json.loads((P / 'RANKING_RULES.json').read_text())['features']:
                name = spec['name']
                frozen = [r for r in rankings if r['scope'] == scope and r['feature'] == name]
                eligible = sorted([r for r in frozen if r['rank_highest_is_1'] and int(r['raw_ordinal']) in ids], key=lambda r: int(r['rank_highest_is_1']))
                m = dict(scope=scope, prediction_group=group, feature=name, direction=spec['direction'],
                         scope_frames=len(scope_labels), reference_and_group_n=len(cohort),
                         feature_missing_n=sum(not r['rank_highest_is_1'] and int(r['raw_ordinal']) in ids for r in frozen))
                m.update(metric(eligible, byraw)); metrics.append(m)
                for rank, r in enumerate(eligible, 1):
                    raw = int(r['raw_ordinal']); lab = byraw[raw]
                    if lab['label'] != 'MATERIAL_BENEFIT':
                        continue
                    benefit_rows.append(dict(scope=scope, prediction_group=group, feature=name, raw_ordinal=raw,
                        window=lab['window'], d_t_m=features[raw]['d_t_m'], d_R_deg=features[raw]['d_R_deg'],
                        delta_e_t_m=lab['delta_e_t_m'], delta_e_R_deg=lab['delta_e_R_deg'],
                        feature_value=r['feature_value'], frozen_scope_rank=r['rank_highest_is_1'],
                        cohort_rank=rank, eligible_n=len(eligible), top_k=m['top_k'], in_top10=rank <= m['top_k']))
    assert len(metrics) == 165
    write(P / 'RANKING_METRICS.csv', metrics)
    if benefit_rows:
        write(P / 'BENEFICIAL_CORRECTION_COUNTEREXAMPLES.csv', benefit_rows)
    global_metrics = [m for m in metrics if m['scope'] == 'GLOBAL' and m['prediction_group'] == 'ALL']
    sufficient = [m for m in global_metrics if m['enough_harm_and_benefit']]
    passes = [m['feature'] for m in sufficient if m['passes_exploratory_criteria']]
    # Insufficient category coverage has preregistered priority over descriptive separation.
    verdict = ('EVALUATION_INSUFFICIENT' if not sufficient else
               'SINGLE_FEATURE_RISK_RANKING_SUPPORTED' if passes else 'REQUIRES_REPORTED_MIXED_OR_NOT_SUPPORTED_ASSESSMENT')
    result = dict(task='P63-R12-HARMFUL-NDT-RISK-SHADOW-RANKING', verdict=verdict, START_SHA=START,
        FEATURE_CODE_SHA=seal['CODE_SHA'], SHADOW_SEAL_COMMIT=SEAL_COMMIT,
        EVALUATION_CODE_SHA=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        formal_frames=120, ranking_features=11, reference_valid=sum(r['reference_valid'] for r in labels),
        label_counts=dict(Counter(r['label'] for r in labels)), cohorts=groups,
        qualifying_global_features=passes, global_metrics=global_metrics,
        native_ndt_calls=0, ukf_calls=0, ros_replays=0, new_candidates=0, online_feedback=0,
        reference=meta, reference_helper_sha256=hashlib.sha256(source).hexdigest(),
        current_ukf_used_as_feature=False, heldout_scope='same failed R10 trajectory; four fixed new windows, not independent dataset',
        seal_sha256=sha(P / 'SHADOW_SEAL.json'))
    dump(P / 'RESULTS.json', result)
    dump(P / 'EVALUATION_SEAL.json', {f: sha(P / f) for f in ['REFERENCE_LABELS.csv', 'RANKING_METRICS.csv', 'BENEFICIAL_CORRECTION_COUNTEREXAMPLES.csv', 'RESULTS.json']})
    print(json.dumps({k: result[k] for k in ['verdict', 'reference_valid', 'label_counts', 'qualifying_global_features']}, indent=2))

if __name__ == '__main__':
    import sys
    self_test() if '--self-test' in sys.argv else main()
