#!/usr/bin/env python3
"""Create a separate R28 manuscript without changing R27 or experimental code."""
import shutil
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
OLD = HERE.parent / 'p62_r27_submission_package'
s = (OLD / 'PAPER_SUBMISSION_DRAFT.tex').read_text()


def replace(old, new):
    global s
    assert s.count(old) == 1, old[:100]
    s = s.replace(old, new)


old = r'''The frozen Base interpretation is supported by the dataset paper and pinned robot-model evidence~\cite{newer,ncframe}; a direct identity binding to the local CSV-export axes remains unresolved. Official documentation distinguishes the stereo-camera and multicamera pose frames~\cite{ncref}. We retain one declared interpretation, perform no error-driven frame switching, and make all accuracy conclusions conditional on this chain.'''
new = r'''The historical Base interpretation is supported by the dataset paper and pinned robot-model evidence~\cite{newer,ncframe}, but its binding to the actual CSV is not certified. A dedicated follow-up verified that the local reference is the original 2020 Stereo-Cam release, with a retained official download identity and unchanged checksum. However, the official reference webpage assigns stereo poses to RS\_C1~\cite{ncref}, and the platform page defines RS\_C1 as optical while distinguishing a co-located robotics-axis Base~\cite{ncplatform}. The pinned robot description instead says that ground truth is expressed in Base~\cite{ncframe}. No inspected per-file metadata or exporter resolves this contradiction.

Let $T_{A B}$ map coordinates from $B$ into $A$, with $C$ the left optical frame, $B$ the robotics Base, $I$ the Ouster IMU and $L$ the cloud frame. The supplied Kalibr and Ouster calibrations give $T_{C I}$ and $T_{I L}$. The two documented interpretations imply different chains:
\begin{align}
 \text{Optical:}\quad T_{M L}&=T_{\mathrm{CSV}}T_{C I}T_{I L},\nonumber\\
 T_{M I}&=T_{\mathrm{CSV}}T_{C I},\label{eq:opticalchain}\\
 \text{Historical Base:}\quad T_{M L}&=T_{\mathrm{CSV}}T_{B C}T_{C I}T_{I L},\nonumber\\
 T_{M I}&=T_{\mathrm{CSV}}T_{B C}T_{C I}.\label{eq:basechain}
\end{align}
Co-location does not make $T_{B C}$ identity: its rotation changes optical axes into robotics axes. Both calibrated chains pass rigid-transform and inverse checks, which cannot certify CSV semantics. We do not switch interpretations by overlap, alignment fit, or minimum reference error. Historical errors below retain their explicitly conditional Base convention; they are not certified absolute accuracy. New reference-dependent experiments remain blocked until the exact-file convention is resolved.'''
replace(old, new)
marker = r'\subsection{Evaluation and Resource Measurement}'
insert = r'''\subsection{Prospective Independent and External Comparisons}
Before any follow-up accuracy evaluation, the proposed anchor raw11000 was found to overlap the previously exposed raw10972--11032 interval. It was replaced deterministically by the next thousand-frame anchor after that interval, raw12000; the second anchor remains raw13500. Each proposed window contains one anchor and 60 evaluation scans. All 122 raw scan identities and integer timestamps were frozen without reading reference poses. Sparse earlier input-geometry screening is disclosed; these windows are not claimed to contain never-inspected geometry.

The planned comparison uses Original, G-only and C+G with normal and $+8^\circ$ initialization, each with an independent real IMU/filter history. This defines 12 prospective trajectories, not 12 completed runs. Source/target and anchor initialization remain unprepared behind the reference-frame gate. A finite feasibility inspection of FAST-LIO-LOCALIZATION~\cite{fastlioloc} found Python/Open3D interface mismatches without changing the installed environment. Available PCL GICP headers identify a possible controlled registration baseline, not a validated external localization system. Neither method has been run for a matched comparison.

'''
replace(marker, insert + marker)
marker = r'\section{Discussion and Limitations}'
insert = r'''\subsection{Independent Causal Validation: Not Executed}
The reference-coordinate gate did not pass. Consequently, none of the 12 prospective independent causal trajectories was executed, and no new recovery rate, normal-initialization safety result or reference-relative error is reported. Non-execution must not be interpreted as a non-triggering successful run. The development-case results in Tables~\ref{tab:ablation} and~\ref{tab:resources} are preserved and are not upgraded into independent confirmation.

\subsection{Matched External Baseline: Not Established}
Only upstream-source and installed-environment feasibility were checked. No external algorithm was compiled or run, and no matched-baseline accuracy table is available. Published full-sequence numbers in Table~\ref{tab:published} remain contextual results under different protocols. They cannot support a state-of-the-art ranking for the proposed disturbed-window experiment.

'''
replace(marker, insert + marker)
replace('The map-anchored reference and conditional pose-frame interpretation limit accuracy conclusions.',
        'The map-anchored reference and unresolved exact-file pose convention limit accuracy conclusions. Resolving the conflict requires authoritative exporter metadata or a file-specific clarification, rather than choosing the transform that produces smaller errors. If that evidence identifies an error in the historical interpretation, every affected reference evaluation must be recomputed as a new version while retaining the original logs. A changed initialization convention also affects the causal experiment itself; re-evaluating old outputs alone cannot recreate a corrected run.')
replace('No external state-of-the-art superiority or new optimization theory is established.',
        'The follow-up coordinate-verification gate remains blocked, so neither independent recovery confirmation nor a matched external comparison has been established. The current manuscript is for advisor review with these prerequisites explicit, rather than an accuracy claim ready for formal submission. No external state-of-the-art superiority or new optimization theory is established.')
replace('All 60 reference-relative fused-state errors in W1/C3.',
        'All 60 historical, conditional-Base reference-relative fused-state errors in W1/C3.')

# Keep R27 abstract/introduction/method unchanged; R28 revisions are confined
# to protocol, results, discussion, conclusion and their supporting references.
assert len(s.split(r'\begin{abstract}')[1].split(r'\end{abstract}')[0].split()) <= 250
(HERE / 'UPDATED_MANUSCRIPT.tex').write_text(s)
for name in ['IEEEtran.cls', 'IEEEtran.bst', 'REFERENCES.bib']:
    shutil.copy2(OLD / name, HERE / name)
with (HERE / 'REFERENCES.bib').open('a') as f:
    f.write(r'''
@misc{ncplatform,
 author={{Oxford Robotics Institute}}, title={{Newer College} Stereo-Camera Platform},
 year={2026}, note={Accessed: Oct. 24, 2026},
 url={https://ori-drs.github.io/newer-college-dataset/stereo-cam/platform-stereo/}}
@misc{fastlioloc,
 author={{HViktorTsoi}}, title={{FAST-LIO-LOCALIZATION}: Official Source and Documentation},
 year={2026}, note={Source snapshot and feasibility inspected Oct. 24, 2026; no execution claimed},
 url={https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION}}
''')
shutil.copytree(OLD / 'figures', HERE / 'figures', dirs_exist_ok=True)
print('Created separate editable manuscript; historical source untouched.')
