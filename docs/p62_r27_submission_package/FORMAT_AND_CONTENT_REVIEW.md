# Final format and content review

The package is ready for advisor review, with author-information placeholders intentionally visible. It has not been submitted, uploaded or approved by authors. Formatting success is not a journal novelty/evidence verdict.

## Measured document properties

- Main manuscript:7US-Letter pages, IEEEtran journal double-column10pt;198-word single-paragraph abstract;5index terms;3contributions;4tables;3numbered main figures plus graphical abstract;8numbered equations;14resolved references.
- Supplement:4pages, including protocol lineage, archived counterexamples, quality-admission figure and resource/reference limitations.
- Chinese advisor summary:2A4pages; all8requested questions answered.
- Graphical abstract:660×295px PNG with300dpi metadata,33321bytes; vector PDF/SVG provided. The manuscript embeds the vector file rather than enlarging the PNG.
- Main and supplement final logs contain no errors, unresolved citations, missing characters or overfull boxes. Some ordinary underfull-box warnings remain; they do not crop content. The unmodified third-party IEEE class/style/README retain original whitespace; the scoped whitespace check excludes those vendor files rather than rewriting their contents. Generated SVG whitespace was normalized without changing plotted geometry.
- All main-PDF fonts are embedded. Page bounds, key frozen values, equation/citation resolution and figure links were checked. All7main pages were visually reviewed in a contact sheet; the causal table/error plot and final references were also inspected at enlarged resolution. The Chinese two-page memo was visually checked.

## Necessary fixes made during preparation

The first document build was attempted before the new graphic files existed and stopped on a missing graphical-abstract file; the graphics were generated and the complete build then succeeded. One helper initially used a Python3.9 string method on Python3.8; it was replaced before writing the intended bibliography changes. Supplementary prose had concatenated numeric tokens that overran a line; spacing was corrected and a standard emergency stretch removed the remaining minor line overrun. These were document-tool issues, not estimator or experimental failures.

The tie-break equation now displays dimensional normalization with explicit fractions. This describes the existing code; it does not change a threshold or mathematical algorithm. DOI fields were additionally printed through IEEEtran bibliography notes because the supplied legacy style does not display the raw BibTeX DOI field automatically. The compiled reference list includes all8paper DOIs and retains author-version provenance in the supplement.

## Scope checks

`PACKAGE_VALIDATION.json` records49passed checks. The selected R26inputs, frozen error/quality figure files, C+G algorithm source and stable-workspace identity are unchanged. Only the new R27publication directory is committed. No NDT, ROS, IMU replay, map build, external localization algorithm or new dataset was run/retrieved.

The two new schematics are explanatory vector figures. The memory bar chart changes internal-stage labels to global/local target and uses the same775308/138200KiB measurements. The error and coverage PDFs are byte-identical copies. No curve, reference error, or scientific output was recomputed to improve the presentation.

## Remaining author decisions

Names/order, affiliations, correspondence, funding/conflicts, originality/prior-publication declarations, final release links/permissions and exact AI-disclosure scope require author confirmation. The generic maintained IEEE journal template was used because the journal-specific selector/legacy download was inaccessible; this is disclosed rather than described as the Sensors-specific color package. Check any portal-specific styling after author approval.

The underlying evidence remains limited by one exposed recovery case, a conditional file/frame convention, no matched external comparison, and uncompleted physical robot calibration/deployment. The advisor should decide whether a narrowed system-design paper suffices or whether one frozen independent real-IMU confirmation is essential. This task does not execute that experiment.
