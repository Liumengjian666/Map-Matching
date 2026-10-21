# Tests and bounded adversarial review

Actual compiled C++ test executable:27 assertions passed, new_ndt_align=0, stdout/exit/compiler/source/binary hashes in BUILD_PROVENANCE.json. No claim that this retroactively repairs R11's float seed deviation. R12 adds no seed API.

User-required cases: legal nominal/legal duplicate; illegal nominal/legal duplicate; both illegal; multiple legal; differing native scores; sole legal member; nontransitive positions0/.09/.18; singleton with nominal competitor; negative D; D=.02 rejection; D>.02 positive; illegal final observation rejection; preserved raw nominal; caller permutation; nonfinite output. Historical non-overwrite is certified by actual hashes and distinct output paths, not a fake zero test.

Additional real C++ checks: every evidence opponent matters, first of two matching anchors wins, immutable original metadata, activeB3–8 and training299. The positive same-group replacement fixture also asserts it is NOT a physically different hypothesis. Tests do not estimate accuracy.

5 Python mock tests passed: success, nonzero process exit, timeout, start error (all exactly one mocked subprocess call), output path isolation. These test wrapper handling, not real OS timeout reliability or optimizer behavior. Actual successful replay independently exited0 and passed the actual asset gate.

Fresh-context local review was bounded to3 cycles for the pool/runner. Findings classified actionable: missing multi-opponent/nontransitive/partial-mask tests; ambiguous original-vs-derived metadata; failure-path AFTER records; physical-group-vs-recovery field naming. All addressed before real offline replay. Final directed review found no blocking defect. No fourth algorithm review cycle. User declined external second opinion; no external CLI.

CONTROL actual replay additionally reproduces212 duplicate labels,200 final decisions/observations,600 selector entries and2 historical D rows. Independent LP vertex enumeration checks all6 directed pair records. All raw eligible rows retain an eligible group representative. New representative64/1 does not change final acceptance.

One bounded fresh-context result review independently checked original rows, totals,64 evidence and all8 raw output identities without running replay/execute/summarize. It requested clarification that6 LP-verified variant rows contain duplicated values (not6 independent scientific samples), and that the group flag means an eligible member exists/preserved while the frame flag is a universal preservation invariant. Count label/report clarified only; no numerical result, C++ or raw CSV changed.
