# Bounded adversarial review disposition

Read-only reviewer: /root/p9_r1b_contract_review. No writes, optimizer execution,
network, GT, or delegation. Three bounded review cycles covered the contract,
implementation, and statistical interpretation. No external Codex CLI was
invoked for R1B without a new user confirmation.

Actionable findings and fixes:

1. Shared NEW labels could conceal a bisection transition. Supplemental complete-
   link groups now have distinct deterministic IDs; a regression test checks it.
2. Map digest was checked only in the initial stage. All stages now pin the
   map and bind U_obs/cohort/canonical/prepared sources to input_provenance.json.
3. Missing/changed samples could silently alter groups. Exact77/168/105 request
   coverage, unique IDs, seed metadata, accepted traces and pose reconstruction
   are validated. Main outputs retain350/350 non-timing parity after this fix.
4. Fixed-u reconstruction alone did not establish an actual refinement seed.
   The Python entry point validates the selected actual endpoint and exhaustive
   distinct-group coverage before invoking the low-level C++ refine mode.
5. Different complete-link labels did not guarantee representative separation.
   Global-profile witnesses now require separation from both the CLOSED anchor
   and selected canonical representative. The original main-only count3 becomes1.
   A synthetic near-anchor/near-representative regression guards this distinction.
6. Support and cusp hits at different beta boundaries could incorrectly combine
   into YES. Both predicates are now evaluated in the same boundary interval.
   The count changes YES1/PARTIAL2 to YES0/PARTIAL3. A regression reproduces it.
7. Supplemental local-cross endpoint groups were omitted from full-refine
   coverage. All23 receive one actual representative refinement;15 MAIN results
   are reused unchanged. Formal coverage becomes38/38 distinct PART A groups,
   not refinements from every beta or from the PART B pattern controls.

Supplemental representative preparation initially failed on an empty beta field
for local-cross seeds. This was localized to the tie-break key, changed to
request-ID ordering for NEW groups, and covered by a regression test before
any supplemental NDT ran. The evidence was preserved in the execution log.

Final limits: finite solver budgets and broad pose grouping cannot certify
strict dynamic local attractors. Lower-energy separated representative witnesses
do not certify branch minima or global minima. No formula replacement, dense
grid, transported W, probabilities or fusion is justified here. The bounded
review was stopped after actionable fixes and regression/consistency checks;
no fourth unchanged-artifact review or unconfirmed CLI invocation is claimed.
