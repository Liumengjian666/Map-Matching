# Independent read-only review record

These are single-model fresh-context artifact/contract reviews, not an external
CLI or cross-model second opinion. The user chose the current independent review
only. Reviewers did not execute experiments or load GT, held-out labels or
scientific outcome data. Their static conclusions are not scientific R4 results.

## Historical selection/label geometry

The first review identified a material boundary error: converting float operands
to double before subtraction/norm did not reproduce the original P9 float
major-separation predicate. The implementation now extracts the literal frozen
C++ carrier and tests the .12/.16 translation boundary. Historical
24-frame/9-MAJOR/15-NO_MAJOR/22-ID parity was rerun afterward and passed.

## NDT execution recovery

The review required refusing a previously started or partial frame and reporting
converged zero-iteration returns truthfully. Exclusive per-frame started markers
and partial/completed checks now prevent unaccounted retries. Zero-iteration
status is distinguished without changing frozen `converged==1` evidence
admission; converged iteration-limit terminals remain retained. Generated source
pins now use explicit failures rather than optimization-removable Python asserts.

## Unexecuted blind evidence scaffold

Required fixes were exact cohort hash/IDs/count/row coverage binding across
candidate, visual and evidence stages; checking frozen calibration input hashes
before recalculating candidate IMU poses; and rejecting readable append modes and
historical canonical inputs. Pure synthetic regression fixtures were added. A
follow-up static review found no remaining required issues. Every scientific stage
also refuses the existing input STOP receipt. The scaffold has not generated new
scientific evidence and is not experimentally validated.

## Input-stop verification

The archive review required checking candidate/visual partial-start artifacts,
not only oracle output files, and verifying numerical counts/hashes rather than
trusting CSV PASS/FAIL text. The verifier now binds diagnostic receipt hashes,
re-executes only the read-only source auditor, checks manifest identities and raw
file hashes, and recomputes each parity flag from its recorded numerical values.
This closes archive integrity checks without restarting a scientific stage.
