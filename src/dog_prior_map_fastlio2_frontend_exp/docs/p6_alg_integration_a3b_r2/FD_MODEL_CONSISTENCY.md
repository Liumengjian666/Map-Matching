# Frozen-surrogate finite-difference consistency

The R2 synthetic factor makes its reliable basis depend on the state. The
pre-R2 dynamic evaluator is retained as a diagnostic reference and demonstrates
that relinearizing that basis at candidate states changes the objective
directional derivative relative to the frozen local `H/g` model.

For the R2 production contract, finite differences evaluate both perturbed
candidate states with the same snapshot `B_k` and `R_s,k`. The central
finite-difference derivative is compared with `2 g^T d`; the synthetic test
requires relative error `<= 1e-8`. The next-outer snapshot uses a changed
projector and repeats the consistency check with its new `H/g`.

The retained A3B-R1 forensic regression now explicitly asserts both sides of
the evidence: frozen-surrogate FD agrees with its local model at a tested step
size, while relinearized-B FD remains discrepant by the required margin. This
diagnostic is not used for production acceptance. Release and Debug targeted
tests pass.
