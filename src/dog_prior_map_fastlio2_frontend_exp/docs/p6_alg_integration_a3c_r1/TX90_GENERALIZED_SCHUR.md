# tx90 generalized-Schur oracle

The independent Python/NumPy oracle consumes the saved capsule, not the
production Schur helper. The capsule contains the full consumed `H_c`, `g_c`,
incoming and charted prior, factor-component Hessians, and the actual
production solve corrections. It is compressed as a single NPZ fixture.

For tx90:

- `H_mm` primary numerical threshold:
  `eps * 15 * max_abs_eigenvalue = 3.3307194895e-6`;
- primary rank/nullity: 15 / 0;
- sensitivity thresholds `1e-14`, `1e-12`, `1e-10` times the spectral scale
  also gave rank 15 / nullity 0;
- `||(I-H_mm H_mm+) H_mr|| / ||H_mr|| = 0` for every threshold;
- `||(I-H_mm H_mm+) g_m|| / ||g_m|| = 0` for every threshold.

Because `H_mm` is full rank here, the generalized inverse equals the inverse
up to numerical solution error; no nullspace inconsistency is present.

| Quantity | Production-correction Schur | Generalized pseudoinverse oracle |
|---|---:|---:|
| `lambda_min` | 0 | 0 |
| `lambda_max` | 20991.159274611786 | 20991.159275151440 |
| relative negative | 0 | 0 |
| Frobenius norm | 33427.401059308160 | 33427.401060057430 |

The relative Frobenius matrix error is `1.2413274714e-10` (absolute
Frobenius difference `4.1494351232e-6`). Gradient relative error is
`3.4026656841e-19` (absolute 2-norm difference `1.3877787808e-17`). An
independent direct solve using the production-symmetrized `H_mm` gives a
relative Schur matrix error of `1.3811890204e-11`; its gradient difference is
zero at reported precision.

The reported `lambda_min`/`lambda_max` are spectra of symmetrized forensic
matrices evaluated by the independent oracle. They do not mean production
evaluated the in-place candidate's spectrum: production rejects at the
preceding elementwise symmetry check, before its PSD eigenvalue check. The
capsule-based C++ emulation separately replays that exact in-place expression
and confirms the symmetry-gate rejection.

`TX90_JITTER_USED = NO`. The production-correction Schur reconstructed from
the saved production solve and the generalized Schur oracle agree to the
errors above. The Moore-Penrose computation is forensic only and was not
introduced into production.
