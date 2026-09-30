# Dense / block / sparse equivalence

`WindowLinearSystem` stores upper 15x15 state blocks, gradient and unchanged
cost. IMU connects two nodes, LiDAR one, visual ref/current two. Dense prior
blocks are accepted. Prior chart J is block diagonal, so assembly computes
`J_i^T H_prior_ij J_j` and `J_i^T(H_prior d+g)_i` without dense J products.

The original independent dense assembler remains available via
`linearizedSystem`; block assembly is not tested solely against its own dense
conversion. Tests include noisy/conflicting factors, nonzero gradient, joint
visual rank-two factors, nonzero SO3 chart displacement and repeated Schur
marginalization. H and gradient use relative norm error; cost uses absolute
error, with unchanged limits 1e-10 / 1e-10 / 1e-12.

Observed maxima:

| Quantity | Error |
|---|---:|
| relative H | 7.52677e-19 |
| relative gradient | 1.01248e-16 |
| absolute cost | 1.73472e-18 |
| relative dense/sparse step | 6.06171e-15 |
| final 15D state difference | 3.86139e-16 |

Solver parity compares dense LDLT and sparse SimplicialLDLT on **the same** H/g,
after independently proving assembly parity. Damping values 1e-6, 0.01 and 1
use `H_lambda = H + lambda diag(max(abs(Hii),1))`; relative step limit is 1e-9.
Full optimizer trajectories are compared as well, not only one linear solve.

An initial test attempted relative step comparison using two independently
accumulated gradients after convergence: step norm 1.09794e-13, absolute
difference 1.27018e-19. That measures amplified accumulation roundoff rather
than solver parity. The final test separates assembly and same-system solve;
no stated acceptance bound was enlarged. H/g/cost parity is still checked
after convergence and after every tested marginalization.

An explicitly indefinite synthetic system verifies that sparse failure emits
`SPARSE_SOLVER_FALLBACK_DENSE`. Fallback is never silent. Normal synthetic
systems do not require it. Debug and Release both pass.
