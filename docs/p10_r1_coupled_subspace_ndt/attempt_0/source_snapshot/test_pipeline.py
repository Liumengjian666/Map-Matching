"""Synthetic orchestration checks, no real NDT or oracle input."""
import unittest
from run_experiment import METHODS, score_winners


class PipelineTests(unittest.TestCase):
    def test_score_winner_uses_score_then_fixed_order_without_labels(self):
        rows = [dict(transaction_id=tx, method=method, visit_order=str(order), grid_index=str(order),
                     refined_score_sum=str(score), full_ndt_status=status)
                for tx in ("616", "2226") for method in METHODS
                for order, score, status in ((0, 1., "SUCCESS"), (1, 2., "SUCCESS"),
                                             (2, 2., "SUCCESS"), (3, 100., "NOT_CONVERGED"))]
        winners = score_winners(rows)
        self.assertEqual(len(winners), 8)
        self.assertTrue(all(r["visit_order"] == "1" for r in winners))

    def test_no_success_is_missing_not_nominal_fake_winner(self):
        winners = score_winners([])
        self.assertTrue(all(r["visit_order"] == "" and r["status"] == "NO_SUCCESSFUL_TERMINAL" for r in winners))


if __name__ == "__main__":
    unittest.main()
