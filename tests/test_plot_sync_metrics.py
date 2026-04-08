import sys
import types
import importlib.util
import unittest
import tempfile
import os
import csv
import io
import math


def inject_fake_numpy():
    # Provide minimal numpy-like API used by the plotting helper
    np_mod = types.ModuleType("numpy")

    def nanmean(a):
        vals = [float(x) for x in a if not math.isnan(float(x))]
        if not vals:
            return float("nan")
        return sum(vals) / len(vals)

    def nanstd(a):
        vals = [float(x) for x in a if not math.isnan(float(x))]
        if not vals:
            return float("nan")
        mean = sum(vals) / len(vals)
        var = sum((x - mean) ** 2 for x in vals) / len(vals)
        return math.sqrt(var)

    np_mod.nanmean = nanmean
    np_mod.nanstd = nanstd
    sys.modules["numpy"] = np_mod


class TestPlotSyncMetrics(unittest.TestCase):
    def setUp(self):
        inject_fake_numpy()
        module_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "tools", "plot_sync_metrics.py"))
        spec = importlib.util.spec_from_file_location("plot_sync_metrics", module_path)
        self.pm = importlib.util.module_from_spec(spec)
        sys.modules["plot_sync_metrics"] = self.pm
        spec.loader.exec_module(self.pm)

    def test_read_and_summarize(self):
        tmpdir = tempfile.mkdtemp()
        path = os.path.join(tmpdir, "sync.csv")
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["timestamp", "ee_z_diff", "contact_fz_diff"])
            w.writerow([1.0, 0.01, 1.2])
            w.writerow([2.0, -0.02, -1.1])
            w.writerow([3.0, 0.0, 0.0])

        rows = self.pm.read_sync_csv(path)
        self.assertEqual(len(rows), 3)

        buf = io.StringIO()
        # capture stdout
        old = sys.stdout
        try:
            sys.stdout = buf
            self.pm.summarize(rows)
        finally:
            sys.stdout = old

        out = buf.getvalue()
        self.assertIn("Rows: 3", out)


if __name__ == "__main__":
    unittest.main()
