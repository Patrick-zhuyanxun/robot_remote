"""Video encoding options of record_dataset.py (no ROS): streaming encoder keyframe interval, PNG-path keyframe
interval, and that an episode whose streaming encoder dropped frames is discarded instead of saved out of sync."""
import argparse
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "recording"))
import record_dataset as rd  # noqa: E402

try:
    import lerobot.datasets.lerobot_dataset as lds
except ImportError:  # pragma: no cover
    lds = None


class FakeEncoder:
    vcodec = "libsvtav1"

    def __init__(self, dropped=None):
        self.g = 2
        self._dropped_frames = dict(dropped or {})


class FakeDataset:
    def __init__(self, encoder=None):
        self._streaming_encoder = encoder
        self.saved = self.cleared = 0

    def save_episode(self, parallel_encoding=True):
        self.saved += 1

    def clear_episode_buffer(self):
        self.cleared += 1


def args(**kw):
    return argparse.Namespace(**{"video_gop": 30, "streaming_encode": True, "encoder_queue": 90, **kw})


class VideoOptionTests(unittest.TestCase):
    def test_streaming_encoder_gets_the_keyframe_interval(self):
        ds = rd.configure_video(FakeDataset(FakeEncoder()), args())
        self.assertEqual(ds._streaming_encoder.g, 30)
        ds = rd.configure_video(FakeDataset(FakeEncoder()), args(video_gop=0))  # 0 = LeRobot default
        self.assertEqual(ds._streaming_encoder.g, 2)

    def test_dropped_frames_discard_the_episode(self):
        ds = FakeDataset(FakeEncoder({"observation.images.head": 0}))
        self.assertTrue(rd.save_episode(ds))
        self.assertEqual((ds.saved, ds.cleared), (1, 0))
        ds = FakeDataset(FakeEncoder({"observation.images.head": 3}))
        self.assertFalse(rd.save_episode(ds))
        self.assertEqual((ds.saved, ds.cleared), (0, 1))
        ds = FakeDataset(None)  # PNG path: nothing to check
        self.assertTrue(rd.save_episode(ds))

    @unittest.skipIf(lds is None, "needs lerobot")
    def test_png_path_encodes_with_the_keyframe_interval(self):
        orig = lds.encode_video_frames
        seen = {}
        try:
            def fake(*a, g=2, **kw):
                seen["g"] = g
            fake.__signature__ = __import__("inspect").signature(orig)
            lds.encode_video_frames = fake
            rd.configure_video(FakeDataset(None), args())
            lds.encode_video_frames("imgs", "out.mp4", 30)
            self.assertEqual(seen["g"], 30)
            lds.encode_video_frames("imgs", "out.mp4", 30, g=5)  # an explicit g from LeRobot is kept
            self.assertEqual(seen["g"], 5)
        finally:
            lds.encode_video_frames = orig

    def test_streaming_only_where_lerobot_supports_it(self):
        def new_api(repo_id, streaming_encoding=False, encoder_queue_maxsize=30):
            pass

        def old_api(repo_id):
            pass
        self.assertEqual(rd.streaming_kwargs(args(), new_api), {"streaming_encoding": True, "encoder_queue_maxsize": 90})
        self.assertEqual(rd.streaming_kwargs(args(), old_api), {})
        self.assertEqual(rd.streaming_kwargs(args(streaming_encode=False), new_api), {})


if __name__ == "__main__":
    unittest.main()
