import json
import sys
import types
from pathlib import Path


ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT))

rerun_visualizer = types.ModuleType("teleop.utils.rerun_visualizer")
rerun_visualizer.RerunLogger = object
sys.modules[rerun_visualizer.__name__] = rerun_visualizer

logging_mp = types.ModuleType("logging_mp")
logging_mp.getLogger = lambda _name: types.SimpleNamespace(info=lambda *_args, **_kwargs: None)
sys.modules[logging_mp.__name__] = logging_mp

from teleop.utils.episode_writer import EpisodeWriter


def test_episode_writer_records_monotonic_timestamps(tmp_path):
    writer = EpisodeWriter(str(tmp_path / "task"), frequency=30, rerun_log=False)
    assert writer.create_episode()
    writer.add_item({}, states={}, actions={})
    writer.add_item({}, states={}, actions={})
    writer.close()

    episode = json.loads((tmp_path / "task" / "episode_0000" / "data.json").read_text())
    assert episode["info"]["timestamps"]["clock"] == "monotonic"
    assert episode["info"]["timestamps"]["episode_timestamp"].endswith("+00:00")
    frames = episode["data"]
    for frame in frames:
        assert all(name in frame for name in ("timestamp", "frame_timestamp", "observation_timestamp", "action_timestamp"))
    assert [frame["timestamp"] for frame in frames] == sorted(frame["timestamp"] for frame in frames)
