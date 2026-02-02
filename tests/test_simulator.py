import os
import tempfile
from simulation.generate_synthetic import make_session


def test_make_session_creates_files():
    td = tempfile.mkdtemp(prefix="planar_test_")
    make_session(td, n_stations=2, points_per_station=36)
    # basic assertions
    assert os.path.exists(os.path.join(td, "metadata.json"))
    assert os.path.exists(os.path.join(td, "events.json"))
    assert os.path.exists(os.path.join(td, "imu_log.csv"))
    assert os.path.exists(os.path.join(td, "lidar_station_0.csv"))
    assert os.path.exists(os.path.join(td, "lidar_station_1.csv"))
