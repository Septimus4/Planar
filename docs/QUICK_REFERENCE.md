# Planar Quick Reference Card

## 🚀 Quick Start

```bash
# On Raspberry Pi
python -m capture.daemon

# Open Web UI at http://raspberrypi.local:8080
```

## 📍 Scanning Workflow

1. **Setup** → Level tripod, connect to Web UI
2. **Start Session** → Click "Start Session" button
3. **Capture** → Position tripod → Click "Mark Station"
4. **Move** → Reposition with 30-50% overlap
5. **Repeat** → Capture 4-8 stations around room
6. **Finish** → Click "Stop Session"
7. **Process** → Run pipeline on desktop

## 🎯 Key Distances

| Measurement | Recommended |
|-------------|-------------|
| LiDAR height | 1.0-1.2m |
| Distance from walls | >30cm |
| Station overlap | 30-50% |
| Min stations/room | 4-6 |

## 💻 Commands

```bash
# Status check
python -m desktop.controller --host PI_IP status

# Process session
python -m processing.pipeline --session sessions/NAME --out output/NAME
```

## ⚠️ Quick Fixes

| Problem | Solution |
|---------|----------|
| LiDAR not found | Check USB, run `sudo chmod 666 /dev/ttyUSB0` |
| IMU not found | Check wiring, run `i2cdetect -y 1` |
| Noisy scan | Stabilize tripod, move from glass |
| Missing walls | Glass surface - add manually in CAD |
| Won't connect | Verify Pi IP, check firewall port 8080 |

## 📊 Quality Checklist

- [ ] Tripod level (check bubble indicator)
- [ ] Wait 2 seconds before marking station
- [ ] Overlap with previous station
- [ ] Include corners/features in overlap
- [ ] Room clear of people/pets
- [ ] Same height for all stations

## 📁 Output Files

```
output/NAME/
├── floor_plan.dxf    # CAD file
├── merged_cloud.json # Point cloud
└── debug/            # Visualizations
```

## 🔧 Key Settings

```json
{
    "voxel_size_m": 0.02,      // Detail level
    "min_wall_length_m": 0.5,  // Filter short segments
    "merge_angle_deg": 5.0     // Wall merge tolerance
}
```

---

*Full documentation: docs/USER_GUIDE.md*
