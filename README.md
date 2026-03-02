# Walkthrough: Fix Jerky Motion & Leftward Drift
## Summary of Changes
### Root Causes Identified
| Symptom | Root Cause | Fix |
|---------|-----------|-----|
| Jerky motion | `SPEED_KI=1.5` too aggressive → oscillation | Reduced to `0.5` |
| Jerky motion | No speed ramp-up → step input | Added 600 ticks/s² ramp |
| Jerky motion | LP filter too fast (α=0.3) | Reduced to `0.15` |
| Leftward drift | Same FF gain both wheels | Per-wheel FF bias via `$BIAS` |
| Leftward drift | Heading PID too slow | KP 3→4, KI 0.1→0.3 |
### Files Modified
#### [robot.c](file:///home/johnw/Documents/VEDC/AMR/Core/Src/robot.c) — Core changes
```diff:robot.c

## New Serial Commands

| Command | Example | Purpose |
|---------|---------|---------|
| `bias <left> <right>` | `bias 1.5 1.6` | Set per-wheel FF gains |
| `spd <Kp> <Ki> <Kd>` | `spd 0.5 0.5 0` | Tune speed PID live |

## Verification

### Build ✅
```
make clean && make -j4
→ 0 errors, firmware: 49140 bytes text
```

## Testing Guide (on hardware)

1. **Flash**: `make flash`
2. **Connect**: `python3 tools/robot_control.py`
3. **Bật stream**: `stream on`
4. **Calibrate motor bias**:
   - `tst 300 300` → đọc speed 2 bên → tính ratio
   - Nếu L=200, R=180 → `bias 1.5 1.67` (1.5 × 200/180)
5. **Test forward**: `w` → xe phải ramp lên mượt, đi thẳng
6. **Fine-tune nếu cần**:
   - Vẫn giật? → `spd 0.3 0.3 0`
   - Vẫn lệch? → điều chỉnh bias

