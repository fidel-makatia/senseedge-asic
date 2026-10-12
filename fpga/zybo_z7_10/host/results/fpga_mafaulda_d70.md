windows 369340, sequences 590
window accuracy 63.87 % (95 % CI 61.36 to 66.08), balanced 66.51 %, sequence majority 71.69 % (CI 67.8 to 75.1)

| true \ pred | HEALTHY | BEARING_WEAR | IMBALANCE | MISALIGNMENT | recall |
|---|---|---|---|---|---|
| HEALTHY | 7152 | 330 | 443 | 1465 | 76.2 % |
| BEARING_WEAR | 22824 | 134944 | 31907 | 13775 | 66.3 % |
| IMBALANCE | 11745 | 4688 | 44417 | 1750 | 71.0 % |
| MISALIGNMENT | 36730 | 5236 | 2567 | 49367 | 52.6 % |

| condition | windows | accuracy |
|---|---|---|
| horizontal-misalignment/0.5mm | 9390 | 28.1 % |
| horizontal-misalignment/1.0mm | 9390 | 27.5 % |
| horizontal-misalignment/1.5mm | 9390 | 86.9 % |
| horizontal-misalignment/2.0mm | 9390 | 84.8 % |
| imbalance/10g | 8764 | 61.7 % |
| imbalance/15g | 8764 | 67.0 % |
| imbalance/20g | 9390 | 70.9 % |
| imbalance/25g | 8764 | 78.9 % |
| imbalance/30g | 8764 | 83.0 % |
| imbalance/35g | 8764 | 88.5 % |
| imbalance/6g | 9390 | 48.4 % |
| normal | 9390 | 76.2 % |
| overhang/ball_fault/0g | 9390 | 99.7 % |
| overhang/ball_fault/20g | 5008 | 99.9 % |
| overhang/ball_fault/35g | 3756 | 99.7 % |
| overhang/ball_fault/6g | 8138 | 99.8 % |
| overhang/cage_fault/0g | 9390 | 64.0 % |
| overhang/cage_fault/20g | 9390 | 73.5 % |
| overhang/cage_fault/35g | 7512 | 75.3 % |
| overhang/cage_fault/6g | 9390 | 73.6 % |
| overhang/outer_race/0g | 9390 | 79.9 % |
| overhang/outer_race/20g | 9390 | 81.5 % |
| overhang/outer_race/35g | 7512 | 73.6 % |
| overhang/outer_race/6g | 9390 | 82.9 % |
| underhang/ball_fault/0g | 9390 | 85.7 % |
| underhang/ball_fault/20g | 9390 | 96.0 % |
| underhang/ball_fault/35g | 6886 | 89.7 % |
| underhang/ball_fault/6g | 9390 | 91.6 % |
| underhang/cage_fault/0g | 9390 | 6.5 % |
| underhang/cage_fault/20g | 9390 | 26.8 % |
| underhang/cage_fault/35g | 8138 | 34.0 % |
| underhang/cage_fault/6g | 8764 | 26.4 % |
| underhang/outer_race/0g | 9390 | 56.0 % |
| underhang/outer_race/20g | 9390 | 32.8 % |
| underhang/outer_race/35g | 6886 | 29.3 % |
| underhang/outer_race/6g | 9390 | 46.5 % |
| vertical-misalignment/0.51mm | 9390 | 32.8 % |
| vertical-misalignment/0.63mm | 9390 | 25.1 % |
| vertical-misalignment/1.27mm | 9390 | 58.9 % |
| vertical-misalignment/1.40mm | 9390 | 58.8 % |
| vertical-misalignment/1.78mm | 9390 | 60.1 % |
| vertical-misalignment/1.90mm | 9390 | 62.7 % |
