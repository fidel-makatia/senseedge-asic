# mafaulda_d70: 8-16-4 INT8 on the corrected arithmetic, seed 0

## test split (host model, what the board must reproduce)
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

## val split
windows 170898, sequences 273
window accuracy 63.03 % (95 % CI 59.49 to 66.85), balanced 64.31 %, sequence majority 69.23 % (CI 63.7 to 74.7)

| true \ pred | HEALTHY | BEARING_WEAR | IMBALANCE | MISALIGNMENT | recall |
|---|---|---|---|---|---|
| HEALTHY | 3275 | 168 | 334 | 605 | 74.7 % |
| BEARING_WEAR | 9870 | 62771 | 14051 | 6582 | 67.3 % |
| IMBALANCE | 7667 | 2596 | 17974 | 1185 | 61.1 % |
| MISALIGNMENT | 16784 | 2182 | 1149 | 23705 | 54.1 % |

| condition | windows | accuracy |
|---|---|---|
| horizontal-misalignment/0.5mm | 3130 | 24.7 % |
| horizontal-misalignment/1.0mm | 7512 | 35.2 % |
| horizontal-misalignment/1.5mm | 5634 | 86.6 % |
| horizontal-misalignment/2.0mm | 6260 | 81.9 % |
| imbalance/10g | 6260 | 65.2 % |
| imbalance/15g | 3756 | 48.4 % |
| imbalance/20g | 3756 | 73.3 % |
| imbalance/25g | 2504 | 74.5 % |
| imbalance/30g | 5008 | 72.2 % |
| imbalance/35g | 5008 | 66.2 % |
| imbalance/6g | 3130 | 16.7 % |
| normal | 4382 | 74.7 % |
| overhang/ball_fault/0g | 5634 | 97.6 % |
| overhang/ball_fault/20g | 2504 | 99.9 % |
| overhang/ball_fault/35g | 3756 | 99.7 % |
| overhang/ball_fault/6g | 3756 | 99.7 % |
| overhang/cage_fault/0g | 3130 | 62.6 % |
| overhang/cage_fault/20g | 6886 | 72.7 % |
| overhang/cage_fault/35g | 1878 | 60.5 % |
| overhang/cage_fault/6g | 5008 | 66.4 % |
| overhang/outer_race/0g | 3756 | 81.3 % |
| overhang/outer_race/20g | 2504 | 81.7 % |
| overhang/outer_race/35g | 3756 | 69.6 % |
| overhang/outer_race/6g | 3130 | 82.8 % |
| underhang/ball_fault/0g | 1878 | 95.0 % |
| underhang/ball_fault/20g | 5634 | 80.8 % |
| underhang/ball_fault/35g | 3756 | 95.7 % |
| underhang/ball_fault/6g | 6260 | 96.9 % |
| underhang/cage_fault/0g | 5634 | 8.6 % |
| underhang/cage_fault/20g | 4382 | 35.0 % |
| underhang/cage_fault/35g | 3130 | 18.9 % |
| underhang/cage_fault/6g | 1252 | 17.7 % |
| underhang/outer_race/0g | 3130 | 68.4 % |
| underhang/outer_race/20g | 5634 | 35.5 % |
| underhang/outer_race/35g | 3130 | 31.2 % |
| underhang/outer_race/6g | 3756 | 43.0 % |
| vertical-misalignment/0.51mm | 3130 | 32.5 % |
| vertical-misalignment/0.63mm | 3130 | 27.1 % |
| vertical-misalignment/1.27mm | 3756 | 54.8 % |
| vertical-misalignment/1.40mm | 4382 | 50.2 % |
| vertical-misalignment/1.78mm | 4382 | 56.7 % |
| vertical-misalignment/1.90mm | 2504 | 66.7 % |

## fit split
windows 681088, sequences 1088
window accuracy 62.83 % (95 % CI 61.03 to 64.62), balanced 66.48 %, sequence majority 72.24 % (CI 69.6 to 74.9)

| true \ pred | HEALTHY | BEARING_WEAR | IMBALANCE | MISALIGNMENT | recall |
|---|---|---|---|---|---|
| HEALTHY | 13482 | 584 | 748 | 2088 | 79.8 % |
| BEARING_WEAR | 44803 | 244814 | 57925 | 26180 | 65.5 % |
| IMBALANCE | 23186 | 8286 | 81518 | 3446 | 70.0 % |
| MISALIGNMENT | 70824 | 9599 | 5459 | 88146 | 50.7 % |

| condition | windows | accuracy |
|---|---|---|
| horizontal-misalignment/0.5mm | 18780 | 23.1 % |
| horizontal-misalignment/1.0mm | 13772 | 25.3 % |
| horizontal-misalignment/1.5mm | 15650 | 79.0 % |
| horizontal-misalignment/2.0mm | 15024 | 84.9 % |
| imbalance/10g | 15024 | 60.0 % |
| imbalance/15g | 17528 | 72.6 % |
| imbalance/20g | 17528 | 78.9 % |
| imbalance/25g | 18154 | 79.8 % |
| imbalance/30g | 15650 | 79.5 % |
| imbalance/35g | 14398 | 82.9 % |
| imbalance/6g | 18154 | 38.8 % |
| normal | 16902 | 79.8 % |
| overhang/ball_fault/0g | 15650 | 98.6 % |
| overhang/ball_fault/20g | 8138 | 99.9 % |
| overhang/ball_fault/35g | 5008 | 99.5 % |
| overhang/ball_fault/6g | 15024 | 99.7 % |
| overhang/cage_fault/0g | 18154 | 63.0 % |
| overhang/cage_fault/20g | 14398 | 75.6 % |
| overhang/cage_fault/35g | 16276 | 71.6 % |
| overhang/cage_fault/6g | 16276 | 71.2 % |
| overhang/outer_race/0g | 17528 | 80.0 % |
| overhang/outer_race/20g | 18780 | 78.6 % |
| overhang/outer_race/35g | 14398 | 78.4 % |
| overhang/outer_race/6g | 18154 | 80.0 % |
| underhang/ball_fault/0g | 20032 | 93.7 % |
| underhang/ball_fault/20g | 15650 | 96.3 % |
| underhang/ball_fault/35g | 13146 | 93.4 % |
| underhang/ball_fault/6g | 15024 | 86.2 % |
| underhang/cage_fault/0g | 15650 | 6.7 % |
| underhang/cage_fault/20g | 16902 | 37.3 % |
| underhang/cage_fault/35g | 15024 | 26.1 % |
| underhang/cage_fault/6g | 20032 | 18.5 % |
| underhang/outer_race/0g | 18154 | 60.0 % |
| underhang/outer_race/20g | 15650 | 32.1 % |
| underhang/outer_race/35g | 13146 | 25.3 % |
| underhang/outer_race/6g | 17528 | 44.7 % |
| vertical-misalignment/0.51mm | 19406 | 31.4 % |
| vertical-misalignment/0.63mm | 18780 | 28.0 % |
| vertical-misalignment/1.27mm | 18154 | 62.1 % |
| vertical-misalignment/1.40mm | 17528 | 55.2 % |
| vertical-misalignment/1.78mm | 17528 | 61.6 % |
| vertical-misalignment/1.90mm | 19406 | 62.4 % |
