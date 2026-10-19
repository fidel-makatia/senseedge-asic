windows 103840, sequences 590
window accuracy 71.65 % (95 % CI 68.97 to 74.06), balanced 73.56 %, sequence majority 76.61 % (CI 72.9 to 80.0)

| true \ pred | HEALTHY | BEARING_WEAR | IMBALANCE | MISALIGNMENT | recall |
|---|---|---|---|---|---|
| HEALTHY | 1896 | 17 | 6 | 721 | 71.8 % |
| BEARING_WEAR | 4617 | 38994 | 7270 | 6319 | 68.2 % |
| IMBALANCE | 1544 | 951 | 14409 | 696 | 81.9 % |
| MISALIGNMENT | 6569 | 435 | 289 | 19107 | 72.4 % |

| condition | windows | accuracy |
|---|---|---|
| horizontal-misalignment/0.5mm | 2640 | 51.8 % |
| horizontal-misalignment/1.0mm | 2640 | 69.1 % |
| horizontal-misalignment/1.5mm | 2640 | 76.1 % |
| horizontal-misalignment/2.0mm | 2640 | 72.0 % |
| imbalance/10g | 2464 | 71.1 % |
| imbalance/15g | 2464 | 86.8 % |
| imbalance/20g | 2640 | 78.3 % |
| imbalance/25g | 2464 | 85.6 % |
| imbalance/30g | 2464 | 89.0 % |
| imbalance/35g | 2464 | 97.4 % |
| imbalance/6g | 2640 | 66.3 % |
| normal | 2640 | 71.8 % |
| overhang/ball_fault/0g | 2640 | 100.0 % |
| overhang/ball_fault/20g | 1408 | 100.0 % |
| overhang/ball_fault/35g | 1056 | 99.9 % |
| overhang/ball_fault/6g | 2288 | 99.9 % |
| overhang/cage_fault/0g | 2640 | 7.2 % |
| overhang/cage_fault/20g | 2640 | 65.0 % |
| overhang/cage_fault/35g | 2112 | 59.9 % |
| overhang/cage_fault/6g | 2640 | 44.8 % |
| overhang/outer_race/0g | 2640 | 48.9 % |
| overhang/outer_race/20g | 2640 | 60.9 % |
| overhang/outer_race/35g | 2112 | 51.4 % |
| overhang/outer_race/6g | 2640 | 55.2 % |
| underhang/ball_fault/0g | 2640 | 92.5 % |
| underhang/ball_fault/20g | 2640 | 99.8 % |
| underhang/ball_fault/35g | 1936 | 97.2 % |
| underhang/ball_fault/6g | 2640 | 96.6 % |
| underhang/cage_fault/0g | 2640 | 27.0 % |
| underhang/cage_fault/20g | 2640 | 43.5 % |
| underhang/cage_fault/35g | 2288 | 64.5 % |
| underhang/cage_fault/6g | 2464 | 58.0 % |
| underhang/outer_race/0g | 2640 | 85.0 % |
| underhang/outer_race/20g | 2640 | 74.7 % |
| underhang/outer_race/35g | 1936 | 69.0 % |
| underhang/outer_race/6g | 2640 | 75.0 % |
| vertical-misalignment/0.51mm | 2640 | 55.7 % |
| vertical-misalignment/0.63mm | 2640 | 59.0 % |
| vertical-misalignment/1.27mm | 2640 | 90.6 % |
| vertical-misalignment/1.40mm | 2640 | 82.6 % |
| vertical-misalignment/1.78mm | 2640 | 75.6 % |
| vertical-misalignment/1.90mm | 2640 | 91.2 % |
