# mafaulda_d249: 8-16-4 INT8 on the corrected arithmetic, seed 3

## test split (host model, what the board must reproduce)
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

## val split
windows 48048, sequences 273
window accuracy 70.50 % (95 % CI 66.82 to 74.18), balanced 73.87 %, sequence majority 74.36 % (CI 69.6 to 79.1)

| true \ pred | HEALTHY | BEARING_WEAR | IMBALANCE | MISALIGNMENT | recall |
|---|---|---|---|---|---|
| HEALTHY | 1003 | 6 | 8 | 215 | 81.4 % |
| BEARING_WEAR | 1977 | 17804 | 3638 | 2805 | 67.9 % |
| IMBALANCE | 1367 | 364 | 6016 | 525 | 72.7 % |
| MISALIGNMENT | 2838 | 204 | 227 | 9051 | 73.5 % |

| condition | windows | accuracy |
|---|---|---|
| horizontal-misalignment/0.5mm | 880 | 44.9 % |
| horizontal-misalignment/1.0mm | 2112 | 56.1 % |
| horizontal-misalignment/1.5mm | 1584 | 77.7 % |
| horizontal-misalignment/2.0mm | 1760 | 76.5 % |
| imbalance/10g | 1760 | 77.6 % |
| imbalance/15g | 1056 | 60.4 % |
| imbalance/20g | 1056 | 94.4 % |
| imbalance/25g | 704 | 75.1 % |
| imbalance/30g | 1408 | 84.7 % |
| imbalance/35g | 1408 | 75.1 % |
| imbalance/6g | 880 | 26.9 % |
| normal | 1232 | 81.4 % |
| overhang/ball_fault/0g | 1584 | 99.4 % |
| overhang/ball_fault/20g | 704 | 100.0 % |
| overhang/ball_fault/35g | 1056 | 100.0 % |
| overhang/ball_fault/6g | 1056 | 100.0 % |
| overhang/cage_fault/0g | 880 | 0.7 % |
| overhang/cage_fault/20g | 1936 | 53.4 % |
| overhang/cage_fault/35g | 528 | 48.5 % |
| overhang/cage_fault/6g | 1408 | 28.0 % |
| overhang/outer_race/0g | 1056 | 48.4 % |
| overhang/outer_race/20g | 704 | 73.3 % |
| overhang/outer_race/35g | 1056 | 46.7 % |
| overhang/outer_race/6g | 880 | 54.0 % |
| underhang/ball_fault/0g | 528 | 99.8 % |
| underhang/ball_fault/20g | 1584 | 91.7 % |
| underhang/ball_fault/35g | 1056 | 99.7 % |
| underhang/ball_fault/6g | 1760 | 98.0 % |
| underhang/cage_fault/0g | 1584 | 32.4 % |
| underhang/cage_fault/20g | 1232 | 56.6 % |
| underhang/cage_fault/35g | 880 | 47.3 % |
| underhang/cage_fault/6g | 352 | 43.8 % |
| underhang/outer_race/0g | 880 | 95.5 % |
| underhang/outer_race/20g | 1584 | 60.2 % |
| underhang/outer_race/35g | 880 | 73.6 % |
| underhang/outer_race/6g | 1056 | 71.0 % |
| vertical-misalignment/0.51mm | 880 | 74.7 % |
| vertical-misalignment/0.63mm | 880 | 60.2 % |
| vertical-misalignment/1.27mm | 1056 | 90.4 % |
| vertical-misalignment/1.40mm | 1232 | 90.7 % |
| vertical-misalignment/1.78mm | 1232 | 82.4 % |
| vertical-misalignment/1.90mm | 704 | 88.2 % |

## fit split
windows 191488, sequences 1088
window accuracy 69.36 % (95 % CI 67.21 to 71.59), balanced 71.58 %, sequence majority 74.26 % (CI 71.5 to 77.1)

| true \ pred | HEALTHY | BEARING_WEAR | IMBALANCE | MISALIGNMENT | recall |
|---|---|---|---|---|---|
| HEALTHY | 3314 | 45 | 43 | 1350 | 69.7 % |
| BEARING_WEAR | 9857 | 68007 | 14086 | 13122 | 64.7 % |
| IMBALANCE | 3768 | 1401 | 25905 | 1662 | 79.1 % |
| MISALIGNMENT | 11771 | 718 | 852 | 35587 | 72.7 % |

| condition | windows | accuracy |
|---|---|---|
| horizontal-misalignment/0.5mm | 5280 | 48.3 % |
| horizontal-misalignment/1.0mm | 3872 | 55.6 % |
| horizontal-misalignment/1.5mm | 4400 | 72.2 % |
| horizontal-misalignment/2.0mm | 4224 | 79.5 % |
| imbalance/10g | 4224 | 73.7 % |
| imbalance/15g | 4928 | 82.3 % |
| imbalance/20g | 4928 | 84.6 % |
| imbalance/25g | 5104 | 88.5 % |
| imbalance/30g | 4400 | 86.3 % |
| imbalance/35g | 4048 | 91.5 % |
| imbalance/6g | 5104 | 50.0 % |
| normal | 4752 | 69.7 % |
| overhang/ball_fault/0g | 4400 | 99.7 % |
| overhang/ball_fault/20g | 2288 | 99.9 % |
| overhang/ball_fault/35g | 1408 | 99.6 % |
| overhang/ball_fault/6g | 4224 | 100.0 % |
| overhang/cage_fault/0g | 5104 | 5.0 % |
| overhang/cage_fault/20g | 4048 | 54.7 % |
| overhang/cage_fault/35g | 4576 | 52.8 % |
| overhang/cage_fault/6g | 4576 | 39.5 % |
| overhang/outer_race/0g | 4928 | 37.0 % |
| overhang/outer_race/20g | 5280 | 54.9 % |
| overhang/outer_race/35g | 4048 | 67.0 % |
| overhang/outer_race/6g | 5104 | 38.4 % |
| underhang/ball_fault/0g | 5632 | 98.7 % |
| underhang/ball_fault/20g | 4400 | 98.2 % |
| underhang/ball_fault/35g | 3696 | 97.4 % |
| underhang/ball_fault/6g | 4224 | 96.4 % |
| underhang/cage_fault/0g | 4400 | 21.1 % |
| underhang/cage_fault/20g | 4752 | 56.6 % |
| underhang/cage_fault/35g | 4224 | 48.6 % |
| underhang/cage_fault/6g | 5632 | 38.2 % |
| underhang/outer_race/0g | 5104 | 88.3 % |
| underhang/outer_race/20g | 4400 | 74.1 % |
| underhang/outer_race/35g | 3696 | 68.6 % |
| underhang/outer_race/6g | 4928 | 80.1 % |
| vertical-misalignment/0.51mm | 5456 | 61.7 % |
| vertical-misalignment/0.63mm | 5280 | 63.4 % |
| vertical-misalignment/1.27mm | 5104 | 91.6 % |
| vertical-misalignment/1.40mm | 4928 | 80.4 % |
| vertical-misalignment/1.78mm | 4928 | 85.2 % |
| vertical-misalignment/1.90mm | 5456 | 87.9 % |
