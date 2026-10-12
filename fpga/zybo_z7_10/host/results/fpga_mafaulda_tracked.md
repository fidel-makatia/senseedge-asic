windows 53066, sequences 590
window accuracy 67.56 % (95 % CI 64.21 to 70.61), balanced 73.51 %, sequence majority 67.80 % (CI 64.1 to 71.4)

| true \ pred | HEALTHY | BEARING_WEAR | IMBALANCE | MISALIGNMENT | recall |
|---|---|---|---|---|---|
| HEALTHY | 1139 | 36 | 0 | 295 | 77.5 % |
| BEARING_WEAR | 2217 | 17339 | 6102 | 3438 | 59.6 % |
| IMBALANCE | 315 | 515 | 7677 | 526 | 85.0 % |
| MISALIGNMENT | 3127 | 602 | 43 | 9695 | 72.0 % |

| condition | windows | accuracy |
|---|---|---|
| horizontal-misalignment/0.5mm | 1326 | 57.2 % |
| horizontal-misalignment/1.0mm | 1328 | 51.1 % |
| horizontal-misalignment/1.5mm | 1335 | 95.7 % |
| horizontal-misalignment/2.0mm | 1151 | 87.7 % |
| imbalance/10g | 1325 | 82.1 % |
| imbalance/15g | 1162 | 94.4 % |
| imbalance/20g | 1318 | 88.7 % |
| imbalance/25g | 1370 | 92.3 % |
| imbalance/30g | 1086 | 87.8 % |
| imbalance/35g | 1274 | 99.9 % |
| imbalance/6g | 1498 | 55.6 % |
| normal | 1470 | 77.5 % |
| overhang/ball_fault/0g | 1447 | 96.4 % |
| overhang/ball_fault/20g | 499 | 98.0 % |
| overhang/ball_fault/35g | 299 | 91.6 % |
| overhang/ball_fault/6g | 1110 | 98.6 % |
| overhang/cage_fault/0g | 1298 | 53.3 % |
| overhang/cage_fault/20g | 1432 | 82.4 % |
| overhang/cage_fault/35g | 1066 | 69.2 % |
| overhang/cage_fault/6g | 1455 | 68.7 % |
| overhang/outer_race/0g | 1453 | 42.2 % |
| overhang/outer_race/20g | 1469 | 67.4 % |
| overhang/outer_race/35g | 900 | 53.6 % |
| overhang/outer_race/6g | 1535 | 65.6 % |
| underhang/ball_fault/0g | 1211 | 97.0 % |
| underhang/ball_fault/20g | 1363 | 97.4 % |
| underhang/ball_fault/35g | 868 | 93.5 % |
| underhang/ball_fault/6g | 1380 | 97.2 % |
| underhang/cage_fault/0g | 1372 | 12.0 % |
| underhang/cage_fault/20g | 1173 | 27.2 % |
| underhang/cage_fault/35g | 1248 | 16.0 % |
| underhang/cage_fault/6g | 1388 | 51.8 % |
| underhang/outer_race/0g | 1296 | 7.7 % |
| underhang/outer_race/20g | 1478 | 29.8 % |
| underhang/outer_race/35g | 878 | 17.2 % |
| underhang/outer_race/6g | 1478 | 42.8 % |
| vertical-misalignment/0.51mm | 1519 | 53.0 % |
| vertical-misalignment/0.63mm | 1159 | 60.0 % |
| vertical-misalignment/1.27mm | 1282 | 80.4 % |
| vertical-misalignment/1.40mm | 1416 | 78.7 % |
| vertical-misalignment/1.78mm | 1560 | 80.4 % |
| vertical-misalignment/1.90mm | 1391 | 77.1 % |
