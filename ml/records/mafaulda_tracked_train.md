# mafaulda_tracked: 8-16-4 INT8 on the corrected arithmetic, seed 9

## test split (host model, what the board must reproduce)
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

## val split
windows 23685, sequences 273
window accuracy 66.13 % (95 % CI 61.22 to 70.40), balanced 72.35 %, sequence majority 65.57 % (CI 59.7 to 70.7)

| true \ pred | HEALTHY | BEARING_WEAR | IMBALANCE | MISALIGNMENT | recall |
|---|---|---|---|---|---|
| HEALTHY | 406 | 14 | 1 | 101 | 77.8 % |
| BEARING_WEAR | 1151 | 7273 | 2603 | 1645 | 57.4 % |
| IMBALANCE | 208 | 161 | 3385 | 379 | 81.9 % |
| MISALIGNMENT | 1346 | 336 | 77 | 4599 | 72.3 % |

| condition | windows | accuracy |
|---|---|---|
| horizontal-misalignment/0.5mm | 409 | 51.1 % |
| horizontal-misalignment/1.0mm | 1286 | 45.3 % |
| horizontal-misalignment/1.5mm | 946 | 92.1 % |
| horizontal-misalignment/2.0mm | 973 | 86.2 % |
| imbalance/10g | 1031 | 84.9 % |
| imbalance/15g | 464 | 78.0 % |
| imbalance/20g | 522 | 99.8 % |
| imbalance/25g | 394 | 91.4 % |
| imbalance/30g | 718 | 87.3 % |
| imbalance/35g | 629 | 80.9 % |
| imbalance/6g | 375 | 34.9 % |
| normal | 522 | 77.8 % |
| overhang/ball_fault/0g | 798 | 99.5 % |
| overhang/ball_fault/20g | 218 | 95.9 % |
| overhang/ball_fault/35g | 297 | 83.2 % |
| overhang/ball_fault/6g | 462 | 93.3 % |
| overhang/cage_fault/0g | 458 | 15.9 % |
| overhang/cage_fault/20g | 976 | 66.2 % |
| overhang/cage_fault/35g | 205 | 48.8 % |
| overhang/cage_fault/6g | 731 | 43.6 % |
| overhang/outer_race/0g | 572 | 36.7 % |
| overhang/outer_race/20g | 405 | 84.0 % |
| overhang/outer_race/35g | 410 | 44.6 % |
| overhang/outer_race/6g | 504 | 67.9 % |
| underhang/ball_fault/0g | 278 | 100.0 % |
| underhang/ball_fault/20g | 695 | 93.7 % |
| underhang/ball_fault/35g | 478 | 82.0 % |
| underhang/ball_fault/6g | 970 | 95.1 % |
| underhang/cage_fault/0g | 957 | 19.5 % |
| underhang/cage_fault/20g | 688 | 32.0 % |
| underhang/cage_fault/35g | 373 | 3.5 % |
| underhang/cage_fault/6g | 147 | 80.3 % |
| underhang/outer_race/0g | 456 | 1.8 % |
| underhang/outer_race/20g | 707 | 52.3 % |
| underhang/outer_race/35g | 409 | 14.4 % |
| underhang/outer_race/6g | 478 | 33.7 % |
| vertical-misalignment/0.51mm | 439 | 61.5 % |
| vertical-misalignment/0.63mm | 442 | 68.6 % |
| vertical-misalignment/1.27mm | 466 | 80.3 % |
| vertical-misalignment/1.40mm | 492 | 78.3 % |
| vertical-misalignment/1.78mm | 440 | 83.4 % |
| vertical-misalignment/1.90mm | 465 | 85.8 % |

## fit split
windows 95620, sequences 1088
window accuracy 67.78 % (95 % CI 65.58 to 69.88), balanced 74.49 %, sequence majority 67.92 % (CI 65.2 to 70.7)

| true \ pred | HEALTHY | BEARING_WEAR | IMBALANCE | MISALIGNMENT | recall |
|---|---|---|---|---|---|
| HEALTHY | 1902 | 33 | 2 | 449 | 79.7 % |
| BEARING_WEAR | 4258 | 30362 | 10331 | 6894 | 58.6 % |
| IMBALANCE | 699 | 553 | 14113 | 1099 | 85.7 % |
| MISALIGNMENT | 5143 | 1261 | 91 | 18430 | 73.9 % |

| condition | windows | accuracy |
|---|---|---|
| horizontal-misalignment/0.5mm | 2753 | 65.3 % |
| horizontal-misalignment/1.0mm | 1721 | 55.5 % |
| horizontal-misalignment/1.5mm | 2165 | 92.6 % |
| horizontal-misalignment/2.0mm | 2219 | 93.7 % |
| imbalance/10g | 2035 | 79.6 % |
| imbalance/15g | 2784 | 92.2 % |
| imbalance/20g | 2634 | 94.0 % |
| imbalance/25g | 2275 | 92.6 % |
| imbalance/30g | 2260 | 94.0 % |
| imbalance/35g | 1868 | 97.2 % |
| imbalance/6g | 2608 | 53.9 % |
| normal | 2386 | 79.7 % |
| overhang/ball_fault/0g | 2226 | 97.9 % |
| overhang/ball_fault/20g | 746 | 96.5 % |
| overhang/ball_fault/35g | 487 | 96.3 % |
| overhang/ball_fault/6g | 2016 | 96.6 % |
| overhang/cage_fault/0g | 2696 | 37.6 % |
| overhang/cage_fault/20g | 2057 | 75.6 % |
| overhang/cage_fault/35g | 2056 | 66.3 % |
| overhang/cage_fault/6g | 2274 | 60.6 % |
| overhang/outer_race/0g | 2438 | 40.2 % |
| overhang/outer_race/20g | 2590 | 64.5 % |
| overhang/outer_race/35g | 2017 | 68.4 % |
| overhang/outer_race/6g | 2424 | 55.2 % |
| underhang/ball_fault/0g | 3019 | 93.6 % |
| underhang/ball_fault/20g | 2410 | 96.6 % |
| underhang/ball_fault/35g | 1597 | 92.2 % |
| underhang/ball_fault/6g | 2105 | 96.9 % |
| underhang/cage_fault/0g | 2150 | 17.0 % |
| underhang/cage_fault/20g | 2616 | 38.1 % |
| underhang/cage_fault/35g | 1926 | 19.7 % |
| underhang/cage_fault/6g | 2856 | 52.9 % |
| underhang/outer_race/0g | 2722 | 7.3 % |
| underhang/outer_race/20g | 2274 | 23.7 % |
| underhang/outer_race/35g | 1632 | 10.1 % |
| underhang/outer_race/6g | 2511 | 61.5 % |
| vertical-misalignment/0.51mm | 2680 | 56.5 % |
| vertical-misalignment/0.63mm | 2900 | 67.5 % |
| vertical-misalignment/1.27mm | 2750 | 74.1 % |
| vertical-misalignment/1.40mm | 2581 | 73.1 % |
| vertical-misalignment/1.78mm | 2505 | 83.1 % |
| vertical-misalignment/1.90mm | 2651 | 79.7 % |
