# cwru: 8-16-4 INT8 on the corrected arithmetic, seed 5

## test split (host model, what the board must reproduce)
windows 3608, sequences 10
window accuracy 70.26 % (95 % CI 52.06 to 83.91), balanced 74.07 %, sequence majority 90.00 % (CI 70.0 to 100.0)

| true \ pred | NORMAL | INNER_RACE | BALL | OUTER_RACE | recall |
|---|---|---|---|---|---|
| NORMAL | 334 | 3 | 22 | 0 | 93.0 % |
| INNER_RACE | 0 | 620 | 289 | 175 | 57.2 % |
| BALL | 133 | 13 | 720 | 215 | 66.6 % |
| OUTER_RACE | 43 | 88 | 92 | 861 | 79.4 % |

| condition | windows | accuracy |
|---|---|---|
| B007_3hp | 359 | 83.0 % |
| B014_3hp | 361 | 62.9 % |
| B021_3hp | 361 | 54.0 % |
| IR007_3hp | 363 | 87.1 % |
| IR014_3hp | 360 | 1.7 % |
| IR021_3hp | 361 | 82.5 % |
| OR007_3hp | 362 | 85.6 % |
| OR014_3hp | 361 | 63.7 % |
| OR021_3hp | 361 | 88.9 % |
| normal_3hp | 359 | 93.0 % |

## val split
windows 2516, sequences 7
window accuracy 71.86 % (95 % CI 49.78 to 91.75), balanced 74.86 %, sequence majority 71.43 % (CI 42.9 to 100.0)

| true \ pred | NORMAL | INNER_RACE | BALL | OUTER_RACE | recall |
|---|---|---|---|---|---|
| NORMAL | 343 | 2 | 13 | 0 | 95.8 % |
| INNER_RACE | 0 | 675 | 0 | 45 | 93.8 % |
| BALL | 131 | 51 | 354 | 183 | 49.2 % |
| OUTER_RACE | 105 | 24 | 154 | 436 | 60.6 % |

| condition | windows | accuracy |
|---|---|---|
| B007_1hp | 359 | 68.2 % |
| B014_0hp | 360 | 30.3 % |
| IR007_2hp | 361 | 99.7 % |
| IR021_1hp | 359 | 87.7 % |
| OR007_2hp | 359 | 91.1 % |
| OR014_2hp | 360 | 30.3 % |
| normal_2hp | 358 | 95.8 % |

## fit split
windows 8110, sequences 23
window accuracy 70.42 % (95 % CI 58.36 to 80.69), balanced 73.37 %, sequence majority 91.30 % (CI 78.3 to 100.0)

| true \ pred | NORMAL | INNER_RACE | BALL | OUTER_RACE | recall |
|---|---|---|---|---|---|
| NORMAL | 459 | 4 | 59 | 15 | 85.5 % |
| INNER_RACE | 119 | 1368 | 607 | 426 | 54.3 % |
| BALL | 224 | 81 | 2069 | 151 | 81.9 % |
| OUTER_RACE | 118 | 258 | 337 | 1815 | 71.8 % |

| condition | windows | accuracy |
|---|---|---|
| B007_0hp | 362 | 65.5 % |
| B007_2hp | 359 | 94.4 % |
| B014_1hp | 361 | 94.2 % |
| B014_2hp | 361 | 77.3 % |
| B021_0hp | 361 | 91.7 % |
| B021_1hp | 360 | 63.6 % |
| B021_2hp | 361 | 87.0 % |
| IR007_0hp | 358 | 86.3 % |
| IR007_1hp | 361 | 98.6 % |
| IR014_0hp | 360 | 37.8 % |
| IR014_1hp | 360 | 8.3 % |
| IR014_2hp | 360 | 4.4 % |
| IR021_0hp | 361 | 56.8 % |
| IR021_2hp | 360 | 87.8 % |
| OR007_0hp | 361 | 88.4 % |
| OR007_1hp | 362 | 81.2 % |
| OR014_0hp | 360 | 46.9 % |
| OR014_1hp | 361 | 33.5 % |
| OR021_0hp | 362 | 86.5 % |
| OR021_1hp | 361 | 83.9 % |
| OR021_2hp | 361 | 82.0 % |
| normal_0hp | 179 | 77.1 % |
| normal_1hp | 358 | 89.7 % |
