# cwru_d249_g512: 8-16-4 INT8 on the corrected arithmetic, seed 6

## test split (host model, what the board must reproduce)
windows 3608, sequences 10
window accuracy 75.86 % (95 % CI 57.52 to 88.37), balanced 77.71 %, sequence majority 90.00 % (CI 70.0 to 100.0)

| true \ pred | NORMAL | INNER_RACE | BALL | OUTER_RACE | recall |
|---|---|---|---|---|---|
| NORMAL | 312 | 14 | 29 | 4 | 86.9 % |
| INNER_RACE | 0 | 582 | 309 | 193 | 53.7 % |
| BALL | 81 | 7 | 869 | 124 | 80.4 % |
| OUTER_RACE | 5 | 44 | 61 | 974 | 89.9 % |

| condition | windows | accuracy |
|---|---|---|
| B007_3hp | 359 | 82.7 % |
| B014_3hp | 361 | 75.9 % |
| B021_3hp | 361 | 82.5 % |
| IR007_3hp | 363 | 94.8 % |
| IR014_3hp | 360 | 1.7 % |
| IR021_3hp | 361 | 64.3 % |
| OR007_3hp | 362 | 94.5 % |
| OR014_3hp | 361 | 83.1 % |
| OR021_3hp | 361 | 92.0 % |
| normal_3hp | 359 | 86.9 % |

## val split
windows 2516, sequences 7
window accuracy 69.91 % (95 % CI 53.82 to 86.63), balanced 72.21 %, sequence majority 100.00 % (CI 100.0 to 100.0)

| true \ pred | NORMAL | INNER_RACE | BALL | OUTER_RACE | recall |
|---|---|---|---|---|---|
| NORMAL | 316 | 4 | 37 | 1 | 88.3 % |
| INNER_RACE | 0 | 559 | 5 | 156 | 77.6 % |
| BALL | 89 | 79 | 399 | 152 | 55.5 % |
| OUTER_RACE | 20 | 8 | 206 | 485 | 67.5 % |

| condition | windows | accuracy |
|---|---|---|
| B007_1hp | 359 | 77.4 % |
| B014_0hp | 360 | 33.6 % |
| IR007_2hp | 361 | 99.4 % |
| IR021_1hp | 359 | 55.7 % |
| OR007_2hp | 359 | 87.7 % |
| OR014_2hp | 360 | 47.2 % |
| normal_2hp | 358 | 88.3 % |

## fit split
windows 8110, sequences 23
window accuracy 70.09 % (95 % CI 56.52 to 81.42), balanced 70.25 %, sequence majority 78.26 % (CI 56.5 to 91.4)

| true \ pred | NORMAL | INNER_RACE | BALL | OUTER_RACE | recall |
|---|---|---|---|---|---|
| NORMAL | 381 | 6 | 96 | 54 | 70.9 % |
| INNER_RACE | 54 | 1364 | 724 | 378 | 54.1 % |
| BALL | 115 | 72 | 2204 | 134 | 87.3 % |
| OUTER_RACE | 43 | 95 | 655 | 1735 | 68.6 % |

| condition | windows | accuracy |
|---|---|---|
| B007_0hp | 362 | 78.2 % |
| B007_2hp | 359 | 96.1 % |
| B014_1hp | 361 | 96.4 % |
| B014_2hp | 361 | 80.6 % |
| B021_0hp | 361 | 85.3 % |
| B021_1hp | 360 | 80.8 % |
| B021_2hp | 361 | 93.6 % |
| IR007_0hp | 358 | 87.4 % |
| IR007_1hp | 361 | 99.2 % |
| IR014_0hp | 360 | 40.8 % |
| IR014_1hp | 360 | 6.1 % |
| IR014_2hp | 360 | 3.9 % |
| IR021_0hp | 361 | 66.8 % |
| IR021_2hp | 360 | 74.7 % |
| OR007_0hp | 361 | 80.9 % |
| OR007_1hp | 362 | 60.2 % |
| OR014_0hp | 360 | 20.0 % |
| OR014_1hp | 361 | 31.0 % |
| OR021_0hp | 362 | 97.0 % |
| OR021_1hp | 361 | 96.7 % |
| OR021_2hp | 361 | 94.5 % |
| normal_0hp | 179 | 46.9 % |
| normal_1hp | 358 | 83.0 % |
