# cwru_d70_g512: 8-16-4 INT8 on the corrected arithmetic, seed 1

## test split (host model, what the board must reproduce)
windows 12764, sequences 10
window accuracy 80.54 % (95 % CI 72.32 to 88.07), balanced 83.69 %, sequence majority 100.00 % (CI 100.0 to 100.0)

| true \ pred | NORMAL | INNER_RACE | BALL | OUTER_RACE | recall |
|---|---|---|---|---|---|
| NORMAL | 1262 | 1 | 7 | 0 | 99.4 % |
| INNER_RACE | 1 | 3037 | 547 | 250 | 79.2 % |
| BALL | 186 | 418 | 3178 | 43 | 83.1 % |
| OUTER_RACE | 36 | 584 | 411 | 2803 | 73.1 % |

| condition | windows | accuracy |
|---|---|---|
| B007_3hp | 1271 | 93.4 % |
| B014_3hp | 1277 | 74.5 % |
| B021_3hp | 1277 | 81.4 % |
| IR007_3hp | 1286 | 87.7 % |
| IR014_3hp | 1273 | 56.8 % |
| IR021_3hp | 1276 | 92.9 % |
| OR007_3hp | 1282 | 83.5 % |
| OR014_3hp | 1276 | 73.1 % |
| OR021_3hp | 1276 | 62.6 % |
| normal_3hp | 1270 | 99.4 % |

## val split
windows 8904, sequences 7
window accuracy 88.87 % (95 % CI 83.18 to 94.39), balanced 90.14 %, sequence majority 100.00 % (CI 100.0 to 100.0)

| true \ pred | NORMAL | INNER_RACE | BALL | OUTER_RACE | recall |
|---|---|---|---|---|---|
| NORMAL | 1255 | 1 | 12 | 0 | 99.0 % |
| INNER_RACE | 0 | 2247 | 70 | 231 | 88.2 % |
| BALL | 135 | 90 | 2189 | 130 | 86.0 % |
| OUTER_RACE | 3 | 186 | 133 | 2222 | 87.3 % |

| condition | windows | accuracy |
|---|---|---|
| B007_1hp | 1270 | 96.7 % |
| B014_0hp | 1274 | 75.4 % |
| IR007_2hp | 1277 | 94.0 % |
| IR021_1hp | 1271 | 82.4 % |
| OR007_2hp | 1270 | 85.6 % |
| OR014_2hp | 1274 | 89.1 % |
| normal_2hp | 1268 | 99.0 % |

## fit split
windows 28693, sequences 23
window accuracy 80.03 % (95 % CI 71.81 to 86.88), balanced 83.79 %, sequence majority 91.30 % (CI 78.3 to 100.0)

| true \ pred | NORMAL | INNER_RACE | BALL | OUTER_RACE | recall |
|---|---|---|---|---|---|
| NORMAL | 1886 | 0 | 15 | 1 | 99.2 % |
| INNER_RACE | 15 | 6082 | 1736 | 1084 | 68.2 % |
| BALL | 156 | 300 | 8014 | 462 | 89.7 % |
| OUTER_RACE | 11 | 1158 | 793 | 6980 | 78.1 % |

| condition | windows | accuracy |
|---|---|---|
| B007_0hp | 1282 | 97.4 % |
| B007_2hp | 1271 | 96.3 % |
| B014_1hp | 1277 | 86.3 % |
| B014_2hp | 1276 | 85.3 % |
| B021_0hp | 1276 | 90.6 % |
| B021_1hp | 1273 | 78.9 % |
| B021_2hp | 1277 | 93.1 % |
| IR007_0hp | 1268 | 94.1 % |
| IR007_1hp | 1276 | 94.5 % |
| IR014_0hp | 1274 | 51.5 % |
| IR014_1hp | 1274 | 30.9 % |
| IR014_2hp | 1274 | 37.3 % |
| IR021_0hp | 1277 | 81.0 % |
| IR021_2hp | 1274 | 88.2 % |
| OR007_0hp | 1276 | 87.5 % |
| OR007_1hp | 1280 | 81.4 % |
| OR014_0hp | 1274 | 63.5 % |
| OR014_1hp | 1277 | 94.4 % |
| OR021_0hp | 1280 | 73.9 % |
| OR021_1hp | 1276 | 76.2 % |
| OR021_2hp | 1279 | 69.5 % |
| normal_0hp | 637 | 98.9 % |
| normal_1hp | 1265 | 99.3 % |
