# cwru_d70: 8-16-4 INT8 on the corrected arithmetic, seed 7

## test split (host model, what the board must reproduce)
windows 12764, sequences 10
window accuracy 80.28 % (95 % CI 74.21 to 86.57), balanced 83.33 %, sequence majority 100.00 % (CI 100.0 to 100.0)

| true \ pred | NORMAL | INNER_RACE | BALL | OUTER_RACE | recall |
|---|---|---|---|---|---|
| NORMAL | 1251 | 0 | 19 | 0 | 98.5 % |
| INNER_RACE | 0 | 3109 | 336 | 390 | 81.1 % |
| BALL | 20 | 570 | 3103 | 132 | 81.1 % |
| OUTER_RACE | 2 | 556 | 492 | 2784 | 72.6 % |

| condition | windows | accuracy |
|---|---|---|
| B007_3hp | 1271 | 90.6 % |
| B014_3hp | 1277 | 81.5 % |
| B021_3hp | 1277 | 71.3 % |
| IR007_3hp | 1286 | 84.3 % |
| IR014_3hp | 1273 | 67.9 % |
| IR021_3hp | 1276 | 91.0 % |
| OR007_3hp | 1282 | 82.1 % |
| OR014_3hp | 1276 | 68.8 % |
| OR021_3hp | 1276 | 66.8 % |
| normal_3hp | 1270 | 98.5 % |

## val split
windows 8904, sequences 7
window accuracy 86.68 % (95 % CI 81.70 to 92.31), balanced 88.21 %, sequence majority 100.00 % (CI 100.0 to 100.0)

| true \ pred | NORMAL | INNER_RACE | BALL | OUTER_RACE | recall |
|---|---|---|---|---|---|
| NORMAL | 1254 | 0 | 14 | 0 | 98.9 % |
| INNER_RACE | 0 | 2094 | 43 | 411 | 82.2 % |
| BALL | 3 | 180 | 2192 | 169 | 86.2 % |
| OUTER_RACE | 1 | 163 | 202 | 2178 | 85.6 % |

| condition | windows | accuracy |
|---|---|---|
| B007_1hp | 1270 | 89.1 % |
| B014_0hp | 1274 | 83.2 % |
| IR007_2hp | 1277 | 90.4 % |
| IR021_1hp | 1271 | 74.0 % |
| OR007_2hp | 1270 | 88.4 % |
| OR014_2hp | 1274 | 82.8 % |
| normal_2hp | 1268 | 98.9 % |

## fit split
windows 28693, sequences 23
window accuracy 80.29 % (95 % CI 73.66 to 85.97), balanced 84.02 %, sequence majority 100.00 % (CI 100.0 to 100.0)

| true \ pred | NORMAL | INNER_RACE | BALL | OUTER_RACE | recall |
|---|---|---|---|---|---|
| NORMAL | 1888 | 0 | 14 | 0 | 99.3 % |
| INNER_RACE | 1 | 6392 | 1330 | 1194 | 71.7 % |
| BALL | 60 | 526 | 7877 | 469 | 88.2 % |
| OUTER_RACE | 2 | 1095 | 965 | 6880 | 76.9 % |

| condition | windows | accuracy |
|---|---|---|
| B007_0hp | 1282 | 91.7 % |
| B007_2hp | 1271 | 91.2 % |
| B014_1hp | 1277 | 90.8 % |
| B014_2hp | 1276 | 86.6 % |
| B021_0hp | 1276 | 88.6 % |
| B021_1hp | 1273 | 76.4 % |
| B021_2hp | 1277 | 92.0 % |
| IR007_0hp | 1268 | 90.1 % |
| IR007_1hp | 1276 | 94.6 % |
| IR014_0hp | 1274 | 63.7 % |
| IR014_1hp | 1274 | 46.5 % |
| IR014_2hp | 1274 | 48.3 % |
| IR021_0hp | 1277 | 75.8 % |
| IR021_2hp | 1274 | 83.0 % |
| OR007_0hp | 1276 | 90.6 % |
| OR007_1hp | 1280 | 81.7 % |
| OR014_0hp | 1274 | 54.7 % |
| OR014_1hp | 1277 | 90.3 % |
| OR021_0hp | 1280 | 74.5 % |
| OR021_1hp | 1276 | 75.4 % |
| OR021_2hp | 1279 | 71.3 % |
| normal_0hp | 637 | 99.5 % |
| normal_1hp | 1265 | 99.1 % |
