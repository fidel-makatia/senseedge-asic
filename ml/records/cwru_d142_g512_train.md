# cwru_d142_g512: 8-16-4 INT8 on the corrected arithmetic, seed 4

## test split (host model, what the board must reproduce)
windows 6324, sequences 10
window accuracy 65.07 % (95 % CI 46.98 to 80.70), balanced 60.92 %, sequence majority 80.00 % (CI 50.0 to 100.0)

| true \ pred | NORMAL | INNER_RACE | BALL | OUTER_RACE | recall |
|---|---|---|---|---|---|
| NORMAL | 253 | 43 | 311 | 22 | 40.2 % |
| INNER_RACE | 0 | 1061 | 267 | 572 | 55.8 % |
| BALL | 0 | 90 | 1406 | 400 | 74.2 % |
| OUTER_RACE | 8 | 229 | 267 | 1395 | 73.5 % |

| condition | windows | accuracy |
|---|---|---|
| B007_3hp | 630 | 91.4 % |
| B014_3hp | 633 | 79.6 % |
| B021_3hp | 633 | 51.5 % |
| IR007_3hp | 637 | 91.5 % |
| IR014_3hp | 631 | 0.0 % |
| IR021_3hp | 632 | 75.6 % |
| OR007_3hp | 635 | 93.7 % |
| OR014_3hp | 632 | 58.2 % |
| OR021_3hp | 632 | 68.4 % |
| normal_3hp | 629 | 40.2 % |

## val split
windows 4411, sequences 7
window accuracy 79.26 % (95 % CI 70.66 to 88.37), balanced 79.19 %, sequence majority 100.00 % (CI 100.0 to 100.0)

| true \ pred | NORMAL | INNER_RACE | BALL | OUTER_RACE | recall |
|---|---|---|---|---|---|
| NORMAL | 495 | 16 | 100 | 17 | 78.8 % |
| INNER_RACE | 0 | 1194 | 31 | 38 | 94.5 % |
| BALL | 127 | 113 | 856 | 164 | 67.9 % |
| OUTER_RACE | 150 | 61 | 98 | 951 | 75.5 % |

| condition | windows | accuracy |
|---|---|---|
| B007_1hp | 629 | 64.5 % |
| B014_0hp | 631 | 71.3 % |
| IR007_2hp | 633 | 98.3 % |
| IR021_1hp | 630 | 90.8 % |
| OR007_2hp | 629 | 86.3 % |
| OR014_2hp | 631 | 64.7 % |
| normal_2hp | 628 | 78.8 % |

## fit split
windows 14217, sequences 23
window accuracy 68.19 % (95 % CI 55.76 to 79.39), balanced 70.69 %, sequence majority 78.26 % (CI 56.5 to 91.4)

| true \ pred | NORMAL | INNER_RACE | BALL | OUTER_RACE | recall |
|---|---|---|---|---|---|
| NORMAL | 762 | 17 | 139 | 24 | 80.9 % |
| INNER_RACE | 218 | 2732 | 1155 | 312 | 61.9 % |
| BALL | 407 | 236 | 3619 | 164 | 81.8 % |
| OUTER_RACE | 330 | 666 | 854 | 2582 | 58.3 % |

| condition | windows | accuracy |
|---|---|---|
| B007_0hp | 635 | 92.1 % |
| B007_2hp | 630 | 89.0 % |
| B014_1hp | 633 | 70.0 % |
| B014_2hp | 632 | 87.8 % |
| B021_0hp | 632 | 94.3 % |
| B021_1hp | 631 | 49.6 % |
| B021_2hp | 633 | 89.4 % |
| IR007_0hp | 628 | 97.8 % |
| IR007_1hp | 632 | 99.5 % |
| IR014_0hp | 631 | 32.5 % |
| IR014_1hp | 631 | 18.7 % |
| IR014_2hp | 631 | 11.9 % |
| IR021_0hp | 633 | 86.1 % |
| IR021_2hp | 631 | 86.5 % |
| OR007_0hp | 632 | 76.4 % |
| OR007_1hp | 635 | 75.6 % |
| OR014_0hp | 631 | 12.2 % |
| OR014_1hp | 633 | 26.9 % |
| OR021_0hp | 635 | 67.4 % |
| OR021_1hp | 632 | 74.8 % |
| OR021_2hp | 634 | 74.3 % |
| normal_0hp | 315 | 53.7 % |
| normal_1hp | 627 | 94.6 % |
