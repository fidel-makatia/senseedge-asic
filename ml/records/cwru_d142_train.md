# cwru_d142: 8-16-4 INT8 on the corrected arithmetic, seed 7

## test split (host model, what the board must reproduce)
windows 6324, sequences 10
window accuracy 53.59 % (95 % CI 33.94 to 73.24), balanced 51.37 %, sequence majority 60.00 % (CI 30.0 to 90.0)

| true \ pred | NORMAL | INNER_RACE | BALL | OUTER_RACE | recall |
|---|---|---|---|---|---|
| NORMAL | 254 | 65 | 280 | 30 | 40.4 % |
| INNER_RACE | 0 | 991 | 31 | 878 | 52.2 % |
| BALL | 0 | 81 | 701 | 1114 | 37.0 % |
| OUTER_RACE | 16 | 180 | 260 | 1443 | 76.0 % |

| condition | windows | accuracy |
|---|---|---|
| B007_3hp | 630 | 71.0 % |
| B014_3hp | 633 | 31.3 % |
| B021_3hp | 633 | 8.8 % |
| IR007_3hp | 637 | 94.2 % |
| IR014_3hp | 631 | 0.2 % |
| IR021_3hp | 632 | 61.7 % |
| OR007_3hp | 635 | 96.2 % |
| OR014_3hp | 632 | 52.8 % |
| OR021_3hp | 632 | 78.8 % |
| normal_3hp | 629 | 40.4 % |

## val split
windows 4411, sequences 7
window accuracy 78.67 % (95 % CI 64.08 to 90.74), balanced 78.62 %, sequence majority 85.71 % (CI 57.1 to 100.0)

| true \ pred | NORMAL | INNER_RACE | BALL | OUTER_RACE | recall |
|---|---|---|---|---|---|
| NORMAL | 492 | 14 | 99 | 23 | 78.3 % |
| INNER_RACE | 0 | 1179 | 5 | 79 | 93.3 % |
| BALL | 87 | 134 | 747 | 292 | 59.3 % |
| OUTER_RACE | 143 | 36 | 29 | 1052 | 83.5 % |

| condition | windows | accuracy |
|---|---|---|
| B007_1hp | 629 | 35.9 % |
| B014_0hp | 631 | 82.6 % |
| IR007_2hp | 633 | 99.8 % |
| IR021_1hp | 630 | 86.8 % |
| OR007_2hp | 629 | 92.1 % |
| OR014_2hp | 631 | 75.0 % |
| normal_2hp | 628 | 78.3 % |

## fit split
windows 14217, sequences 23
window accuracy 68.80 % (95 % CI 58.21 to 78.09), balanced 71.89 %, sequence majority 82.61 % (CI 65.2 to 95.7)

| true \ pred | NORMAL | INNER_RACE | BALL | OUTER_RACE | recall |
|---|---|---|---|---|---|
| NORMAL | 796 | 12 | 72 | 62 | 84.5 % |
| INNER_RACE | 258 | 2699 | 865 | 595 | 61.1 % |
| BALL | 466 | 365 | 3260 | 335 | 73.7 % |
| OUTER_RACE | 330 | 549 | 526 | 3027 | 68.3 % |

| condition | windows | accuracy |
|---|---|---|
| B007_0hp | 635 | 84.9 % |
| B007_2hp | 630 | 78.4 % |
| B014_1hp | 633 | 75.4 % |
| B014_2hp | 632 | 87.7 % |
| B021_0hp | 632 | 88.1 % |
| B021_1hp | 631 | 27.3 % |
| B021_2hp | 633 | 73.8 % |
| IR007_0hp | 628 | 98.4 % |
| IR007_1hp | 632 | 99.8 % |
| IR014_0hp | 631 | 43.7 % |
| IR014_1hp | 631 | 22.7 % |
| IR014_2hp | 631 | 21.9 % |
| IR021_0hp | 633 | 68.1 % |
| IR021_2hp | 631 | 73.2 % |
| OR007_0hp | 632 | 85.6 % |
| OR007_1hp | 635 | 79.2 % |
| OR014_0hp | 631 | 17.0 % |
| OR014_1hp | 633 | 55.5 % |
| OR021_0hp | 635 | 76.5 % |
| OR021_1hp | 632 | 82.6 % |
| OR021_2hp | 634 | 81.5 % |
| normal_0hp | 315 | 69.5 % |
| normal_1hp | 627 | 92.0 % |
