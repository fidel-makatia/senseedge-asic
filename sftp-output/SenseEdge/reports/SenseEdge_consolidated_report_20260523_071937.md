# 🚀 ChipFoundry Tapeout Report

**Consolidated Tapeout and DRC Report**

*Custom Chip Design and Fabrication Platform*

**Generated:** 2026-05-23 07:19:38

![ChipFoundry Logo](https://static-files.umso.co/lib_lnlnuhLgkYnZdkSC/05rk210br2vbseno.png)

## Project Information

| Field | Value |
|-------|-------|
| Project Path | `/mnt/shuttle/ci2605/fidel-omusilibwa/SenseEdge` |
| Project Name | **SenseEdge** |
| User | fidel-omusilibwa |
| Project ID | 2605c0e9 |
| Project Type | digital |
| Slot | 26 |
| Version | 2 |

## Tapeout Status

| Field | Value |
|-------|-------|
| Status | **COMPLETED** |
| Shuttle | ci2605 |
| PDK | sky130A |
| PDK Version | unknown |
| PDK Short Hash | unknown |
| PDK Tag | unknown |
| Caravel Version | CC2509 |
| Caravel Commit Hash | a0b50ad |
| Target Foundry | skywater |
| Run Tapeout Version | 1.9.0 |
| KLayout Version | 0.30.2 |
| Magic Version | 8.3.471 |
| DRC Input File | `gds/caravel_2605c0e9.gds` |
| DRC Top Cell | `caravel_2605c0e9` |
| OAS Output File | `tapeout/outputs/oas/caravel_2605c0e9.oas` |

## GPIO Configuration

This section shows the GPIO configuration for your project based on layout analysis and RTL definitions.

**Note:** GPIO 0-4 have fixed configurations and are not user-configurable. GPIO 5-37 can be configured to either user or management areas.

### GPIO Mode Reference

#### Management Area Modes
- `0403`: Input, no pull-up/down
- `0c01`: Input with pull-down
- `0801`: Input with pull-up
- `1809`: Output
- `1801`: Bidirectional
- `000b`: Analog

#### User Area Modes
- `0402`: Input, no pull-up/down
- `0c00`: Input with pull-down
- `0800`: Input with pull-up
- `1808`: Output
- `1800`: Bidirectional
- `1802`: Output with monitoring
- `000a`: Analog

### GPIO Configuration Details

| GPIO | Layout Config | Layout Mode | RTL Config | RTL Mode | Status |
|------|---------------|-------------|------------|----------|--------|
| GPIO_5 | `1809` | GPIO_MODE_MGMT_STD_OUTPUT | `1809` | GPIO_MODE_MGMT_STD_OUTPUT | ✅ Matches |
| GPIO_6 | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | ✅ Matches |
| GPIO_7 | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | ✅ Matches |
| GPIO_8 | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | ✅ Matches |
| GPIO_9 | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | ✅ Matches |
| GPIO_10 | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | ✅ Matches |
| GPIO_11 | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | ✅ Matches |
| GPIO_12 | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | ✅ Matches |
| GPIO_13 | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | ✅ Matches |
| GPIO_14 | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | ✅ Matches |
| GPIO_15 | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | ✅ Matches |
| GPIO_16 | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | ✅ Matches |
| GPIO_17 | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | ✅ Matches |
| GPIO_18 | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | ✅ Matches |
| GPIO_19 | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | ✅ Matches |
| GPIO_20 | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | ✅ Matches |
| GPIO_21 | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | ✅ Matches |
| GPIO_22 | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | ✅ Matches |
| GPIO_23 | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | ✅ Matches |
| GPIO_24 | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | ✅ Matches |
| GPIO_25 | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | ✅ Matches |
| GPIO_26 | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | ✅ Matches |
| GPIO_27 | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | ✅ Matches |
| GPIO_28 | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | ✅ Matches |
| GPIO_29 | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | ✅ Matches |
| GPIO_30 | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | ✅ Matches |
| GPIO_31 | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | ✅ Matches |
| GPIO_32 | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | ✅ Matches |
| GPIO_33 | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | ✅ Matches |
| GPIO_34 | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | ✅ Matches |
| GPIO_35 | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | ✅ Matches |
| GPIO_36 | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | ✅ Matches |
| GPIO_37 | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | `0403` | GPIO_MODE_MGMT_STD_INPUT_NOPULL | ✅ Matches |

**Legend:**
- **Layout:** GPIO configuration extracted from layout (GPIO defaults blocks)
- **RTL:** GPIO configuration from user_defines.v file
- **Status:** Comparison result between layout and RTL

*This configuration is based on the user_defines.v file from the caravel_user_project template repository.*


#### User Wrapper GDS

| Property | Value |
|----------|-------|
| File | `user_project_wrapper.gds` |
| Size | 230.24 MB |
| Modified | 2026-05-17T21:04:48.741000 |
| Stored Hash | `4b36463fc7d6e5bed100aa530392759871cc8b44daa23f320f4facbfd0f77ff8` |
| Current Hash | `4b36463fc7d6e5bed100aa530392759871cc8b44daa23f320f4facbfd0f77ff8` |
| Hash Valid | ✅ |

#### OASIS File

| Property | Value |
|----------|-------|
| File | `caravel_2605c0e9.oas` |
| Size | 40.09 MB |
| Modified | 2026-05-22T13:56:59.337000 |
| Current Hash | `b4e18258805fca6f1005904e7f8c11dec84acca176550dee698a3832383f488f` |
| Stored Hash | `b4e18258805fca6f1005904e7f8c11dec84acca176550dee698a3832383f488f` |
| Hash Valid | ✅ |

#### OASIS Layout Images

Generated layout images from OASIS files showing the chip design visualization.

**caravel_layout**

![OASIS Layout: caravel_layout](../outputs/images/caravel_layout.png)

| Property | Value |
|----------|-------|
| File | `caravel_layout.png` |
| Size | 0.36 MB |
| Modified | 2026-05-22 13:57:29 |
| Format | PNG |

*Images are generated from OASIS files using GDSFactory for high-quality visualization.*


## DRC Results Summary

| Metric | Value |
|--------|-------|
| Total Checks | **26** |
| Passed | 25 |
| Failed | 1 |
| Total Errors | 1483 |
| Total Execution Time | 204.12 minutes |
| Last Updated | 2026-05-23T07:19:37.085521 |

## Information Checks

These checks provide information about design features:

| Check Name | Status | Execution Time | Last Updated | Report File |
|------------|--------|----------------|--------------|-------------|
| Commercial SRAM (`cs`) | ⚫ NO | 0.39 min | 2026-05-23T07:19:37.085429 | `../outputs/klayout_CF_SRAM_check.xml` |
| RRAM Layers (`rr`) | ⚫ NO | 0.38 min | 2026-05-23T07:19:37.085488 | `../outputs/klayout_rram_layers_check.xml` |
| OpenRAM (`os`) | ⚫ NO | 0.38 min | 2026-05-23T07:19:37.085492 | `../outputs/klayout_open_sram_check.xml` |

## DRC Validation Checks

These checks validate design rules and constraints:

| Check Name | Status | Errors | Execution Time | Last Updated | Report File |
|------------|--------|--------|----------------|--------------|-------------|
| Backend (`be`) | ✅ PASS | 0 | 64.43 min | 2026-05-23T07:19:37.085368 | `../outputs/klayout_beol_check.xml` |
| Frontend (`fe`) | ✅ PASS | 0 | 35.06 min | 2026-05-23T07:19:37.085347 | `../outputs/klayout_feol_check.xml` |
| Metal Global Density (`md`) | ✅ PASS | 0 | 24.78 min | 2026-05-23T07:19:37.085386 | `../outputs/klayout_mgld_check.xml` |
| Metal 1 Drawn vs Fill (`m1`) | ✅ PASS | 0 | 16.35 min | 2026-05-23T07:19:37.085394 | `../outputs/klayout_metal1_drawn_vs_fill_check.xml` |
| Unknown Layers (`ul`) | ✅ PASS | 0 | 14.38 min | 2026-05-23T07:19:37.085470 | `../outputs/klayout_unknown_layers_check.xml` |
| Off-Grid (`og`) | ✅ PASS | 0 | 12.34 min | 2026-05-23T07:19:37.085417 | `../outputs/klayout_grid_check.xml` |
| Metal 2 Drawn vs Fill (`m2`) | ✅ PASS | 0 | 10.07 min | 2026-05-23T07:19:37.085398 | `../outputs/klayout_metal2_drawn_vs_fill_check.xml` |
| Metal 4 Drawn vs Fill (`m4`) | ✅ PASS | 0 | 6.17 min | 2026-05-23T07:19:37.085408 | `../outputs/klayout_metal4_drawn_vs_fill_check.xml` |
| Metal 3 Drawn vs Fill (`m3`) | ✅ PASS | 0 | 4.28 min | 2026-05-23T07:19:37.085403 | `../outputs/klayout_metal3_drawn_vs_fill_check.xml` |
| Metal Tiled Density (`mt`) | ✅ PASS | 0 | 3.34 min | 2026-05-23T07:19:37.085390 | `../outputs/klayout_metd_check.xml` |
| Special Created Layers (`cl`) | ✅ PASS | 0 | 2.78 min | 2026-05-23T07:19:37.085515 | `../outputs/klayout_special_cl_check.xml` |
| Boundary Check (`bc`) | ✅ PASS | 0 | 1.94 min | 2026-05-23T07:19:37.085521 | `../outputs/klayout_boundary_check.xml` |
| FOM LI Overlap (`fl`) | ✅ PASS | 0 | 1.88 min | 2026-05-23T07:19:37.085412 | `../outputs/klayout_foml_check.xml` |
| Poly Density (`pd`) | ❌ FAIL | 1483 | 1.05 min | 2026-05-23T07:19:37.085381 | `../outputs/klayout_poly_check.xml` |
| Missing Cells (`mc`) | ✅ PASS | 0 | 1.03 min | 2026-05-23T07:19:37.085465 | `../outputs/klayout_missing_cells_check.xml` |
| FOM2 (2000x2000) (`f2`) | ✅ PASS | 0 | 0.72 min | 2026-05-23T07:19:37.085461 | `../outputs/klayout_fom_density_2000_check.xml` |
| FOM1 (700x700) (`f1`) | ✅ PASS | 0 | 0.72 min | 2026-05-23T07:19:37.085456 | `../outputs/klayout_fom_density_700_check.xml` |
| GPIO Check (`gp`) | ✅ PASS | 0 | 0.42 min | 2026-05-23T07:19:37.085501 | `../outputs/klayout_gp_check.xml` |
| id (`id`) | ✅ PASS | 0 | 0.41 min | 2026-05-23T07:19:37.085496 | `../outputs/klayout_id_check.xml` |
| Illegal Character (`ic`) | ✅ PASS | 0 | 0.40 min | 2026-05-23T07:19:37.085484 | `../outputs/klayout_illegal_char_check.xml` |
| Spike Check (`sp`) | ✅ PASS | 0 | 0.39 min | 2026-05-23T07:19:37.085425 | `../outputs/klayout_spike_check.xml` |
| Big FOM Density Delta (`bd`) | ✅ PASS | 0 | 0.00 min | 2026-05-23T07:19:37.085475 | No report |
| Little FOM Density Delta (`ld`) | ✅ PASS | 0 | 0.00 min | 2026-05-23T07:19:37.085479 | No report |

---

## 🏢 About ChipFoundry

**ChipFoundry** is a leading platform for advanced chip design and verification, providing comprehensive tools and services for the semiconductor industry.

### 🌐 Resources
- **Website:** [https://chipfoundry.io](https://chipfoundry.io)
- **Documentation:** [https://chipfoundry.io/docs](https://chipfoundry.io/docs)
- **Support:** [https://chipfoundry.io/support](https://chipfoundry.io/support)

---
*Report generated by ChipFoundry Tapeout Tools on 2026-05-23 07:19:38*