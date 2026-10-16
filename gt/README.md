# Processed ground truth

Written by `scripts/process_gt.py` (`make gt`) from the datasets' ground-truth bags. These files let `make results` run without any dataset file. Each `gt/<seq>/` holds:

- `gt_tum.txt`: ground-truth positions in TUM format (`t x y z qx qy qz qw`), with header stamps and identity orientation (the ground truth has positions only).
- `gt_quality.json`: frame, rate, time span, gaps, path length and RTK quality (fix status, reported covariance, satellites, consistency with the raw fixes).
- `gt_status.pdf`, `gt_status.png`: trajectory coloured by RTK fix status.

## CitrusFarm (`cf_2`, `cf_3`, `cf_5`, `cf_6`, `cf_7`)

- **Source:** CitrusFarm dataset, H. Teng, Y. Wang, X. Song and K. Karydis, "Multimodal Dataset for Localization, Mapping and Crop Monitoring in Citrus Tree Farms", ISVC 2023; https://ucr-robotics.github.io/Citrus-Farm-Dataset/. Files used: `ground_truth/<folder>/gt.bag`, topic `/gps/fix/odometry` (post-processed RTK positions in the dataset's `rtk_path_frame`), and, for the RTK quality fields of `gt_quality.json`, `/piksi/navsatfix_best_fix` and `/piksi/debug/receiver_state` in the `base_*.bag` files.
- **Changes made:**
  - converted to TUM text, with positions only (in the dataset's `rtk_path_frame`, unchanged);
  - message header stamps instead of bag recording times;
  - quality statistics and plots added.
- **Licence:** these files are adapted from the CitrusFarm dataset, which is released under the Creative Commons Attribution-ShareAlike 4.0 International licence (CC BY-SA 4.0, https://creativecommons.org/licenses/by-sa/4.0/). They are shared under the same licence.

## Pullally (`pl_1`)

- **Source:** Pullally dataset, Nazate-Burgos, Torres-Torriti, Huang and Auat Cheein (2026), *Computers and Electronics in Agriculture* 247, 111687, https://doi.org/10.1016/j.compag.2026.111687; release https://github.com/RAL-UC/Pullally_Dataset (MIT licence, "Copyright (c) 2025 Robotics and Automation Laboratory - UC Chile").
- **File used:** run 1 (6 August 2023), ground-truth bag `GT/gt.bag` (SHA-256 in `datasets/pullally/SHA256SUMS`), topic `/fix` (`sensor_msgs/NavSatFix`, 2 Hz).
- **Availability:** `GT/gt.bag` is not part of the public Pullally release; it is available from the corresponding author of the article on request.
- **Changes made:**
  - converted to local East-North-Up coordinates with pymap3d, with the origin at the first fix (its latitude, longitude and altitude are in `gt_quality.json`);
  - converted to TUM text;
  - precision estimated from the data and added to `gt_quality.json`.
