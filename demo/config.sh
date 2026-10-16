# Demo settings, sourced by demo/run_demo.sh. Every value can be overridden from the
# environment or as a make variable, e.g.  make demo METHODS="lego dlo"
# Empty values mean "use the default".

# Methods to run, in this order (any subset of: hdl lego aloam dlo kiss mesh).
METHODS="${METHODS:-hdl lego aloam dlo kiss mesh}"

# Sequence in datasets/sequences.yaml: LiDAR topic, sensor settings, lever arm and the
# ground truth gt/<SEQ>/gt_tum.txt. cf_6 = CitrusFarm 06_14B_Jackal, as in the paper.
SEQ="${SEQ:-cf_6}"

# Output folder (an existing one is moved aside to <OUT>.<date-time>, never overwritten).
OUT="${OUT:-demo_output}"

# Play only the first DURATION seconds (empty = the whole bag, as in the paper).
# A partial run is not comparable with the paper's values; use it as a quick check.
DURATION="${DURATION:-}"

# ---------------------------------------------------------------- demo data
# The demo plays the original public CitrusFarm bag of sequence 06 (302 s; 2.49 GB;
# LiDAR, IMU, GNSS and magnetometer). Only /velodyne_points is played, exactly as in the
# paper's runs. Data: CitrusFarm dataset (Teng et al., 2023), CC BY-SA 4.0,
# https://ucr-robotics.github.io/Citrus-Farm-Dataset/. Size and MD5 are the official
# values of the dataset's file list (datasets/citrusfarm_files.csv).
DEMO_DATA_DIR="${DEMO_DATA_DIR:-demo_data}"
DEMO_BAG_URL="${DEMO_BAG_URL:-https://ucr-robotics.s3.us-west-2.amazonaws.com/citrus-farm-dataset/06_14B_Jackal/base_2023-07-18-14-29-35_0.bag}"
DEMO_BAG_SIZE="${DEMO_BAG_SIZE:-2486606484}"
DEMO_BAG_MD5="${DEMO_BAG_MD5:-b2135ac96dc6fc14ef4d9d2719227a53}"

# Your own bag instead of the demo bag: BAG=/path/to/file.bag (no download, no checksum).
# SEQ must then name an entry of datasets/sequences.yaml for that recording (README).
BAG="${BAG:-}"

# Set SKIP_CHECKSUM=1 to skip the MD5 check of an already downloaded demo bag (about 5 s).
SKIP_CHECKSUM="${SKIP_CHECKSUM:-}"

# ---------------------------------------------------------------- images and disk
# Image names: demo/images.conf (registry name and local build tag per method).
IMAGES_CONF="${IMAGES_CONF:-demo/images.conf}"

# Free space required where Docker stores images, when images still have to be pulled.
# The seven images share one base: about 1.3 GB to download and about 4.7 GB unpacked
# (docker images lists each image with the shared base, 3.3-3.8 GB each). 12 GB leaves
# room for the compressed copies Docker keeps during and after the pull.
MIN_FREE_GB_IMAGES="${MIN_FREE_GB_IMAGES:-12}"
# Free space required for the outputs (about 0.1 GB for six methods).
MIN_FREE_GB_OUT="${MIN_FREE_GB_OUT:-1}"
