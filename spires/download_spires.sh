#!/bin/bash

DOWNLOAD_DIR=${1:-"/home/data/oxford"}

pip install gdown

gdown 1k_jLnP2-7_2k2JmVvJS--RWNGxusKhWv -O "${DOWNLOAD_DIR}/bodleian2.bag"
gdown 1SJF-1FZ9VSNcBYM1N8Da80U3T0-u68cs -O "${DOWNLOAD_DIR}/christ5.bag"
gdown 1UKGEYxShWH7UcZOrnkazYyq7w7_81uCV -O "${DOWNLOAD_DIR}/christ3.bag"
gdown 1ChnpJptHRaD8bLFpQBjGhq9RTB0ud_IC -O "${DOWNLOAD_DIR}/blenheim5.bag"
gdown 1dAfi8KAdCCZ6XEs1na4TjDCTdG5T7om0 -O "${DOWNLOAD_DIR}/blenheim2.bag"
gdown 1HlVtXEf1ZkWL6DpqRKPMmW_woY7CjNUQ -O "${DOWNLOAD_DIR}/blenheim1.bag"
gdown 1sa-5WaJW2S8huY8FCy_urS-vE9ZbO7QK -O "${DOWNLOAD_DIR}/observatory2.bag"
gdown 1DzhAwebjQ_HMwItezkUGUOtgv1lchJGu -O "${DOWNLOAD_DIR}/observatory1.bag"
gdown 1Yc6I6HTO2U3FrNd_MrMPCLh_Xf_V6utD -O "${DOWNLOAD_DIR}/keble3.bag"
gdown 17OBsiqxI6rf7QXghYjMuehxEVEKE6uNd -O "${DOWNLOAD_DIR}/keble2.bag"

gdown 1zEtmrhxmA9TMABjmktek5F16wb3HA_mV -O "${DOWNLOAD_DIR}/christ2_1.bag"
gdown 14071OBxBbV0BZCcOlqw4wTruZNTarBam -O "${DOWNLOAD_DIR}/christ2_2.bag"
gdown 1mg1PYQozKl9TaE4PISGMXP-r9nfcpOFk -O "${DOWNLOAD_DIR}/christ1_1.bag"
gdown 1OQ_T5sewdiOlizEy43Op4t088uU7HB_7 -O "${DOWNLOAD_DIR}/christ1_2.bag"
gdown 1cMbCqVDagBrlmMmd0tWctIQM_Cw9vPDX -O "${DOWNLOAD_DIR}/keble5_1.bag"
gdown 1Ohn06m4b2vIw_zps1PTfNICAUvjHitKo -O "${DOWNLOAD_DIR}/keble5_2.bag"
gdown 1WR6MJ6xK-CP2RmKfBRKKtWh6z2tEBOCJ -O "${DOWNLOAD_DIR}/keble4_1.bag"
gdown 1uGYvAMbjtlOZkskVBbDYwDaR-MOI_Plc -O "${DOWNLOAD_DIR}/keble4_2.bag"

python3 stitch_bags.py "${DOWNLOAD_DIR}/christ2_1.bag" "${DOWNLOAD_DIR}/christ2_2.bag" "${DOWNLOAD_DIR}/christ2.bag"
rm "${DOWNLOAD_DIR}/christ2_1.bag"
rm "${DOWNLOAD_DIR}/christ2_2.bag"

python3 stitch_bags.py "${DOWNLOAD_DIR}/christ1_1.bag" "${DOWNLOAD_DIR}/christ1_2.bag" "${DOWNLOAD_DIR}/christ1.bag"
rm "${DOWNLOAD_DIR}/christ1_1.bag"
rm "${DOWNLOAD_DIR}/christ1_2.bag"

python3 stitch_bags.py "${DOWNLOAD_DIR}/keble5_1.bag" "${DOWNLOAD_DIR}/keble5_2.bag" "${DOWNLOAD_DIR}/keble5.bag"
rm "${DOWNLOAD_DIR}/keble5_1.bag"
rm "${DOWNLOAD_DIR}/keble5_2.bag"

python3 stitch_bags.py "${DOWNLOAD_DIR}/keble4_1.bag" "${DOWNLOAD_DIR}/keble4_2.bag" "${DOWNLOAD_DIR}/keble4.bag"
rm "${DOWNLOAD_DIR}/keble4_1.bag"
rm "${DOWNLOAD_DIR}/keble4_2.bag"