#!/bin/bash

# Copyright (c) Facebook, Inc. and its affiliates.

set -ex

mkdir -p ./src/frankmocap/extra_data/body_module/
cd ./src/frankmocap/extra_data/body_module/

echo "Downloading extra data from SPIN"
wget http://visiondata.cis.upenn.edu/spin/data.tar.gz && tar -xvf data.tar.gz && rm data.tar.gz
mv data data_from_spin

echo "Downloading pretrained model"
mkdir -p pretrained_weights
cd pretrained_weights
wget https://dl.fbaipublicfiles.com/eft/2020_05_31-00_50_43-best-51.749683916568756.pt
wget https://dl.fbaipublicfiles.com/eft/fairmocap_data/body_module/smplx-03-28-46060-w_spin_mlc3d_46582-2089_2020_03_28-21_56_16.pt
cd ..

echo "Downloading other data"
wget https://dl.fbaipublicfiles.com/eft/fairmocap_data/body_module/J_regressor_extra_smplx.npy

echo "Done"