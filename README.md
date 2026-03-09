# FORM to HDMapping simplified instruction

## Step 1 (prepare data)
Download the dataset `reg-1.bag` by clicking [link](https://cloud.cylab.be/public.php/dav/files/7PgyjbM2CBcakN5/reg-1.bag) (it is part of [Bunker DVI Dataset](https://charleshamesse.github.io/bunker-dvi-dataset)) and convert with [tool](https://github.com/MapsHD/livox_bag_aggregate) to 'reg-1.bag-pc.bag'.

File 'reg-1.bag-pc.bag' is an input for further calculations.
It should be located in '~/hdmapping-benchmark/data'.


## Step 2 (prepare docker)
```shell
mkdir -p ~/hdmapping-benchmark
cd ~/hdmapping-benchmark
git clone https://github.com/MapsHD/benchmark-FORM-to-HDMapping.git --recursive
cd benchmark-FORM-to-HDMapping
git checkout Bunker-DVI-Dataset-reg-1
docker build -t form_noetic .
```

## Step 3 (run docker, file 'reg-1.bag-pc.bag' should be in '~/hdmapping-benchmark/data')
```shell
cd ~/hdmapping-benchmark/benchmark-FORM-to-HDMapping
chmod +x docker_session_run-ros1-form.sh
cd ~/hdmapping-benchmark/data
~/hdmapping-benchmark/benchmark-FORM-to-HDMapping/docker_session_run-ros1-form.sh reg-1.bag-pc.bag .
```

## Step 4 (Open and visualize data)
Expected data should appear in ~/hdmapping-benchmark/data/output_hdmapping-form
Use tool [multi_view_tls_registration_step_2](https://github.com/MapsHD/HDMapping) to open session.json from ~/hdmapping-benchmark/data/output_hdmapping-form.

You should see following data in folder '~/hdmapping-benchmark/data/output_hdmapping-form'

lio_initial_poses.reg

poses.reg

scan_lio_*.laz

session.json

trajectory_lio_*.csv

## Contact email
januszbedkowski@gmail.com
