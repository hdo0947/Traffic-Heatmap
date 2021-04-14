oxt_dir="0014"
if [ ! -d "data_set/$oxt_dir/velodyne/data" ] && [ ! -d "data_set/$oxt_dir/oxts/data" ]
then
  echo "Downloading"
  if [ ! -d "2011_09_26_drive_0014_sync.zip" ]
  then
    wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0014/2011_09_26_drive_0014_sync.zip
  else
    echo "Exists, Stop Downloading"
  fi
  unzip 2011_09_26_drive_0014_sync.zip
  mkdir -p data_set/0014/oxts/
  mkdir -p data_set/0014/velodyne
  mv  2011_09_26/2011_09_26_drive_0014_sync/velodyne_points/data/  data_set/0014/velodyne
  mv  2011_09_26/2011_09_26_drive_0014_sync/oxts/data/  data_set/0014/oxts
  rm -rf 2011_09_26/
  rm 2011_09_26_drive_0014_sync.zip
else
  echo "Escape Downloading!";
fi

if [ ! -d "matlab/pcd_data/$oxt_dir" ]
then
  mkdir -p matlab/pcd_data/$oxt_dir
fi

cd matlab
matlab -nodisplay -nosplash -nodesktop -r "data_preprocess('$oxt_dir');exit;"
cd ..
velodyne_dir="data_set/$oxt_dir/velodyne/data";
all_velodyne_files=$(ls ${velodyne_dir});
gnng_mat_dir="matlab/pcd_data/$oxt_dir"
len=$(ls ${velodyne_dir}| wc -l)
len2=$(ls ${gnng_mat_dir}| wc -l)
if [ ! $len2 = $(($len * 2)) ]
then
  echo "Doing ground segmentation...."
  count=1;
  source progressbar.sh || exit 1
# shellcheck disable=SC2045
  for entry in $(ls ${velodyne_dir})
  do
    gfilename="${entry%.bin}"ground.pcd;
    ngfilename="${entry%.bin}"notground.pcd;file:///home/kardel/project568_py3.5/map.html
    progressbar "Ground Segmentation" $count $len

  done
else
  echo "Have already done ground segmentation!"
fi



cd matlab
echo "Tracking"
matlab -nodisplay -nosplash -nodesktop -r "run demo_car_no_plotting.m;exit;"
echo "Generating the Heatmap"
matlab -nodisplay -nosplash -nodesktop -r "run generateHeatmap.m;exit;"
cd ..
/usr/bin/python3 ./representation/map.py
xdg-open map.html


