if [ -d "data_set/0014/velodyne" ] && [ -d "data_set/0014/oxts" ]
then
wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0014/2011_09_26_drive_0014_sync.zip
unzip 2011_09_26_drive_0014_sync.zip
mkdir -p data_set/0014/oxts
mkdir -p data_set/0014/velodyne
mv  2011_09_26/2011_09_26_drive_0014_sync/velodyne_points/data/  data_set/0014/velodyne
mv  2011_09_26/2011_09_26_drive_0014_sync/oxts/data/  data_set/0014/oxts
rm -rf 2011_09_26/
rm 2011_09_26_drive_0014_sync.zip
fi

if [ -d "matlab/pcd_data/$oxt_dir" ]
then
  mkdir -p matlab/pcd_data/$oxt_dir
fi

cd matlab
oxt_dir="0014"
matlab -nodisplay -nosplash -nodesktop -r "data_preprocess('$oxt_dir');exit;"
cd ..
velodyne_dir="data_set/$oxt_dir/velodyne/data";
all_velodyne_files=$(ls ${velodyne_dir});
len=$(ls ${velodyne_dir}| wc -l)
echo $len;
count=1;
source progressbar.sh || exit 1
# shellcheck disable=SC2045
for entry in $(ls ${velodyne_dir})
do
  gfilename="${entry%.bin}"ground.pcd;
  ngfilename="${entry%.bin}"notground.pcd;
  /usr/bin/python3 python_/main.py --input_file $velodyne_dir/$entry --output_ground_filename matlab/pcd_data/$oxt_dir/$gfilename --output_notground_filename matlab/pcd_data/$oxt_dir/$ngfilename
#  draw_progress_bar $count $len "epochs"
  ((count=count+1))
  progressbar "Ground Segmentation" $count $len

done

cd matlab
matlab  -r "run demo_car.m"

