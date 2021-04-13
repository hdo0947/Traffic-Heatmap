cd matlab
oxt_dir="0014"
matlab -nodisplay -nosplash -nodesktop -r "data_preprocess('$oxt_dir');exit;"
cd ..

velodyne_dir="data_set/0014/velodyne/data";



# originally I wanted to write by myself, then I screwed up. now I only copy&paste from stack overflow
# https://stackoverflow.com/questions/238073/how-to-add-a-progress-bar-to-a-shell-script
function ProgressBar {
# Process data
    let _progress=(${1}*100/${2}*100)/100
    let _done=(${_progress}*4)/10
    let _left=40-$_done
# Build progressbar string lengths
    _fill=$(printf "%${_done}s")
    _empty=$(printf "%${_left}s")

# 1.2 Build progressbar strings and print the ProgressBar line
# 1.2.1 Output example:
# 1.2.1.1 Progress : [########################################] 100%
printf "\rProgress : [${_fill// /#}${_empty// /-}] ${_progress}%%"

}




all_velodyne_files=$(ls ${velodyne_dir});
len=$(ls ${velodyne_dir}| wc -l)
echo $len;
count=1;
# shellcheck disable=SC2045
for entry in $(ls ${velodyne_dir})
do
  gfilename="${entry%.bin}"grond.pcd;
  ngfilename="${entry%.bin}"notgrond.pcd;
 ((count=count+1))
  sleep 0.01
  ProgressBar $count $len
  /usr/bin/python3 python_/main.py --input_file $velodyne_dir/$entry --output_ground_filename matlab/pcd_data/$oxt_dir/$gfilename --output_notground_filename matlab/pcd_data/$oxt_dir/$ngfilename
done

cd matlab
matlab -nodisplay -nosplash -nodesktop -r "demo_car.m"
