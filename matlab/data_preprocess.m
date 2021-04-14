% FolderName is the name of the folder inside the data_set folder. The
% data_set folder is outside this matlab folder and it contains folders
% like: 
% 0014/oxts
% 0014/velodyne
% 0059/oxts
% 0059/velodyne
% So if you are using the KITTI dataset 2011_09_26_drive_0014_sync, then please 
% copy the oxts and velodyne folder to the data_set/0014 folder. The
% FolderName will then be '0014'. 
% Running example:
% data_preprocess('0014')

function data_preprocess(FolderName)
    
    oxts_ = readOxts(FolderName);
    pose = convertOxtsToPose(oxts_).';
    save(sprintf('true_pose_%s.mat', FolderName), 'pose');
end