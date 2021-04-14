function data_preprocess(FolderName)
    
    oxts_ = readOxts(FolderName);
    pose = convertOxtsToPose(oxts_).';
    save(sprintf('true_pose_%s.mat', FolderName), 'pose');
end