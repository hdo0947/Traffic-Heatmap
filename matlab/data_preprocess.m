function data_preprocess(FolderName)
    
    oxts_ = readOxts(FolderName);
%    disp(oxts_);
    pose = convertOxtsToPose(oxts_).';
    save('./true_pose.mat', 'pose');
end