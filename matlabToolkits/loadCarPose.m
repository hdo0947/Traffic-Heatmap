function pose = loadCarPose(oxt_filename)
%LOADCARPOSE Summary of this function goes here
%   Detailed explanation goes here
thisdata = load(oxt_filename);
oxts_ = cell(1,1);
oxts_{1} = thisdata;
pose_cell = convertOxtsToPose(oxts_);
pose = pose_cell{1,1};
end

