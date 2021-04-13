function oxts_ = readOxts(FolderName)

    files = dir(sprintf('../data_set/%s/oxts/data/*.txt',FolderName));
    N = length(files);
    oxts_ = cell(N,1);
    for i = 1:N
        thisfilename = sprintf('../data_set/%s/oxts/data/%010d.txt', FolderName, i-1);
        thisdata = load(thisfilename);
        oxts_{i} = thisdata;
    end
    save('./oxts.mat', 'oxts_');

end