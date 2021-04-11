dbclear all;

% compile matlabToolkits wrappers
disp('Building wrappers ...');
mex('readTrackletsMex.cpp','-I../cpp','-lboost_serialization');
disp('...done!');
