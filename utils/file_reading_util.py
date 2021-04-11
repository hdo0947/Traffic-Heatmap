import matlab.engine
import numpy as np
def getCarPose(matlab_path, oxts_file):
    eng = matlab.engine.start_matlab()
    eng.addpath(matlab_path,nargout=0)
    pose = eng.loadCarPose(oxts_file)
    # eng.disp(pose)

    pose = np.array(pose._data).reshape(4,4)
    return pose

def getCarCoordinate(matlab_path, oxts_file):
    pose = getCarPose(matlab_path, oxts_file)
    print(pose)
    x = pose[0, 3]
    y = pose[1, 3]
    z = pose[2, 3]
    return np.array([x, y, z])


if __name__ == '__main__':
    getCarPose("../matlabToolkits", "../oxts/data/0000000000.txt")