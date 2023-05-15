import numpy as np

id = np.arange(22)
face = np.array([3, 4, 1, 2, 1, 4, 2, 3, 2, 4, 2, 3, 4, 3, 1, 2, 4, 1, 3, 1, 3, 4])
# coordinate = [(10, 140), "", "", (130, 30), "", "", (130, 170), (140, 270), (310, 120)
# , "", (310, 255), (270, 270), "", (140, 140), (270, 170), "", ""
# , (270, 140), "", (10, 0), (310, 300)]
coordinate = np.array([[20,230], [35,550], [32,550], [250,530], [25,290], [290,550],
                       [250,270], [270,60], [250,20], [290,30],
                       [530,20], [530,60], [25,290], [270,330],
                       [320,270], [510,270], [320,310], [290,550], [530,330], [560,290],
                       [20,510], [570, 20]])

#modified:0,3,6,13,14,16


parameter = np.transpose(np.vstack([id, face]))
parameter = np.hstack([parameter, coordinate])
print(parameter)
np.savez('../aruco_param.npz', parameter = parameter)
ep = np.load('aruco_param.npz')
ep = ep['parameter']
print(ep)

