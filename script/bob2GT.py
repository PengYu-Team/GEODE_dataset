from dateutil import parser
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
from pyquaternion import Quaternion

import re
import os

# def pq_to_tran(t_, q_ , T_ )
    
    
#     R = np.array([[1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
#                 [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
#                 [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]])
#     T_ = np.vstack((np.hstack((R, t[:, np.newaxis])),
#                 np.array([0, 0, 0, 1])))    

def color_map(data, cmap):
 
    dmin, dmax = np.nanmin(data), np.nanmax(data)
    cmo = plt.cm.get_cmap(cmap)
    cs, k = list(), 256/cmo.N
    
    for i in range(cmo.N):
        c = cmo(i)
        for j in range(int(i*k), int((i+1)*k)):
            cs.append(c)
    cs = np.array(cs)
    data = np.uint8(255*(data-dmin)/(dmax-dmin))
    
    return cs[data]

if __name__ == '__main__': 


    # define the extrinsic
    # -0.0108864 0.293397 -0.452674 0.999991 0.000903519 0.0035434 -0.00220342
    # 0.00947221 -0.308202 -0.365733 0.999901 -0.00492765 0.00575961 0.0117651
    # bob
    qw, qx, qy, qz = 0.999991, 0.000903519, 0.0035434, -0.00220342  # quant   
    t = np.array([-0.0108864, 0.293397, -0.452674])  # translation
    # carol
    # qw, qx, qy, qz = 0.999901, -0.00492765, 0.00575961, 0.0117651  # quant   
    # t = np.array([0.00947221, -0.308202, -0.365733])  # translation

    R = np.array([[1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
                [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
                [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]])
    # R_left2right = np.array([[1,0,0],
    #                          [0,-1,0],
    #                          [0,0,1]]) 
    # R = R_left2right * R          #### transform left hand frame to right hand frame

    T = np.vstack((np.hstack((R, t[:, np.newaxis])),
                np.array([0, 0, 0, 1])))
    print( T)

    raw_folder = "./Offroad1_alpha_rect"
    output_folder = "./tran2body"

    for file_name in os.listdir(raw_folder):
        if file_name.endswith(".txt"):
            input_file_path=os.path.join(raw_folder,file_name)
            output_file_path=os.path.join(output_folder,file_name)

            print("Current file is {}".format(input_file_path))
            with open(input_file_path, "r") as f_in, open(output_file_path, "w") as f_out:
                lines = f_in.readlines()
                for line in lines:
                    data = line.split()
                    timestamp = float(data[0])
                    x, y, z = float(data[1]), float(data[2]), float(data[3])
                    qx_m, qy_m, qz_m, qw_m = float(data[4]), float(data[5]), float(data[6]), float(data[7])
                    p_device = np.array([x, y, z])
                    q_device_quaternion = Quaternion(qw_m, qx_m, qy_m, qz_m)
                    R_device = q_device_quaternion.rotation_matrix
                    # R_device = np.array([[1 - 2*qy_m**2 - 2*qz_m**2, 2*qx_m*qy_m - 2*qz_m*qw_m, 2*qx_m*qz_m + 2*qy_m*qw_m],
                    #                     [2*qx_m*qy_m + 2*qz_m*qw_m, 1 - 2*qx_m**2 - 2*qz_m**2, 2*qy_m*qz_m - 2*qx_m*qw_m],
                    #                     [2*qx_m*qz_m - 2*qy_m*qw_m, 2*qy_m*qz_m + 2*qx_m*qw_m, 1 - 2*qx_m**2 - 2*qy_m**2]])
                    
                    T_device = np.vstack((np.hstack((R_device, p_device[:, np.newaxis])), np.array([0, 0, 0, 1])))
                    # print ("T_device:\n",T_device)
                    
                    T_device_to_eval = np.linalg.inv(T)
                    # print ("T_device_to_eval:\n",T_device_to_eval)
                    
                    T_eval = T_device @ T_device_to_eval
                    # print ("T_eval:\n",T_eval)

                    # 提取平移向量和四元数
                    t_eval = T_eval[:3, 3]
                    R_eval = T_eval[:3, :3]
                    # print ("R_eval:\n",R_eval)
                    q_eval = Quaternion(matrix=R_eval)
                    # #print (p_body)
                    # #print (q_body)
                    f_out.write("{:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(timestamp, t_eval[0], t_eval[1], t_eval[2], q_eval[1], q_eval[2], q_eval[3], q_eval[0])) 

        # # extract frame data
        # x = []
        # y = []
        # z = []
        # times = []
        # num_point =0
        # for line in lines:
        #     data = line.split()
        #     times.append(float(data[0]))
        #     x.append(float(data[1]))
        #     y.append(float(data[2]))
        #     z.append(float(data[3]))   
        #     num_point +=1
        # # plot 3d traj
        # times = np.array(times)
        
        # norm = plt.Normalize(times.min(), times.max())
        # colors = cm.ScalarMappable(norm=norm, cmap='jet').to_rgba(times)

        # fig = plt.figure(figsize=(16,12))
        # ax = fig.add_subplot(111, projection='3d')
        # ax.plot(x[:], y[:], z[:],color =colors[0], label= file_name)

        # for i in range(num_point -1):
        #     color1, color2 = colors[i], colors[i+1]
        #     ax.plot(x[i:i+2], y[i:i+2], z[i:i+2],color =color1)
        # ax.scatter(x[:],y[:],z[:], c=colors)


        # ##ax.plot(x, y, z, label=file_name, color =colors )
        # sm = plt.cm.ScalarMappable(cmap='jet', norm = norm)
        # sm.set_array([])
        # plt.colorbar(sm)
        
        # ax.legend()
        # ax.set_xlabel('X')
        # ax.set_ylabel('Y')
        # ax.set_zlabel('Z')
        
        # #plt.show()

        # image_file_name = os.path.splitext(file_name)[0] + ".png"
        # plt.savefig(os.path.join("image", image_file_name))
        # # plt.show(block=False )
        # # plt.pause(1)
        # plt.close()