import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def read_g2o_viewer_file(file_path):
    poses = []
    edges = []
    
    with open(file_path, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if line.startswith('VERTEX_SE3:QUAT'):
                x = float(parts[2])
                y = float(parts[3])
                z = float(parts[4])
                qx = float(parts[5])
                qy = float(parts[6])
                qz = float(parts[7])
                qw = float(parts[8])
                poses.append((x, y, z, qx, qy, qz, qw))
            elif line.startswith('EDGE_SE3:QUAT'):
                vertex_ids = [int(part) for part in parts[1:3]]
                edges.append(vertex_ids)
    
    return poses, edges

def plot_trajectory(poses, edges):
    plt.figure()
    
    x = [pose[0] for pose in poses]
    y = [pose[1] for pose in poses]
    plt.plot(x, y, 'b-',  linewidth= 0.3, label='fast_livo_opt' , marker='o' ,markersize=0.5 )
    # plt.scatter(x[0], y[0], color='r', label='Start' , marker='>' ,linewidth=8 ) 
    # plt.scatter(x[-1], y[-1], color='g', label='End' , marker='>' ,linewidth=8 ) 
    label_ed = False
    for edge in edges:
        # print('edge:', edge )
        if  abs(edge[0] - edge[1]) < 2:
            continue
        v1 = poses[edge[0]-1]
        v2 = poses[edge[1]-1]

        if label_ed:
            plt.plot([v1[0], v2[0]], [v1[1], v2[1]], 'r-' ,linewidth=0.3, marker='*' ,markersize=10 )
        else:
            label_ed = True
            plt.plot([v1[0], v2[0]], [v1[1], v2[1]], 'r-' ,linewidth=0.3, marker='*' ,markersize=10, label='fast_livo_loopclosure_opt' )
    
    plt.xlabel('X : m')
    plt.ylabel('Y : m ')
    plt.title('fast_livo_opt')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    # plt.savefig('./plot.png')
    plt.grid(False)
    plt.show()




if __name__ == "__main__":
    g2o_file = '/home/map/region4-3/pose_graph/graph.g2o'
    # g2o_file = '/home/map/rgb_test_big/pose_graph/graph.g2o'
    print('g2o_file:', g2o_file )
    poses, edges = read_g2o_viewer_file(g2o_file)
    plot_trajectory(poses, edges)