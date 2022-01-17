import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import imageio
from matplotlib.animation import FFMpegWriter

CAR_BOX_SIZE = [1.6, 3.8] # unit: m

class DataStruct:
    def __init__(self, data):
        self.frame_id = int(data[0])
        self.time_stamp = data[1]
        self.sensor_id = int(data[2])
        self.z = data[3]
        self.x = data[4]
        self.y = data[5]
        self.vz = data[6]
        self.vx = data[7]
        self.vy = data[8]


def readTxt(filename):
    with open(filename, 'r') as f:
        lines = f.readlines()
        data_frame = []
        for i, line in enumerate(lines):
            if i == 0: continue
            data_line = line.split()
            data_line = [float(data) for data in data_line]
            if len(data_frame) == 0 or data_frame[0][0] == data_line[0]:
                data_frame.append(data_line) 
            else:
                yield np.array(data_frame)
                data_frame = []
                data_frame.append(data_line)


def rectanglePatch(data, color):
    w, h = CAR_BOX_SIZE[0], CAR_BOX_SIZE[1]
    top_left_x = -data.x - w / 2
    top_left_z = data.z
    angle = np.arctan( data.vz / -data.vx)
    angle = 90 - abs(angle / np.pi * 180)
    angle = 0 if angle > 45 else angle
    # angle = 0
    return patches.Rectangle((top_left_x, top_left_z), w, h, angle=angle , fill=False, edgecolor=color, linewidth=1)



if __name__ == "__main__":

    simulation_path = "./small_session_camcam_txt.txt"

    metadata = dict(title='test', artist='Matplotlib',comment='Movie support!')
    writer = FFMpegWriter(fps=15, metadata=metadata)

    plt.ion()
    fig = plt.figure()
    ax1 = fig.add_subplot(111)
    with writer.saving(fig, "test.mp4", 100):
        for data in readTxt(simulation_path):
            if len(data) == 0:
                continue
            # plt.cla()
            ax1.clear()
            labeled_radar = False
            labeled_camera = False
            for data_sensor_raw in data:
                data_sensor = DataStruct(data_sensor_raw)
                color = 'r' if data_sensor.sensor_id == 1 else 'b'
                marker = '^' if data_sensor.sensor_id == 1 else 'o'
                label = "camera" if data_sensor.sensor_id == 1 else "radar"
                rect = rectanglePatch(data_sensor, color)
                if label == "camera" and not labeled_camera:
                    labeled_camera = True
                    ax1.scatter(-data_sensor.x, data_sensor.z, s=30, c=color, marker=marker, label=label)
                    ax1.add_patch(rect)
                if label == "radar" and not labeled_radar:
                    labeled_radar = True
                    ax1.scatter(-data_sensor.x, data_sensor.z, s=30, c=color, marker=marker, label=label)
                    ax1.add_patch(rect)
                ax1.scatter(-data_sensor.x, data_sensor.z, s=30, c=color, marker=marker)
                ax1.add_patch(rect)
                ax1.quiver(-data_sensor.x, data_sensor.z, -data_sensor.vx, data_sensor.vz, \
                            color=color, width = 0.003, scale=200)
            ax1.set_xlim(-60, 60)
            ax1.set_ylim(0, 60)
            ax1.legend()
            fig.canvas.draw()
            plt.pause(0.01)
            # writer.grab_frame()
