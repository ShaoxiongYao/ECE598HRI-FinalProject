import multiprocessing
import numpy as np
from copy import deepcopy
import time
import warnings
try:
    import pyrealsense2 as rs
    HAS_RS2 = True
except ImportError:
    HAS_RS2 = False

class RealSenseCamera:
    """Driver for a realsense camera. Handles L515s and SRs.
    REMARK: `update()` can only be called in the same process in
        which this driver was instantiated, otherwise, it will
        runtime error saying frames did not arrive for 5000 ms.
    """
    def __init__(self, serial_num,camera_type,extra_settings):
        if not HAS_RS2:
            warnings.warn("Realsense camera data is not available because the pyrealsense2 module is not installed")
            self.pipeline = None
            return

        self.extra_settings_dict = {
            'laser_power':rs.option.laser_power,
            'min_distance':rs.option.min_distance,
            'receiver_gain':rs.option.receiver_gain,
            'noise_filtering':rs.option.noise_filtering,
            'pre_processing_sharpening':rs.option.pre_processing_sharpening,
            'post_processing_sharpening':rs.option.post_processing_sharpening,
            'confidence_threshold':rs.option.confidence_threshold
        }
        self.serial_num = serial_num
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.latest_aligned_frame = None
        self.frame_time = time.time()
        try:
            self.config.enable_device(serial_num.encode('utf-8'))
            if(camera_type == 'SR'):
                self.config.enable_stream(
                    rs.stream.depth, 640, 480, rs.format.z16, 30)
                self.config.enable_stream(
                    rs.stream.color, 640, 480, rs.format.rgb8, 30)
                self.numpix = 640*480
            elif(camera_type == 'L515'):
                self.config.enable_stream(
                    rs.stream.depth, 640, 480, rs.format.z16, 30)
                self.config.enable_stream(
                    rs.stream.color, 1280, 720, rs.format.rgb8, 30)
                self.numpix  = 1280*720
            else:
                print('\n SensorModule: Failed to initialize camera with serial number {} due to unsuported camera type {}. Please review the camera_settings_file and try again.'.format(serial_num,camera_type))
                raise TypeError('Non-supported camera type selected found!')
            self.align_to = rs.stream.color
            self.align = rs.align(self.align_to)
            # Start streaming
            self.profile = self.pipeline.start(self.config)
            self.depth_sensor = self.profile.get_device().first_depth_sensor()
            self.depth_scale = self.depth_sensor.get_depth_scale()
            if(camera_type == 'L515'):
                if(extra_settings is not None):
                    print("Starting realsense L515 with extra settings")
                    print(extra_settings)
                    for key in extra_settings.keys():
                        prop = self.extra_settings_dict[key]
                        self.depth_sensor.set_option(prop,extra_settings[key])

                        print(self.depth_sensor.get_option(prop))
            # we sleep for 3 seconds to stabilize the color image - no idea why, but if we query it soon after starting, color image is distorted.
            self.pc = rs.pointcloud()
        except Exception as e:
            print(e, 'Invalid Camera Serial Number')
            self.pipeline.stop()
        # atexit.register(self.safely_close)

    def update(self):
        if self.pipeline is None:
            return
        frames = self.pipeline.wait_for_frames()

        self.frame_time = time.time()
        self.pc_time = self.frame_time
        if not frames.get_depth_frame() or not frames.get_color_frame():
            return
        # Fetch color and depth frames and align them
        aligned_frames = self.align.process(frames)
        depth_frame = np.asarray(aligned_frames.get_depth_frame())
        color_frame = aligned_frames.get_color_frame()
        self.latest_capture_time = time.time()
        self.latest_aligned_frame = aligned_frames

    def latest_point_cloud(self):
        """
        Returns the point cloud from the last frame.

        Args:
        Returns:
            transformed_pc : returns the point cloud in the camera's local frame
            as a numpy array PointCloud, or None if the data is not available
        """
        if self.pipeline is None:
            return None
        if self.latest_aligned_frame is None:
            print("Data Not Available at the moment")
            return None
        # Fetch color and depth frames and align them
        depth_frame = self.latest_aligned_frame.get_depth_frame()
        color_frame = self.latest_aligned_frame.get_color_frame()

        # Tell pointcloud object to map to this color frame
        self.pc.map_to(color_frame)
        # Generate the pointcloud and texture mappings
        points = self.pc.calculate(depth_frame)
        vtx = np.asarray(points.get_vertices())
        pure_point_cloud = np.zeros((self.numpix, 3))
        pure_point_cloud[:, 0] = vtx['f0']
        pure_point_cloud[:, 1] = vtx['f1']
        pure_point_cloud[:, 2] = vtx['f2']
        color_t = np.asarray(color_frame.get_data()).reshape(self.numpix, 3)/255
        return np.hstack((pure_point_cloud,color_t))
        # point_cloud = o3d.geometry.PointCloud()
        # point_cloud.points = o3d.utility.Vector3dVector(pure_point_cloud)
        # point_cloud.colors = o3d.utility.Vector3dVector(color_t)
        # return point_cloud

    def latest_rgbd_images(self):
        if self.pipeline is None:
            return None
        if self.latest_aligned_frame is None:
            print("Data Not Available at the moment")
            return None
        depth_frame = self.latest_aligned_frame.get_depth_frame()
        color_frame = self.latest_aligned_frame.get_color_frame()

        tmp_depth = deepcopy(np.array(depth_frame.get_data())).astype(float)
        tmp_depth = tmp_depth*self.depth_scale
        depth_mm = (tmp_depth*1000).astype(np.uint16)

        return [deepcopy(np.array(color_frame.get_data())),depth_mm]

    def safely_close(self):
        if self.pipeline is None:
            return
        print('safely closing Realsense camera', self.serial_num)
        self.pipeline.stop()
        self.pipeline = None

def go():
    cam = RealSenseCamera("f0190400", "L515", None)
    for i in range(2):
        print(i)
        cam.update()
        a = cam.latest_rgbd_images()
        # b = cam.latest_point_cloud()
        from matplotlib import pyplot as plt
        plt.imshow(a[1])
        plt.show()
        # time.sleep(0.1)

if __name__ == '__main__':
    # cam = RealSenseCamera("f0190400","L515",{
    #         "laser_power":0,
    #         "min_distance":0.0,
    #         "receiver_gain":18.0,
    #         "noise_filtering":2.0,
    #         "pre_processing_sharpening":2.0
    #         })
    print(help(rs.option))
    # p = multiprocessing.Process(target=go)
    # p.start()
    # p.join()
