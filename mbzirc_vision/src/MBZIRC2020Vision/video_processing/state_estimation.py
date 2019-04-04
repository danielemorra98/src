import math


# RUNCAM2 parameters. Resolution: 640x480 (fixed) 1/3'' sensor 4:3
CAM_FOCAL = 2.8 / 1000          # [m] focal length
CAM_WIDTH = 4.8 / 1000          # [m] sensor width
CAM_HEIGHT = 3.6 / 1000         # [m] sensor height
CAM_DIAG = 6.0 / 1000           # [m] sensor diagonal

# Balloon properties
BALL_RADIUS = 0.15              # [m] balloon radius

# Camera calibration parameters. They must be updated every time a new calibration is done.
CALIBR_DIST = 2.40                                         # [m] distance betwen balloon and camera
CALIBR_FOV_W = CAM_WIDTH / CAM_FOCAL * CALIBR_DIST         # [m] horizontal Field of View
CALIBR_FOV_H = CAM_HEIGHT / CAM_FOCAL * CALIBR_DIST        # [m] vertical Field of View
CALIBR_BALL_RADIUS_PIX = 800 / CALIBR_FOV_W * BALL_RADIUS  # [pixel] balloon radius in pixel


class StateEstimator():


    def __init__(self):

        self.w_frame = 0
        self.h_frame = 0
        self.frame_center_x = 0
        self.frame_center_y = 0
        self.x = 0
        self.y = 0
        self.radius_pix = 0


    def get_distance(self):

        return CALIBR_DIST * CALIBR_BALL_RADIUS_PIX / self.radius_pix


    def _get_pos_error_pix(self):

        err_x_pix = self.x - self.frame_center_x  	# [pixel] x offset from the center
        err_y_pix = self.y - self.frame_center_y  	# [pixel] y offset from the center

        return err_x_pix, err_y_pix


    def get_pos_error_m(self):

        err_x_pix, err_y_pix = self._get_pos_error_pix()
        dist = self.get_distance()

        fov_w = CALIBR_FOV_W / CALIBR_DIST * dist   # [m] horizontal field of view at balloon distance
        fov_h = CALIBR_FOV_H / CALIBR_DIST * dist   # [m] vertical field of view at balloon distance
        resol_w = fov_w / self.w_frame              # [m/pixel] horizontal resolution 
        resol_h = fov_h / self.h_frame              # [m/pixel] vertical resolution
        err_x_m = resol_w * err_x_pix               # [m] x error from the center
        err_y_m = resol_h * err_y_pix               # [m] y error from the center

        return err_x_m, err_y_m


    def get_pos_error_angle(self):

        dist = self.get_distance()
        err_x_m, err_y_m = self.get_pos_error_m()

        psi_err = 180 / math.pi * math.atan(err_x_m / dist)   #[deg] yaw error
        theta_err = 180 / math.pi * math.atan(err_y_m / dist) #[deg] pitch error

        return psi_err, theta_err


    def get_recomputed_radius(self):

        dist = self.get_distance()

        fov_w = CALIBR_FOV_W / CALIBR_DIST * dist   # [m] horizontal field of view at balloon distance
        fov_h = CALIBR_FOV_H / CALIBR_DIST * dist   # [m] vertical field of view at balloon distance
        resol_w = fov_w / self.w_frame              # [m/pixel] horizontal resolution
        resol_h = fov_h / self.h_frame              # [m/pixel] vertical resolution

        radius = self.radius_pix * resol_w          # [m] object radius (computed again for validation)

        return radius


    def put_observation(self, x, y, radius_pix, w_frame, h_frame):

        self.w_frame = w_frame
        self.h_frame = h_frame
        self.frame_center_x = int(self.w_frame / 2)
        self.frame_center_y = int(self.h_frame / 2)

        self.x = x
        self.y = y
        self.radius_pix = radius_pix
