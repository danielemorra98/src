from collections import namedtuple

ProcessedFrame = namedtuple("ProcessedFrame",
    'frame locked err_x_pix err_y_pix err_x_m err_y_m dist psi_err theta_err')
