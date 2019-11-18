"""Webcam demo application.

Example commands:
    python3 -m pifpaf.webcam  # usbcam or webcam 0
    python3 -m pifpaf.webcam --source=1  # usbcam or webcam 1

    # streaming source
    python3 -m pifpaf.webcam --source=http://128.179.139.21:8080/video

    # file system source (any valid OpenCV source)
    python3 -m pifpaf.webcam --source=docs/coco/000000081988.jpg

Trouble shooting:
* MacOSX: try to prefix the command with "MPLBACKEND=MACOSX".
"""
import argparse
import time, math
import numpy as np
import PIL
import torch
import cv2  # pylint: disable=import-error
from openpifpaf.network import nets
from openpifpaf import decoder, show, transforms
import pyzed.sl as sl
import matplotlib.pyplot as plt


class Visualizer(object):
    def __init__(self, processor, args):
        self.processor = processor
        self.args = args

    def __call__(self, first_image, fig_width=4.0, **kwargs):
        if plt is None:
            while True:
                image, all_fields = yield
            return

        if 'figsize' not in kwargs:
            kwargs['figsize'] = (fig_width, fig_width * first_image.shape[0] / first_image.shape[1])

        fig = plt.figure(**kwargs)
        ax = plt.Axes(fig, [0.0, 0.0, 1.0, 1.0])
        ax.set_axis_off()
        ax.set_xlim(0, first_image.shape[1])
        ax.set_ylim(first_image.shape[0], 0)
        text = 'OpenPifPaf'
        ax.text(1, 1, text,
                fontsize=10, verticalalignment='top',
                bbox=dict(facecolor='white', alpha=0.5, linewidth=0))
        fig.add_axes(ax)
        mpl_im = ax.imshow(first_image)
        fig.show()

        # visualizer
        if self.args.colored_connections:
            viz = show.KeypointPainter(show_box=False, color_connections=True,
                                       markersize=1, linewidth=6)
        else:
            viz = show.KeypointPainter(show_box=False)

        while True:
            image, all_fields = yield
            annotations = self.processor.annotations(all_fields)

            draw_start = time.time()
            while ax.lines:
                del ax.lines[0]
            mpl_im.set_data(image)
            viz.annotations(ax, annotations)
            fig.canvas.draw()
            #print('draw', time.time() - draw_start)
            plt.pause(0.01)

        plt.close(fig)


def cli():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    nets.cli(parser)
    decoder.cli(parser, force_complete_pose=False, instance_threshold=0.1, seed_threshold=0.5)
    parser.add_argument('--no-colored-connections',
                        dest='colored_connections', default=True, action='store_false',
                        help='do not use colored connections to draw poses')
    parser.add_argument('--disable-cuda', action='store_true',
                        help='disable CUDA')
    parser.add_argument('--source', default='0',
                        help='OpenCV source url. Integer for webcams. Or ipwebcam streams.')
    parser.add_argument('--scale', default=0.1, type=float,
                        help='input image scale factor')
    args = parser.parse_args()

    # check whether source should be an int
    if len(args.source) == 1:
        args.source = int(args.source)

    # add args.device
    args.device = torch.device('cpu')
    if not args.disable_cuda and torch.cuda.is_available():
        args.device = torch.device('cuda')

    return args


def main():
    args = cli()

    # load model
    model, _ = nets.factory_from_args(args)
    model = model.to(args.device)
    processor = decoder.factory_from_args(args, model)

    # zed   
    init = sl.InitParameters()
    init.depth_mode = sl.DEPTH_MODE.DEPTH_MODE_ULTRA
    init.coordinate_units = sl.UNIT.UNIT_METER
    init.coordinate_system = sl.COORDINATE_SYSTEM.COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP

    cam = sl.Camera()
    status = cam.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit()

    runtime_parameters = sl.RuntimeParameters()    
    runtime_parameters.sensing_mode = sl.SENSING_MODE.SENSING_MODE_STANDARD  # Use STANDARD sensing mode
    
    img = sl.Mat()
    depth = sl.Mat()
    point_cloud = sl.Mat()

    last_loop = time.time()
    #capture = cv2.VideoCapture(args.source)

    visualizer = None
    while True:
        err = cam.grab(runtime_parameters)
        if err == sl.ERROR_CODE.SUCCESS:
            # Retrieve left image
            cam.retrieve_image(img, sl.VIEW.VIEW_LEFT)
            # Retrieve depth map. Depth is aligned on the left image
            cam.retrieve_measure(depth, sl.MEASURE.MEASURE_DEPTH)
            # Retrieve colored point cloud. Point cloud is aligned on the left image.
            cam.retrieve_measure(point_cloud, sl.MEASURE.MEASURE_XYZRGBA)

            # Get and print distance value in mm at the center of the image
            # We measure the distance camera - object using Euclidean distance
            x = round(img.get_width() / 2)
            y = round(img.get_height() / 2)
            err, point_cloud_value = point_cloud.get_value(x, y)
            err, depth_value = depth.get_value(x, y)
            print("depth ", depth_value)
            
            distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                                 point_cloud_value[1] * point_cloud_value[1] +
                                 point_cloud_value[2] * point_cloud_value[2])

            if not np.isnan(distance) and not np.isinf(distance):
                distance = round(distance)
                #print("Distance to Camera at ({0}, {1}): {2} mm\n".format(x, y, distance))
            else:
                print("Can't estimate distance at this position, move the camera\n")
            cv2.imshow("Depth", depth.get_data())
        else:
            print("Err", err)
            continue

        image = cv2.resize(img.get_data(), None, fx=args.scale, fy=args.scale)
        #print('resized image size: {}'.format(image.shape))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        if visualizer is None:
            visualizer = Visualizer(processor, args)(image)
            visualizer.send(None)

        start = time.time()
        image_pil = PIL.Image.fromarray(image)
        processed_image_cpu, _, __ = transforms.EVAL_TRANSFORM(image_pil, [], None)
        processed_image = processed_image_cpu.contiguous().to(args.device, non_blocking=True)
        #print('preprocessing time', time.time() - start)

        fields = processor.fields(torch.unsqueeze(processed_image, 0))[0]
        visualizer.send((image, fields))

        #print('loop time = {:.3}s, FPS = {:.3}'.format(
        #    time.time() - last_loop, 1.0 / (time.time() - last_loop)))
        last_loop = time.time()

    cam.close()

def print_camera_information(cam):
    print("Resolution: {0}, {1}.".format(round(cam.get_resolution().width, 2), cam.get_resolution().height))
    print("Camera FPS: {0}.".format(cam.get_camera_fps()))
    print("Firmware: {0}.".format(cam.get_camera_information().firmware_version))
    print("Serial number: {0}.\n".format(cam.get_camera_information().serial_number))

def settings(key, cam, runtime, mat):
    if key == 115:  # for 's' key
        switch_camera_settings()
    elif key == 43:  # for '+' key
        current_value = cam.get_camera_settings(camera_settings)
        cam.set_camera_settings(camera_settings, current_value + step_camera_settings)
        print(str_camera_settings + ": " + str(current_value + step_camera_settings))
    elif key == 45:  # for '-' key
        current_value = cam.get_camera_settings(camera_settings)
        if current_value >= 1:
            cam.set_camera_settings(camera_settings, current_value - step_camera_settings)
            print(str_camera_settings + ": " + str(current_value - step_camera_settings))
    elif key == 114:  # for 'r' key
        cam.set_camera_settings(sl.CAMERA_SETTINGS.CAMERA_SETTINGS_BRIGHTNESS, -1, True)
        cam.set_camera_settings(sl.CAMERA_SETTINGS.CAMERA_SETTINGS_CONTRAST, -1, True)
        cam.set_camera_settings(sl.CAMERA_SETTINGS.CAMERA_SETTINGS_HUE, -1, True)
        cam.set_camera_settings(sl.CAMERA_SETTINGS.CAMERA_SETTINGS_SATURATION, -1, True)
        cam.set_camera_settings(sl.CAMERA_SETTINGS.CAMERA_SETTINGS_GAIN, -1, True)
        cam.set_camera_settings(sl.CAMERA_SETTINGS.CAMERA_SETTINGS_EXPOSURE, -1, True)
        cam.set_camera_settings(sl.CAMERA_SETTINGS.CAMERA_SETTINGS_WHITEBALANCE, -1, True)
        print("Camera settings: reset")
    elif key == 122:  # for 'z' key
        record(cam, runtime, mat)

if __name__ == '__main__':
    main()
