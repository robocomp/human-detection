# Make sure to have the server side running in V-REP: 
# in a child script of a V-REP scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

import vrep
import torch
from openpifpaf.network import nets
from openpifpaf import decoder, show, transforms
import time
import cv2
import numpy as np
import sys
import argparse
import matplotlib.pyplot as plt
import PIL

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
    parser.add_argument('--scale', default=0.5, type=float,
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

    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',20000,True,True,1300,5) # Connect to V-REP
    if clientID == -1:
        sys.exit()
    print ('Connected to remote API server')

    res, camhandle = vrep.simxGetObjectHandle(clientID, 'camara_1', vrep.simx_opmode_oneshot_wait)
    print(res)
    res, resolution, image = vrep.simxGetVisionSensorImage(clientID, camhandle, 0, vrep.simx_opmode_streaming)
    
    ##############

    args = cli()

    # load model
    model, _ = nets.factory_from_args(args)
    model = model.to(args.device)
    processor = decoder.factory_from_args(args, model)

    visualizer = None
    while True:
        res, resolution, image = vrep.simxGetVisionSensorImage(clientID, camhandle, 0, vrep.simx_opmode_buffer)
        if len(image)==0:
            continue
        img = np.array(image, dtype = np.uint8)
        img.resize([resolution[1], resolution[0], 3])
        img = np.rot90(img,2)
        img = np.fliplr(img)
        cv2.imshow('t', img)
        cv2.waitKey(1)
        image = cv2.resize(img, None, fx=args.scale, fy=args.scale)
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

    vrep.simxFinish(clientID)

if __name__ == '__main__':
    main()
