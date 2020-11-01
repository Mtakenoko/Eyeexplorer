import os
import glob
import matplotlib
import cv2
import numpy as np

class Test(object):
    def __init__(self):
        self.load_model()
        
    def load_model(self):
        # Keras / TensorFlow
        os.environ['TF_CPP_MIN_LOG_LEVEL'] = '5'
        from keras.models import load_model
        from layers import BilinearUpSampling2D

        # Custom object needed for inference and training
        custom_objects = {'BilinearUpSampling2D': BilinearUpSampling2D, 'depth_loss_function': None}

        print('Loading model...')

        # Load model into GPU / CPU
        self.model = load_model('/home/takeyama/workspace/ros2_eyeexplorer/src/depth_predict/densedepth/eye_data.h5', custom_objects=custom_objects, compile=False)

        print('\nModel loaded ({0}).'.format('eye_data.h5'))

    def load_test_images(self):
        from utils import load_images
        self.inputs = load_images(glob.glob('/home/takeyama/workspace/ros2_eyeexplorer/src/depth_predict/densedepth/examples/eye/*.jpg'))
        print('\nLoaded ({0}) images of size {1}.'.format(self.inputs.shape[0], self.inputs.shape[1:]))

    def input_image(self, cvimage):
        from utils import load_cvimage
        self.inputs = load_cvimage(cvimage)
        print('\nLoaded ({0}) images of size {1}.'.format(self.inputs.shape[0], self.inputs.shape[1:]))

    def depth_predict(self):
        from utils import predict
        # Compute results
        self.outputs = predict(self.model, self.inputs, minDepth=0.0, maxDepth=25.0, batch_size=4)
        return self.outputs

    def display_images(self, outputs, is_rescale=False):
        import matplotlib.pyplot as plt
        import skimage
        from skimage.transform import resize

        plasma = plt.get_cmap('plasma')
        
        imgs = []

        for i in range(outputs.shape[0]):
            rescaled = outputs[i][:,:,0]
            if is_rescale:
                rescaled = rescaled - np.min(rescaled)
                rescaled = rescaled / np.max(rescaled)
            imgs.append(plasma(rescaled)[:,:,:3])

        imgs = np.stack(imgs)
        image_uint = imgs.astype(np.uint8)[0]
        imgs_gray = cv2.cvtColor(image_uint, cv2.COLOR_BGR2GRAY)
        plasma_image = cv2.applyColorMap(imgs_gray, cv2.COLORMAP_PLASMA)