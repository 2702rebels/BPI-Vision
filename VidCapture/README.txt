The files in this folder provide an example of how to capture
images directly from the /dev/video system.  Exposure times for
the images are managed by a camera thread which does very little
other than managing the camera and distributing video frames to
worker threads.

In this case the sensor is an MT9V034 which returns a raw
Bayer Image.  Algorithms to convert from the Bayer Raw to
scaled colour and grey-scale (based on green pixels) are
included.

The processed images from 2 cameras is displayed on the
frame-buffer device (an HDMI monitor is needed).

Code to create a "camera feed" that can be displayed on the
FRC Smart-dashboard is also included, but is commented out.

