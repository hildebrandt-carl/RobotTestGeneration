#!/usr/bin/env python

import csv
import cv2
import math
import os
import shlex
import subprocess
import tempfile
import time
import face_recognition
import numpy as np
import glob

import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing
from olympe.messages.ardrone3.Piloting import moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.messages.ardrone3.PilotingSettings import MaxTilt
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged

# NOTE: Line numbers of this example are referenced in the user guide.
# Don't forget to update the user guide after every modification of this example.


class StreamingExample:

    PHYSICAL_IP = "192.168.42.1"
    SIMULATED_IP = "10.202.0.1"

    def __init__(self):
        # Create the olympe.Drone object from its IP address
        # self.drone = olympe.Drone(
        #     "10.202.0.1",
        #     loglevel=3,
        # )

        # Load the faces
        self.known_face_encodings = []
        self.known_face_names = []

        
        os.chdir("./faces")
        for f in glob.glob("*.jpg"):
            print("Loading: " + str(f))
            img = face_recognition.load_image_file(f)
            img_encoding = face_recognition.face_encodings(img)[0]
            self.known_face_encodings.append(img_encoding)
            self.known_face_names.append(f[0:-4])


        self.drone = olympe.Drone(
            StreamingExample.PHYSICAL_IP,
            loglevel=3,
        )

        self.tempd = tempfile.mkdtemp(prefix="olympe_streaming_test_")
        print("Olympe streaming example output dir: {}".format(self.tempd))
        self.h264_frame_stats = []
        self.h264_stats_file = open(
            os.path.join(self.tempd, 'h264_stats.csv'), 'w+')
        self.h264_stats_writer = csv.DictWriter(
            self.h264_stats_file, ['fps', 'bitrate'])
        self.h264_stats_writer.writeheader()

    def start(self):
        # Connect the the drone
        self.drone.connection()

        # You can record the video stream from the drone if you plan to do some
        # post processing.
        self.drone.set_streaming_output_files(
            h264_data_file=os.path.join(self.tempd, 'h264_data.264'),
            h264_meta_file=os.path.join(self.tempd, 'h264_metadata.json'),
            # Here, we don't record the (huge) raw YUV video stream
            # raw_data_file=os.path.join(self.tempd,'raw_data.bin'),
            # raw_meta_file=os.path.join(self.tempd,'raw_metadata.json'),
        )

        # Setup your callback functions to do some live video processing
        self.drone.set_streaming_callbacks(
            raw_cb=self.yuv_frame_cb,
            h264_cb=self.h264_frame_cb
        )
        # Start video streaming
        self.drone.start_video_streaming()

    def stop(self):
        # Properly stop the video stream and disconnect
        self.drone.stop_video_streaming()
        self.drone.disconnection()
        self.h264_stats_file.close()

    def yuv_frame_cb(self, yuv_frame):
        """
        This function will be called by Olympe for each decoded YUV frame.

            :type yuv_frame: olympe.VideoFrame
        """
        # the VideoFrame.info() dictionary contains some useful informations
        # such as the video resolution
        info = yuv_frame.info()
        height, width = info["yuv"]["height"], info["yuv"]["width"]

        # convert pdraw YUV flag to OpenCV YUV flag
        cv2_cvt_color_flag = {
            olympe.PDRAW_YUV_FORMAT_I420: cv2.COLOR_YUV2BGR_I420,
            olympe.PDRAW_YUV_FORMAT_NV12: cv2.COLOR_YUV2BGR_NV12,
        }[info["yuv"]["format"]]

        # yuv_frame.as_ndarray() is a 2D numpy array with the proper "shape"
        # i.e (3 * height / 2, width) because it's a YUV I420 or NV12 frame

        # Use OpenCV to convert the yuv frame to RGB
        rgb_frame = cv2.cvtColor(yuv_frame.as_ndarray(), cv2_cvt_color_flag)

        # Do face recognition
        face_locations= face_recognition.face_locations(rgb_frame, number_of_times_to_upsample=0, model="cnn")
        face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)

        # Loop through each face in this frame of video
        for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
            # See if the face is a match for the known face(s)
            matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding)

            name = "Unknown"

            # If a match was found in known_face_encodings, just use the first one.
            # if True in matches:
            #     first_match_index = matches.index(True)
            #     name = known_face_names[first_match_index]

            # Or instead, use the known face with the smallest distance to the new face
            face_distances = face_recognition.face_distance(self.known_face_encodings, face_encoding)
            best_match_index = np.argmin(face_distances)
            if matches[best_match_index]:
                name = self.known_face_names[best_match_index]

            # Draw a box around the face
            cv2.rectangle(rgb_frame, (left, top), (right, bottom), (0, 0, 255), 2)

            # Draw a label with a name below the face
            cv2.rectangle(rgb_frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(rgb_frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

        # Use OpenCV to show this frame
        cv2.imshow("Olympe Streaming Example", rgb_frame)
        cv2.waitKey(1)  # please OpenCV for 1 ms...

    def h264_frame_cb(self, h264_frame):
        """
        This function will be called by Olympe for each new h264 frame.

            :type yuv_frame: olympe.VideoFrame
        """

        # Get a ctypes pointer and size for this h264 frame
        frame_pointer, frame_size = h264_frame.as_ctypes_pointer()

        # For this example we will just compute some basic video stream stats
        # (bitrate and FPS) but we could choose to resend it over an another
        # interface or to decode it with our preferred hardware decoder..

        # Compute some stats and dump them in a csv file
        info = h264_frame.info()
        frame_ts = info["ntp_raw_timestamp"]
        if not bool(info["h264"]["is_sync"]):
            if len(self.h264_frame_stats) > 0:
                while True:
                    start_ts, _ = self.h264_frame_stats[0]
                    if (start_ts + 1e6) < frame_ts:
                        self.h264_frame_stats.pop(0)
                    else:
                        break
            self.h264_frame_stats.append((frame_ts, frame_size))
            h264_fps = len(self.h264_frame_stats)
            h264_bitrate = (
                8 * sum(map(lambda t: t[1], self.h264_frame_stats)))
            self.h264_stats_writer.writerow(
                {'fps': h264_fps, 'bitrate': h264_bitrate})

    def postprocessing(self):
        # Convert the raw .264 file into an .mp4 file
        h264_filepath = os.path.join(self.tempd, 'h264_data.264')
        mp4_filepath = os.path.join(self.tempd, 'h264_data.mp4')
        subprocess.run(
            shlex.split('ffmpeg -i {} -c:v copy {}'.format(
                h264_filepath, mp4_filepath)),
            check=True
        )

        # Replay this MP4 video file using the default video viewer (VLC?)
        # subprocess.run(
        #     shlex.split('xdg-open {}'.format(mp4_filepath)),
        #     check=True
        # )


if __name__ == "__main__":
    streaming_example = StreamingExample()
    # Start the video stream
    streaming_example.start()
    # Perform some live video processing while the drone is flying
    time.sleep(30)
    # Stop the video stream
    streaming_example.stop()
    # Recorded video stream postprocessing
    streaming_example.postprocessing()

