# streaming the camera feed to make debugging easyer
# using flask and Marci's server and domain

from flask import Flask, Response
import cv2
import threading
import time
import os

app = Flask(__name__)

frames = {}

def generate(frame):
    "Encode and yield frames for streaming"
    while True:
        _, buffer = cv2.imencode('.jpg', frames[frame])
        time.sleep(0.02)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/video_feed/<frame>')
def video_feed(frame):
    "Route to stream video feed"
    if frame in frames:
        return Response(generate(frame),
                        mimetype='multipart/x-mixed-replace; boundary=frame')
    return "Camera not found", 404

def init():
    flask_thread = threading.Thread(target=app.run, kwargs={"host": "0.0.0.0", "port": 5000, "threaded": True}, daemon=True)
    flask_thread.start()

def show(name, frame):
    "Register a frame for streaming, processing loops are expected to call this for each frame"
    frames[name] = frame