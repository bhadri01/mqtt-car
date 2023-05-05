from picamera import PiCamera
from flask import Flask, Response, render_template, send_file
import io

app = Flask(__name__)

camera = PiCamera()
camera.resolution = (240, 120)
camera.framerate = 20

last_frame = None

@app.route('/')
def index():
    return render_template('index.html')

def gen_frames():
    while True:
        stream = io.BytesIO()
        camera.capture(stream, 'jpeg', use_video_port=True)
        stream.seek(0)
        last_frame = stream.getvalue()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + last_frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/get_frame')
def get_frame():
    stream = io.BytesIO()
    camera.capture(stream, 'jpeg', use_video_port=True)
    stream.seek(0)
    last_frame = stream.getvalue()
    if last_frame is not None:
        return send_file(io.BytesIO(last_frame), mimetype='image/jpeg')
    else:
        return 'No frame captured yet'

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8000, threaded=True)
