from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter
from pycoral.utils.edgetpu import run_inference
from networktables import NetworkTables
import numpy as np
import cv2

"""To Do:
Find where to put run_inference
Find where to input video
Find where to do interpreter.invoke (if needed)
Use 'try' to prevent crashing/use networktables to send ready signal"""
#see if I need a server (put this in main?)
NetworkTables.initialize()
notePub = NetworkTables.getTable('noteDetector')

def getClosestNote(objs):
    #print('getClosestNote called')
    if objs:
        #print('objs exist')
        objs.sort(key=lambda obj: obj.bbox.area, reverse=True)
        return objs[0]
    else:
        return False

def publishBBox(obj):
    if obj:
        notePub.putString('testKey', "Hello world")
        notePub.putBoolean('hasTarget', True)
        notePub.putNumber('xmin', obj.bbox.xmin)
        notePub.putNumber('xmax', obj.bbox.xmax)
        notePub.putNumber('ymin', obj.bbox.ymin)
        notePub.putNumber('ymax', obj.bbox.ymax)
        #print('hasTarget')
        print('target at %f' % obj.bbox.xmin)
    else:
        notePub.putBoolean('hasTarget', False)
        print('no target detected')
    return

def main():
    print("main called")

    model = "/home/mendel/notedetector29/edgetpu.tflite"
    source = "/dev/video1"
    source_size = (800, 600)
    source_format = "raw"
    threshold = 0.75

    interpreter = make_interpreter(model)
    interpreter.allocate_tensors()
    inference_size = input_size(interpreter)

    cap = cv2.VideoCapture(1)
    while cap.isOpened():
        #print("cap opened")
        ret, frame = cap.read()
        if not ret:
            print('frame not received')
            break

        cv2_im = frame
        cv2_im_rgb = cv2.cvtColor(cv2_im, cv2.COLOR_BGR2RGB)
        cv2_im_rgb = cv2.resize(cv2_im_rgb, inference_size)
        run_inference(interpreter, cv2_im_rgb.tobytes())
        objs = get_objects(interpreter, threshold)

        publishBBox(getClosestNote(objs))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    return

if __name__ == '__main__':
    main()

