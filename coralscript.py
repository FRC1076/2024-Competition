from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter
from pycoral.utils.edgetpu import run_inference
from networktables import NetworkTables
import numpy as np
import cv2

# coral IP is 10.10.76.16

config = {
    "MODEL": "/home/mendel/notedetector29/edgetpu.tflite", # absolute path of the tflite model in the DevBoard
    "SOURCE_ID": 1, # default USB camera source ID is often 0, but the DevBoard likes 1
    "CONFIDENCE_THRESHOLD": 0.75, # minimum confidence threshold
    "NETWORKTABLES_IP": '10.10.76.2',
}

NetworkTables.initialize(server=config["NETWORKTABLES_IP"]) # NetworkTables server IP must be the same on the coral and the robot
notePub = NetworkTables.getTable('noteDetector')

def getClosestNote(objs):
    #print('getClosestNote called')
    if objs:
        # returns the bounding box with the greatest area
        objs.sort(key=lambda obj: obj.bbox.area, reverse=True)
        return objs[0]
    else:
        return False

def publishBBox(obj):
    notePub.putBoolean('isAlive', True)
    if obj:
        notePub.putBoolean('hasTarget', True)
        notePub.putNumber('xmin', obj.bbox.xmin)
        notePub.putNumber('xmax', obj.bbox.xmax)
        notePub.putNumber('ymin', obj.bbox.ymin)
        notePub.putNumber('ymax', obj.bbox.ymax)

        message = 'target at ({}, {})'
        print(message.format(((obj.bbox.xmin + obj.bbox.xmax)/2), ((obj.bbox.ymin + obj.bbox.ymax)/2)))
    else:
        notePub.putBoolean('hasTarget', False)
        print('no target detected')
    return

def main():
    print("main called")

    interpreter = make_interpreter(config["MODEL"])
    interpreter.allocate_tensors()
    inference_size = input_size(interpreter)
    print(inference_size)

    try:   
        cap = cv2.VideoCapture(config["SOURCE_ID"])
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
            objs = get_objects(interpreter, config["CONFIDENCE_THRESHOLD"])
            publishBBox(getClosestNote(objs))

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except cv2.error as error:
        print("[Error]: {}".format(error))
        notePub.putBoolean('isAlive', False)
        notePub.putBoolean('hasTarget', False)
        raise
    except Exception as exc:
        print("[Exception]: {}".format(exc))
        notePub.putBoolean('isAlive', False)
        notePub.putBoolean('hasTarget', False)
        raise

    cap.release()
    cv2.destroyAllWindows()
    return

if __name__ == '__main__':
    main()

