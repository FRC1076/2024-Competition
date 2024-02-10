from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter
from pycoral.utils.edgetpu import run_inference
from networktables import NetworkTables
import gstreamer

"""To Do:
Find where to put run_inference
Find where to input video
Find where to do interpreter.invoke (if needed)
Use 'try' to prevent crashing/use networktables to send ready signal"""
#see if I need a server (put this in main?)
NetworkTables.initialize()
notePub = NetworkTables.getTable('noteDetector')

def getClosestNote(objs):
    objs.sort(key=lambda obj: obj.bbox.area, reverse=True)
    return objs[0]

def publishBBox(bbox):
    notePub.putString('testKey', "Hello world")
    notePub.putNumber('xmin', bbox.xmin)
    notePub.putNumber('xmax', bbox.xmax)
    notePub.putNumber('ymin', bbox.ymin)
    notePub.putNumber('ymax', bbox.ymax)
    return

def main():
    model = "~/notedetector29/edgetpu.tflite"
    source = "/dev/video1:YUY2:800x600:20/1"
    source_size
    source_format
    threshold = "0.75"

    interpreter = make_interpreter(model)
    interpreter.allocate_tensors()
    inference_size = input_size(interpreter)

    def user_callback(input_tensor, src_size, inference_box):
        run_inference(interpreter, input_tensor)

        objs = get_objects(interpreter, threshold)
        publishBBox(getClosestNote(objs))

    gstreamer.run_pipeline(user_callback, src_size=source_size, appsink_size=inference_size, videosrc=source, videofmt=source_format)
    return

if __name__ == '__main__':
    main()

