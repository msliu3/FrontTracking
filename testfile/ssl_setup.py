import pyaudio
import os,sys

GCC_LENG = 366
GCC_BIAS = 6
ACTION_SPACE = 8
CHUNK = 1024
RECORD_DEVICE_NAME = "USB Camera-B4.09.24.1"
RECORD_WIDTH = 2
CHANNELS = 4
RATE = 16000

RECORD_SECONDS = 1
FORMAT = pyaudio.paInt16

FORWARD_SECONDS = 3
STEP_SIZE = 1

pwd = os.path.abspath(os.path.abspath(__file__))
father_path = os.path.abspath(os.path.dirname(pwd) + os.path.sep + "..")
print(father_path)
sys.path.append(father_path)

MODEL_PATH = father_path + "/resource/model/save20.ckpt"
WAV_PATH = father_path + "/resource/wav/online"
ONLINE_MODEL_PATH = father_path + "/resource/model/online.ckpt"

# KWS parameters
KWS_WAVE_PATH = father_path + "/resource/stream_tmp"
KWS_MODEL_PATH = father_path + "/resource/Pretrained_models/DNN/follow.pb"
KWS_LABEL_PATH = father_path+"/resource/Pretrained_models/follow_labels.txt"

# sliding window size can be seen in KWS detector
