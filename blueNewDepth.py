import sys
import cv2
import numpy as np
import ArducamDepthCamera as ac
import time
from depthWithApf import getDirection
import bluetooth


server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)

port = 0x1001

server_sock.bind(("", port))
server_sock.listen(1)
print("Listening...")
#uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ef"
#bluetooth.advertise_service(server_sock, "SampleServerL2CAP",
#                            service_id=uuid, service_classes = [uuid])
client_sock, address = server_sock.accept()
print("Accepted connection from", address)


print(dir(ac))

MAX_DISTANCE = 8
divNum = 8


def blur(disps):
    he, we = disps.shape
    yDim = int(he / divNum)
    xDim = int(we / divNum)
    newImX = np.empty((divNum, divNum), int)
    newImY = np.empty((divNum, divNum), int)
    for ye in range(0, divNum):
        yD = yDim * ye
        for xe in range(0, divNum):
            xD = xe * xDim
            #print(xD, yD)
            arrAlt = disps[yD:(yD + yDim), xD:(xD + xDim)]
            #print(arrAlt)
            currMean = int(np.mean(arrAlt))
            newImX[ye][xe] = currMean
            newImY[xe][ye] = currMean
    return newImX, newImY



def process_frame(depth_buf: np.ndarray, amplitude_buf: np.ndarray) -> np.ndarray:
        
    depth_buf = np.nan_to_num(depth_buf)

    amplitude_buf[amplitude_buf<=7] = 0
    amplitude_buf[amplitude_buf>7] = 255

    depth_buf = (1 - (depth_buf/MAX_DISTANCE)) * 255
    depth_buf = np.clip(depth_buf, 0, 255)
    result_frame = depth_buf.astype(np.uint8)  & amplitude_buf.astype(np.uint8)
    return result_frame 

class UserRect():
    def __init__(self) -> None:
        self.start_x = 0
        self.start_y = 0
        self.end_x = 0
        self.end_y = 0

selectRect = UserRect()

followRect = UserRect()

def on_mouse(event, x, y, flags, param):
    global selectRect,followRect
    
    if event == cv2.EVENT_LBUTTONDOWN:
        pass

    elif event == cv2.EVENT_LBUTTONUP:
        selectRect.start_x = x - 4 if x - 4 > 0 else 0
        selectRect.start_y = y - 4 if y - 4 > 0 else 0
        selectRect.end_x = x + 4 if x + 4 < 240 else 240
        selectRect.end_y=  y + 4 if y + 4 < 180 else 180
    else:
        followRect.start_x = x - 4 if x - 4 > 0 else 0
        followRect.start_y = y - 4 if y - 4 > 0 else 0
        followRect.end_x = x + 4 if x + 4 < 240 else 240
        followRect.end_y = y + 4 if y + 4 < 180 else 180
        
def usage(argv0):
    print("Usage: python "+argv0+" [options]")
    print("Available options are:")
    print(" -d        Choose the video to use")


if __name__ == "__main__":
    cam = ac.ArducamCamera()
    if cam.init(ac.TOFConnect.CSI,0) != 0 :
        print("initialization failed")
    if cam.start(ac.TOFOutput.DEPTH) != 0 :
        print("Failed to start camera")
    cam.setControl(ac.TOFControl.RANG,MAX_DISTANCE)
    cv2.namedWindow("preview", cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback("preview",on_mouse)
    tm0 = time.time()
    while True:
        frame = cam.requestFrame(200)
        if frame != None:
            depth_buf = frame.getDepthData()
            amplitude_buf = frame.getAmplitudeData()
            cam.releaseFrame(frame)
            amplitude_buf*=(1024/1024)
            amplitude_buf = np.clip(amplitude_buf, 0, 255)
            newAmp = 255 - np.uint8(amplitude_buf)
            he, we = newAmp.shape
            newAmp = newAmp[34:he, 0:we]

            cv2.imshow("preview_amplitude", newAmp)
            #bl = blur(redIm)
            #print(bl[0])
            dirs = getDirection(newAmp)
            tm1 = time.time()
            tmPrev = tm1 - tm0
            print(str(dirs[3]))
            print(tmPrev)
            client_sock.send(str(dirs[3]).encode())
            tm0 = tm1
            key = cv2.waitKey(1)
            if key == ord("q"):
                exit_ = True
                cam.stop()
                sys.exit(0)
                break
client_sock.close()
server_sock.close()
