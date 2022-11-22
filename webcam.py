import cv2

from dectect import detection

cam_feed = cv2.VideoCapture(0)

cam_feed.set(cv2.CAP_PROP_FRAME_WIDTH, 650)
cam_feed.set(cv2.CAP_PROP_FRAME_HEIGHT, 750)

while True:
    ret, frame = cam_feed.read()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    if ret == False:
        break
    else:
        detection(frame)
        cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


cam_feed.release()
cv2.destroyAllWindows()
