import cv2

def testDevice(source):
    cap = cv2.VideoCapture(source) 
    if cap is None or not cap.isOpened():
        pass
    else:
        print("available source:", source)

if __name__ == '__main__':


    for cam_idx in range(100):
        testDevice(cam_idx)
    
    # cam idx lst: [0, 2, 8, 10, 16]:
    # cam_idx = 0
    # cap = cv2.VideoCapture(cam_idx)
    # while True:
    #     ret, frame = cap.read()
    #     cv2.imshow(f'frame_{cam_idx}',frame)
    #     if cv2.waitKey(1) & 0xFF == ord('q'):
    #         break

    # cap.release()
    # cv2.destroyAllWindows()
