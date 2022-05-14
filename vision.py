import numpy as np
import cv2
import matplotlib.pyplot as plt
import imutils
import serial
import time
from scipy import interpolate


#from cvforbounce2 import undistort_
#from cvforbounce2 import undistort_ as fish
#import serial


# cap = cv2.VideoCapture('1.mp4')  # 打开视频文件
cap = cv2.VideoCapture(1)#打开USB摄像头
cap.set(10,cap.get(10)-20)
cap.set(11,125)
board = [50, 442, 63, 457]
kernel = np.ones((5, 5), np.uint8)  # 卷积核
X = [3]
Z = [3]
h = 65.5
ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM6'
print(ser)
ser.open()
print(ser.is_open)
z = 33
flag = 0
point = 0
check = 0
if (cap.isOpened()):  # 视频打开成功
    flag = 1
else:
    flag = 0
num = 0
PTS = []
MIDX = 0
MIDY = 0
WARPED = 0
ground_r = 25
r_list = []
bounce = 0

DIM=(640, 480)
K=np.array([[364.3073757627725, 0.0, 300.3725403625572], [0.0, 364.398742267465, 226.1662111647935], [0.0, 0.0, 1.0]])
D=np.array([[0.009071582169233611], [-0.12615190177005284], [0.19432095862174534], [-0.1119595657100262]])
def CalculateZ(r):
    H =  h/((r/ground_r)+1)
    return H


def undistort_(img):
    #img = cv2.imread(img_path)
    img = cv2.resize(img, DIM)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM,cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)    
    #cv2.imwrite('unfisheyeImage.png', undistorted_img)
    #cv2.imshow('uf',undistorted_img)
    #cv2.waitKey(5)
    return undistorted_img

def order_points(pts):
    # 初始化矩形4个顶点的坐标
    rect = np.zeros((4, 2), dtype='float32')
    # 坐标点求和 x+y
    s = pts.sum(axis = 1)
    # np.argmin(s) 返回最小值在s中的序号
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    # diff就是后一个元素减去前一个元素  y-x
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    # 返回矩形有序的4个坐标点
    return rect

def perTran(image, pts):
    rect = order_points(pts)
    tl, tr, br, bl = rect
    # 计算宽度
    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
    maxWidth = max(int(widthA), int(widthB))
    # 计算高度
    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
    maxHeight = max(int(heightA), int(heightB))
    # 定义变换后新图像的尺寸
    dst = np.array([[0, 0], [maxWidth-1, 0], [maxWidth-1, maxHeight-1],
                   [0, maxHeight-1]], dtype='float32')
    # 变换矩阵
    M = cv2.getPerspectiveTransform(rect, dst)
    # 透视变换
    warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
    return warped

print(flag)
if (flag):
    while (True):
        if len(r_list)>10000:
            r_list = []
        bounce = 0
        now =  time.time()
        ret, frame = cap.read()  # 读取一帧
        frame = frame[board[0]:board[1], board[2]:board[3]]
        #frame = cv2.resize(frame,None,fx=0.75, fy=0.75, interpolation = cv2.INTER_CUBIC)

        #frame = undistort_(frame)
        # print("now")
        GrayImage=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        GrayImage= cv2.medianBlur(GrayImage,3)
        #GrayImage = cv2.cv2.bilateralFilter(gray, 11, 17, 17)
        #cv2.imshow('t',GrayImage)
        #cv2.waitKey(5)

        image = frame
        output = image.copy()
        # 转换成灰度图像
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # 双边滤波器能够做到平滑去噪的同时还能够很好的保存边缘
        gray = cv2.bilateralFilter(gray, 11, 17, 17)
        # 检测边缘

        edged = cv2.Canny(gray, 50, 200)
        cv2.imshow('Canny', edged)
        # 查找轮廓
        cnts = cv2.findContours(edged.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        # 获取前3个最大的轮廓
        #print(cnts)
        cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:3]
        
        screenCnt = None
        check = 1
        if check:
            for c in cnts:
                # 轮廓周长
                peri = cv2.arcLength(c, True)
                # print('arcLength : {:.3f}'.format(peri))
                # approxPolyDP主要功能是把一个连续光滑曲线折线化，对图像轮廓点进行多边形拟合。
                # 近似轮廓的多边形曲线， 近似精度为轮廓周长的1.5%
                approx = cv2.approxPolyDP(c, 0.015 * peri, True)
                # 矩形边框具有4个点， 将其他的剔除
                if len(approx) == 4:
                    screenCnt = approx
                    break
                # print(screenCnt)
            # 绘制轮廓矩形边框
            if screenCnt is not None and peri>1000:
                #print(screenCnt)
                pts = screenCnt.reshape(4, 2)
                PTS =pts

                warped = perTran(output, pts)
                warped_G = perTran(GrayImage, pts)
                WARPED = 1
                #print(pts)
                cv2.drawContours(image, [screenCnt], -1, (0, 255, 0), 3)
                warped_G2 = cv2.resize(warped_G,dsize = (300,300))
                cv2.imshow('cor',warped_G2)
                cv2.waitKey(5)

       
        circles = cv2.HoughCircles(GrayImage,cv2.HOUGH_GRADIENT,1,300,param1=55,param2=50,minRadius=20,maxRadius=60)
        # if WARPED :
        #     circles_2 = cv2.HoughCircles(warped_G2,cv2.HOUGH_GRADIENT,1,300,param1=35,param2=35,minRadius=15,maxRadius=35)
        # if circles_2 is not None:
        #     circle_2 = circles_2[0,:,:]

        if circles is not None:

            circles = circles[0, :, :]
            circles = np.uint16(np.around(circles))
            if len(PTS)>0:
                #print('T1:',(PTS[0][0]+PTS[2][0])/2)
                #print(PTS[1])
                #print('T2:',(PTS[0][1]+PTS[2][1])/2)
                #print(PTS[3])
                MIDX = (PTS[0][0]+PTS[2][0])/2
                MIDY = (PTS[0][1]+PTS[2][1])/2

            for i in circles[:1]:
            # draw the outer circle
               cv2.circle(frame, (i[0], i[1]), i[2], (0, 255, 0), 2)
               r_list.append(i[2])
               s_r_list = sorted(r_list)
               ground_r = np.mean(s_r_list[:3])
               #print(s_r_list[:3])
               #print(i[2])
               if len(r_list)>1:
                   if(r_list[-1]< r_list[-2]):
                       bounce = 1
                   # if(r_list[-1]> r_list[-2]):
                   #     point+= 1
                   #     z = CalculateZ(i[2])
                   #     x = np.sqrt(i[0]**2+i[1]**2)
                   #     Z.append(z)
                   #     X.append(x)

               # if point == 3:
               #      f = interpolate.interp1d(X,Z,kind="quadratic")
               #      plt.plot(X,Z)
               #      plt.show()
               #      X = {0}
               #      Z = {0}

                    # point = 0
               #print('dx:',i[0]-MIDX)
               #print('dy:',i[1]-MIDY)
               cv2.circle(frame, (int(MIDX), int(MIDY)), 5, (255, 0, 0), cv2.FILLED)
            # draw the center of the circle
               cv2.circle(frame, (i[0], i[1]), 2, (0, 255, 0), 3)

               text = 'dx='+ str(i[0]-MIDX) + ' dy=' + str(i[1]-MIDY)
               text2 = 'mx='+ str(MIDX) + ' my=' + str(MIDY)
               text3 = 'bx=' + str(i[0]) + ' by=' + str(i[1])
               cv2.putText(frame, text, (50, 100), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 2)
               cv2.putText(frame, text2, (50, 70), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 2)
               cv2.putText(frame, text3, (50, 30), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 2)
               #ser.write(str(i[0]).encode())
               st = str(i[0])
               #print('r1:',st)
               st2 = str(i[1])
               stz = str(i[2])
               stMX = str(int(MIDX))
               stMY = str(int(MIDY))
               ser.write(st.encode())
               ser.write('S'.encode())
               ser.write(st2.encode())
               ser.write('T'.encode())
               ser.write(stz.encode())
               ser.write('Z'.encode())



               dr = np.int16(z -np.int16(i[2]))
               ds = 0
               if dr<0:
                   ds = 2
               if dr>0:
                   ds = 1


               #print(i[0],i[1],i[2],dr,ds)
               z = i[2]
               dr = str(bounce)
               ser.write(dr.encode())
               ser.write('D'.encode())

               ser.write(stMX.encode())
               ser.write('X'.encode())
               ser.write(stMY.encode())
               ser.write('Y'.encode())
            
            cv2.imshow('detected circles',frame)
        # print(ground_r,bounce)
        print('time:', time.time() - now)
        cv2.imshow('video',frame)
        if cv2.waitKey(20)==ord("q"):
            break


cv2.waitKey(0)
cv2.destroyAllWindows()

def CalculateZ(r):
    H =  0
    H =  h/((r/ground_r)+1)
    return H
