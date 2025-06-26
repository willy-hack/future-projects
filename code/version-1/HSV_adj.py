import cv2
import numpy as np
import pickle
import os  # 用于检查文件是否存在

def nothing(x):
    pass

# 创建用于显示的空白图像窗口
img2 = np.zeros((600, 1024, 3), np.uint8)  # 增大图像大小
cv2.namedWindow('image')
cv2.resizeWindow('image', 1024, 768)  # 增大窗口的尺寸

# 创建滑动条来调整HSV阈值
cv2.createTrackbar('H_low', 'image', 0, 255, nothing)
cv2.createTrackbar('H_high', 'image', 255, 255, nothing)
cv2.createTrackbar('S_low', 'image', 0, 255, nothing)
cv2.createTrackbar('S_high', 'image', 255, 255, nothing)
cv2.createTrackbar('V_low', 'image', 0, 255, nothing)
cv2.createTrackbar('V_high', 'image', 255, 255, nothing)

# 相机设置
imcap = cv2.VideoCapture(
    'nvarguscamerasrc ! video/x-raw(memory:NVMM), width=640, height=480, format=(string)NV12, framerate=(fraction)30/1 ! '
    'nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink', 
    cv2.CAP_GSTREAMER
)

imcap.set(cv2.CAP_PROP_BRIGHTNESS, 60)
imcap.set(3, 640)  # 宽度
imcap.set(4, 480)  # 高度
kernal = np.ones((5, 5))

# 颜色HSV值字典
hsv_values = {'Blue_final': None, 'Orange_final': None, 'Red': None, 'Green': None, 'Pink': None, 'qualifications_Blue': None, 'qualifications_Orange': None}

# 检查并加载之前保存的HSV值
if os.path.exists('hsv_values.pkl'):
    with open('hsv_values.pkl', 'rb') as f:
        hsv_values = pickle.load(f)

# 显示HSV值和操作指南
print("Loaded previously saved HSV values:", hsv_values)
def print_actions():
    print("\nKey Operation Instructions:")
    print("Press '1' to save the current HSV values as Blue_final")
    print("Press '2' to save the current HSV values as Orange_final")
    print("Press '3' to save the current HSV values as Red")
    print("Press '4' to save the current HSV values as Green")
    print("Press '5' to save the current HSV values as Pink")
    print("Press '6' to save the current HSV values as qualifications_Blue")
    print("Press '7' to save the current HSV values as qualifications_Orange")
    print("Press 'q' to save all HSV values and exit the program\n")

print_actions()

# 创建按钮窗口并调整尺寸
cv2.namedWindow('buttons')
buttons_img = np.zeros((500, 600, 3), np.uint8)  # 增大按钮窗口尺寸
button_size = (250, 80)  # 按钮大小 (宽度, 高度)

# 定义按钮字典和位置，增加水平间距以防止按钮连在一起
buttons = {
    'Blue_final': (20, 100),
    'Orange_final': (20, 220),
    'Red': (20, 340),
    'Green': (300, 100),  # 在x轴上移动，使其不与左侧按钮重叠
    'Pink': (300, 220),
    'qualifications_Blue': (300, 340),
    'qualifications_Orange': (300, 460),  # 增加x轴位置，确保三个列之间有间隔
    'Save & Quit': (20, 460)
}

# 绘制按钮
for text, pos in buttons.items():
    color = (255, 255, 255)  # 默认白色背景
    if "final" in text:
        color = (0, 255, 0)  # 最终保存为绿色按钮
    elif "qualifications" in text:
        color = (0, 0, 255)  # qualifications 相关按钮为红色

    cv2.rectangle(buttons_img, (pos[0], pos[1] - button_size[1]), (pos[0] + button_size[0], pos[1]), color, -1)
    cv2.putText(buttons_img, text, (pos[0] + 10, pos[1] - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2)

i = 0
while True:
    H_high = cv2.getTrackbarPos('H_high', 'image')
    H_low = cv2.getTrackbarPos('H_low', 'image')
    S_high = cv2.getTrackbarPos('S_high', 'image')
    S_low = cv2.getTrackbarPos('S_low', 'image')
    V_high = cv2.getTrackbarPos('V_high', 'image')
    V_low = cv2.getTrackbarPos('V_low', 'image')
    success, img = imcap.read()

    if not success:
        break
    elif i == 0:
        print_actions()
        i += 1

    hls = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hls_low = np.array([H_low, S_low, V_low])
    hls_high = np.array([H_high, S_high, V_high])

    mask = cv2.inRange(hls, hls_low, hls_high)
    mask = cv2.dilate(mask, kernal, iterations=1)
    res = cv2.bitwise_and(img, img, mask=mask)

    cv2.imshow('image', res)
    cv2.imshow('buttons', buttons_img)

    key = cv2.waitKey(30) & 0xFF

    # 处理键盘输入
    if key == ord('1'):
        hsv_values['Blue_final'] = ([H_low, S_low, V_low], [H_high, S_high, V_high])
        print("Saved Blue_final HSV values:", hsv_values['Blue_final'])
    elif key == ord('2'):
        hsv_values['Orange_final'] = ([H_low, S_low, V_low], [H_high, S_high, V_high])
        print("Saved Orange_final HSV values:", hsv_values['Orange_final'])
    elif key == ord('3'):
        hsv_values['Red'] = ([H_low, S_low, V_low], [H_high, S_high, V_high])
        print("Saved Red HSV values:", hsv_values['Red'])
    elif key == ord('4'):
        hsv_values['Green'] = ([H_low, S_low, V_low], [H_high, S_high, V_high])
        print("Saved Green HSV values:", hsv_values['Green'])
    elif key == ord('5'):
        hsv_values['Pink'] = ([H_low, S_low, V_low], [H_high, S_high, V_high])
        print("Saved Pink HSV values:", hsv_values['Pink'])
    elif key == ord('6'):
        hsv_values['qualifications_Blue'] = ([H_low, S_low, V_low], [H_high, S_high, V_high])
        print("Saved qualifications_Blue HSV values:", hsv_values['qualifications_Blue'])
    elif key == ord('7'):
        hsv_values['qualifications_Orange'] = ([H_low, S_low, V_low], [H_high, S_high, V_high])
        print("Saved qualifications_Orange HSV values:", hsv_values['qualifications_Orange'])
    elif key == ord('q'):
        with open('hsv_values.pkl', 'wb') as f:
            pickle.dump(hsv_values, f)
        print("Saved all HSV values to hsv_values.pkl")
        break

    # 处理鼠标输入
    def mouse_callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            for button, pos in buttons.items():
                if pos[0] <= x <= pos[0] + button_size[0] and pos[1] - button_size[1] <= y <= pos[1]:
                    if button == 'Save & Quit':
                        with open('hsv_values.pkl', 'wb') as f:
                            pickle.dump(hsv_values, f)
                        print("Saved all HSV values to hsv_values.pkl")
                        cv2.destroyAllWindows()
                        exit()
                    else:
                        hsv_values[button] = ([H_low, S_low, V_low], [H_high, S_high, V_high])
                        print(f"Saved {button} HSV values:", hsv_values[button])

    cv2.setMouseCallback('buttons', mouse_callback)

cv2.destroyAllWindows()
