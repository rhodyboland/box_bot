# Jetson Specific Setups

***The Jetson can be very tedious about setting up certain parts so I made this little guide mainly for myself.***

**Flash using SDK Manager. Ensure OpenCV selected and GStreamer selected in both instances. I think they are under the computer vision tab. Easiest is to just select all.**

To check install, show cv2 build information, look for Gstreamer. It should have a YES next to it.

```
python
import cv2
print(cv2.getBuildInformation())
```

Enable CSI Camera port, IMX219-A (RPI Camera V2.1)

```
sudo /opt/nvidia/jetson-io/jetson-io.py
```

Install v4l-utils and check camera is recognised

```
sudo apt update; sudo apt install -y v4l-utils
```

```
v4l2-ctl --list-devices
```

### Now it should work. Seems simple when its written out.

Some test code in python

```
import cv2

def main():
    # The sensor-mode here is set to 2, which may correspond to the 1920x1080 mode.
    # You might need to adjust this number if your camera uses a different index.
    gst_pipeline = (
        "nvarguscamerasrc sensor-mode=2 ! "
        "video/x-raw(memory:NVMM), width=1920, height=1080, format=NV12, framerate=30/1 ! "
        "nvvidconv flip-method=0 ! "
        "video/x-raw, format=BGRx ! videoconvert ! "
        "video/x-raw, format=BGR ! appsink"
    )
    
    cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break

        cv2.imshow("CSI Camera", frame)

        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
```



## DT Overlay for enabling GPIO Jetpack 6.2
***Follow this guide for more info about whats behind this - [JetsonHacks repository](https://github.com/JetsonHacks/jetson-orin-gpio-patch)***

**1. Compile**
```
dtc -I dts -O dtb -o gpio7-11-uart8-10.dtbo gpio7-11-uart8-10.dts
```
**2. Install**
```
sudo cp gpio7-11-uart8-10.dtbo /boot/
sudo /opt/nvidia/jetson-io/jetson-io.py    # choose “Custom for hardware?” → "GPIO7/11 + UART8/10"
sudo reboot
```
**3. Test**

User should be apart of gpio
Update Jetson.GPIO
```
sudo pip install --upgrade Jetson.GPIO
```

### Work around to permission errors and isaac launch
```
sudo chmod 666 /dev/ttyTHS1
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyACM0

```

```
cd $ISAAC_ROS_WS && ./src/isaac_ros_common/scripts/run_dev.sh
```