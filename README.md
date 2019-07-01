# Camera_MJPEG
MJPEG CAMERA on UART interface (scheme and programs)

The project of a video camera from a camera OV2640 and a microcontroller STM32F407VGT6.
![Image alt](https://github.com/fademike/Camera_MJPEG/blob/master/IMG_20190628_132948_2.jpg)

You can transfer images from a microcontroller via UART, Ethernet or via a Wireless connection. Or write to the microSD memory card.
The image 320x240 is stable and does not float. In another case, it can be washed away and float. Therefore, large sizes are suitable only for photos.

In repository:
- COMReadJPEG - Project Builder C++ (Receives pictures via UART and displays. Changes camera settings and saves images.)
- sch+pcb - Scheme and Board (Altium project)
- stm32f407_ov2640 - Project SW4STM32 for stm32 microcontroller.
- Video_2019-06-27_172429.wmv - short video

My debug layout
![Image alt](https://github.com/fademike/Camera_MJPEG/blob/master/IMG_20190628_094840_2.jpg)


