## Improved "usb_sound_card" sample with feedback and mute function

Essentially same to the pull request at https://github.com/raspberrypi/pico-playground/pull/53
but you can download revised code directly here.

### Description

USB-audio feedback (USB Audio Streaming Isochronous Feedback) is indispensable for stable sound playback without clicking noise. 
I implemented a simple feedback function by monitoring the buffer's vacancy. 
To avoid modifying other sources (such as audio.cpp in pico-extras), the number of free buffers is checked directly within this source code.

USB audio also includes a mute function (mute without changing the volume value). 
I enhanced this functionality by adding just three lines of code, as the original source code was almost complete.

Many DIY USB-DAC enthusiasts have been eagerly anticipating the implementation of the feedback function. 
The modified parts are isolated using switches named USB_FEEDBACK and MUTE_CMD.
