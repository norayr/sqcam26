
Argus DC-1730 camera linux kernel driver.

This driver works (tested) only with 2.6.26 kernel back in 2008. I have no access to the camera to test it with newer kernels.

I have used sqcam driver info about which you can find below.

More information about the work can be found [on my blog](http://norayr.arnet.am/weblog/2008/11/13/технические-шалости/) in Russian. Many years passed since 2008 and I don't remember why and what was done. Sharing this in hope it might be useful.


------

This is SQCam driver version 0.1b - 2004/03/16

Edit KERNEL_DIR variable in Makefile to reflect your 2.6 kernel dir.
Then just run make.
You will get sqcam.ko. Copy it to /lib/modules/`uname -r`/drivers/usb/media/
dir and run depmod -a. 
 Remember this driver is in development, however, camera unplugging and repluggind is now supported.
You might get a small error message when closing your webcam application (to bo fixed in future version of this driver).


Any suggestions or patches should be sent to:
  Wilfred <wln@aol.com> containing sqcam in the subject.

Enjoy. 
