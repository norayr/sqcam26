/*
 * USB SQcam WebCam driver based on the ViCam driver written by Joe Burks, 
 * Christopher L Cheney, Pavel Machek, John Tyner, Monroe Williams.
 *
 * Thanks to Theodore Kilgore for his help.
 * Ported to kernel 2.6 by Wilfred Nelson
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * This source code is based heavily on the CPiA webcam driver which was
 * written by Peter Pregler, Scott J. Bertin and Johannes Erdfelt
 *
 * Portions of this code were also copied from usbvideo.c
 *
 * The bayer decoding functions are copied from libgphoto2
 * as is the model type determination (with modifications)
 *
 * Locking Policy and Rationale:
 * Some code in this driver can be called from the USB subsystem when a
 * device is unplugged. The USB subsystem appears to be holding a lock when
 * this happens. Other code in the driver calls parts of the USB subsystem
 * that appear to block on that same lock. Holding our own lock when making
 * those calls can therefore result in a deadlock.
 * 
 * We therefore adopt the policy of trying not to hold our own lock when
 * calling external routines and enforce that policy strictly when the
 * external routine is part of the USB subsystem. Because kernel locks are
 * not recursive (you can't down() a lock that is already down) routines
 * that make USB calls must know if they are going to be called with the lock
 * held. A precondition is therefore declared for all such functions.
 *
 * Disconnect is careful not to do anything which could interfere with an in
 * progress I/O operation if is_open is true so usually it is safe to just
 * check if udev is non NULL and cache the value before releasing our lock
 * then reaquire the lock and check it is still non NULL.
 *
 * All functions ensure that the lock state on exit is the same as on entry
 * even if an error exit has been taken. The only exception to this is if
 * down_interruptible() fails. In that case the unnecessary up() should not
 * cause any harm and as the call is aborting due to getting EINTR it should
 * not need to hold the lock.
 *
 */

#include <linux/autoconf.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/videodev.h>
#include <linux/usb.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <media/v4l2-dev.h>

// #define SQCAM_DEBUG

#ifndef MODULE_LICENSE
#define MODULE_LICENSE(a)
#endif

#ifndef bool
#define bool int
#endif

#ifdef SQCAM_DEBUG
#define ADBG(lineno,fmt,args...) printk(fmt, jiffies, __FUNCTION__, lineno, ##args)
#define DBG(fmt,args...) ADBG((__LINE__),KERN_DEBUG __FILE__"(%ld):%s (%d):"fmt,##args)
#else
#define DBG(fmn,args...) do {} while(0)
#endif

/* Version Information */
#define DRIVER_VERSION "v1.0"
#define DRIVER_AUTHOR "Marcell Lengyel, miketkf@mailbox.hu"
#define DRIVER_DESC "SQcam WebCam Driver"

/* Define these values to match your device */
#define USB_SQCAM_VENDOR_ID	0x2770
#define USB_SQCAM_PRODUCT_ID	0x913c

#define SQCAM_BYTES_PER_PIXEL 3
#define SQCAM_MAX_READ_SIZE 0x8000
#define SQCAM_MAX_RAW_SIZE 320*240+64
#define SQCAM_MAX_FRAME_SIZE (SQCAM_BYTES_PER_PIXEL*320*240)
#define SQCAM_FRAMES 2
#define SHARPNESS_MIN	0
#define SHARPNESS_MAX	6
#define FRAMERATE_MIN	0
#define FRAMERATE_MAX	6

#define MAX_SQCAM 4

enum SQModel {
	SQ_MODEL_DEFAULT,
	SQ_MODEL_POCK_CAM,
	SQ_MODEL_MAGPIX
};


struct sqcam_camera {

	u8 *raw_image;		// raw data captured from the camera
	u8 *framebuf;		// processed data in BGR24 format
	struct video_device vdev;	// v4l video device
	struct usb_device *udev;	// usb device

	struct semaphore busy_lock;	// guard against SMP multithreading

        int channel;
	bool is_initialized;
	bool is_removed ;
	bool is_opened ;
	u8 bulkEndpoint;
        double gamma;
	u32 framebuf_size;	// # of valid bytes in framebuf
	u32 framebuf_read_start;	// position in frame buf that a read is happening at.

	enum SQModel model;

};

#include "gamma.h"

#define	SQCAM_T(uvd)	((struct sqcam_camera *)((uvd)->user_data))

static int sqcam_probe( struct usb_interface *intf, const struct usb_device_id *id);
static void sqcam_disconnect(struct usb_interface *intf);
static void read_frame(struct sqcam_camera *cam, int framenum);

static int gp_bayer_decode (unsigned char *input, int w, int h,
			     unsigned char *output, int tile);

static void usbvideo_rvfree(void *mem, unsigned long size);
static void *usbvideo_rvmalloc(unsigned long size);
 unsigned long usbvideo_kvirt_to_pa(unsigned long adr);

static int gamma_fill_table     (unsigned char *table, double g);
static int gamma_correct_single (unsigned char *table, unsigned char *data, 
			  unsigned int data_size);

			  
/*static int flags  = FLAGS_DISPLAY_HINTS | FLAGS_OVERLAY_STATS; */
static const int min_canvasWidth  = 8;
static const int min_canvasHeight = 4;

/*
 * Precondition: cam->busy_lock should be held when calling
 */
static int
send_control_msg(int set, struct sqcam_camera *cam, u8 request, u16 value,
		 u16 index, unsigned char *cp, u16 size)
{
	int status;

	// for reasons not yet known to me, you can't send USB control messages
	// with data in the module (if you are compiled as a module).  Whatever
	// the reason, copying it to memory allocated as kernel memory then
	// doing the usb control message fixes the problem.

	struct usb_device *udev = cam->udev;
	unsigned char *transfer_buffer = kmalloc(size, GFP_KERNEL);
	    memcpy(transfer_buffer, cp, size);
	
	if (udev == NULL)
		return -ENODEV;
	up(&cam->busy_lock);

	status = usb_control_msg(udev,
				 set ? usb_sndctrlpipe(udev, 0) : usb_rcvctrlpipe(udev, 0),
				 request,
				 (set ? USB_DIR_OUT : USB_DIR_IN) | USB_TYPE_VENDOR |
				 USB_RECIP_DEVICE, value, index,
				 transfer_buffer, size, HZ);

	kfree(transfer_buffer);

	DBG("ctrl msg:: value: %0x, index: %0x, size: %i\n", value, index, size);
	status = min(status, 0);
	if (status < 0) {
		printk(KERN_ERR "send_msg: Failed sending control message, error %d.\n",
		       status);
	printk(KERN_ERR "ctrl msg:: dir: %i, value: %0x, index: %0x, size: %i\n", set, value, index,  size);
	}

	if (down_interruptible(&cam->busy_lock))
		return -EINTR;

	return status;
}

/*
 * precondition: should be called with cam->busy_lock held
 */
static int
initialize_camera(struct sqcam_camera *cam)
{
	int status;
	unsigned char ch[5], zh;
	unsigned char model[4];
	int act_len;
	
	/*
000001	CS        80 06 00 01 00 00 12 00	

	C<  0000:  1201 1001 ffff ff08 7027 2091 0001 0002  ........p' .....
	    0010:  0001                                     ..

000002	CS        80 06 00 02 00 00 40 00	

	C<  0000:  0902 2700 0101 0080 fa09 0400 0003 ffff  ..'.............
	    0010:  ff00 0705 8102 4000 0007 0502 0240 0000  ......@......@..
	    0020:  0705 8303 0100 03                        .......
	
	
	
	*/
	

	/* Request ID */
	zh = 0x00;

	if ((status =
	     send_control_msg(1, cam, 0x0c, 0x06, 0xf0, &zh, 1)) < 0)
		{
		printk(KERN_ERR "initalize: message 1, error %d.\n",status);
		return status;
		}
	zh = 0x00;
	if ((status =
	     send_control_msg(0, cam, 0x0c, 0x07, 0, &zh, 1)) < 0)
		{
		printk(KERN_ERR "initalize: message 1a, error %d.\n",status);
		return status;
		}
	zh = 0x00;
	if ((status = send_control_msg(1, cam, 0x0c, 0x03, 4, &zh, 1)) < 0)
		 {
		 printk(KERN_ERR "initalize: message 1b, error %d.\n",status);
		 return status;
		 }
	if ((status = usb_bulk_msg(cam->udev,
				usb_rcvbulkpipe(cam->udev, cam->bulkEndpoint),
				model,4,&act_len,50)) < 0)
		 {
		 printk(KERN_ERR "initalize: message 1c, error %d.\n",status);
		 return status;
		 }
	if (memcmp (model, "\x09\x05\x01\x19", 4) == 0) {
		cam->model = SQ_MODEL_POCK_CAM;
	} else if (memcmp (model, "\x09\x05\x01\x32", 4) == 0) {
		cam->model = SQ_MODEL_MAGPIX;
	} else {
		cam->model = SQ_MODEL_DEFAULT;
	}
	printk(KERN_INFO "SQcam model code is %02x %02x %02x %02x, matched %d\n",
		model[0],model[1],model[2],model[3],cam->model);


	zh = 0x00;
	if ((status = send_control_msg(0, cam, 0x0c, 0x07, 0, &zh, 1)) < 0)
		{
		printk(KERN_ERR "initalize: message 2, error %d.\n",status);
		return status;
		}
	
/*

000005	CS        c0 0c 07 00 00 00 01 00	

	C<  0000:  09                                       .

000006	CS        c0 0c 07 00 00 00 01 00	

	C<  0000:  05                                       .

000007	CS        c0 0c 07 00 00 00 01 00	

	C<  0000:  00                                       .

000008	CS        c0 0c 07 00 00 00 01 00	

	C<  0000:  01                                       .

000009	CS        40 0c 06 00 a0 00 01 00	

	C>  0000:  50                                       P

000010	CS        c0 0c 07 00 00 00 01 00	

	C<  0000:  50                                       P


*/	
	
	ch[0] = 0x00;
	if ((status =
	     send_control_msg(0,cam, 0x0c, 0x07, 0, &ch[0], 4)) < 0)
		{
		printk(KERN_ERR "initalize: message 3, error %d.\n",status);
		return status;
		}
	zh = 0x00;
	if ((status =
	     send_control_msg(1, cam, 0x0c, 0x06, 0xa0, &zh, 1)) < 0)
		{
		printk(KERN_ERR "initalize: message 4, error %d.\n",status);
		return status;
		}
	zh = 0x00;
	if ((status =
	     send_control_msg(0, cam, 0x0c, 0x07, 0, &zh, 1)) < 0)
		{
		printk(KERN_ERR "initalize: message, error %d.\n",status);
		return status;
		}
	cam->channel = 0;
	return 0;
}

/*
 * precondition: should be called with cam->busy_lock held
 */
static int
reset_camera(struct sqcam_camera *cam)
{
	int status;
	unsigned char ch;

	ch = 0xff;
	if ((status =
	     send_control_msg(1, cam, 0x0c, 0xc0, 0x00, &ch, 1)) < 0)
		return status;
	if ((status =
	     send_control_msg(1, cam, 0x0c, 0x06, 0xa0, NULL, 0)) < 0)
		return status;
	if ((status =
	     send_control_msg(0, cam, 0x0c, 0x07, 0, &ch, 1)) < 0)
		return status;
	return 0;
}

static int
sqcam_ioctl(struct inode *inode, struct file *file, unsigned int ioctlnr, unsigned long ul_arg)
{
	void *arg = (void *)ul_arg;
	struct video_device *dev = video_devdata(file);
	struct sqcam_camera *cam = dev_get_drvdata(dev->dev);
	int retval = 0;

	if (!cam)
		return -ENODEV;

	if (down_interruptible(&cam->busy_lock))
		return -EINTR;

	switch (ioctlnr) {
		/* query capabilites */
	case VIDIOCGCAP:
		{
			struct video_capability b;

			DBG("VIDIOCGCAP\n");
			strcpy(b.name, "SQcam-based Camera");
			b.type = VID_TYPE_CAPTURE;
			b.channels = 1;
			b.audios = 0;
			b.maxwidth = 320;	/* VIDEOSIZE_CIF */
			b.maxheight = 240;
			b.minwidth = 320;	/* VIDEOSIZE_160_120 */
			b.minheight = 240;

			if (copy_to_user(arg, &b, sizeof (b)))
				retval = -EFAULT;

			break;
		}
	case VIDIOC_QUERYCAP:
		{
			struct v4l2_capability b;

			DBG("VIDIOC_QUERYCAP\n");
			strcpy(b.driver, "sqcam");
			strcpy(b.card, "Video Camera");
			strcpy(b.card, "USB");
			b.version = 1;
			b.capabilities = V4L2_CAP_VIDEO_CAPTURE |
				 V4L2_CAP_READWRITE ;
			b.reserved[0] = 0;
			b.reserved[1] = 0;
			b.reserved[2] = 0;
			b.reserved[3] = 0;

			if (copy_to_user(arg, &b, sizeof (b)))
				retval = -EFAULT;

			break;
		}
	case VIDIOC_G_PARM:
		{
			struct v4l2_streamparm *p = arg;
			switch (p->type)
			{
				case V4L2_BUF_TYPE_VIDEO_CAPTURE:
					break;
				case V4L2_BUF_TYPE_VIDEO_OUTPUT:
				case V4L2_BUF_TYPE_VIDEO_OVERLAY:
				case V4L2_BUF_TYPE_VBI_CAPTURE:
				case V4L2_BUF_TYPE_VBI_OUTPUT:
					retval = -EFAULT;
					break;
				default:
					retval = -EFAULT;
					break;
			}
			if (retval == -EFAULT) break;

			p->parm.capture.capability = 0;
			p->parm.capture.capturemode = 0;
			p->parm.capture.timeperframe.numerator = 0;
			p->parm.capture.timeperframe.denominator = 1;
			p->parm.capture.extendedmode = 0;
			p->parm.capture.readbuffers = 1; //cam->nreadbuffers;
			p->parm.capture.reserved[0] = 0;
			p->parm.capture.reserved[1] = 0;
			p->parm.capture.reserved[2] = 0;
			p->parm.capture.reserved[3] = 0;

			break;
		}
	/* case VIDIOC_G_STD: */
	case VIDIOC_S_STD:
	/* case VIDIOC_QUERYSTD: */
	case VIDIOC_ENUMSTD:
	/* case VIDIOC_QUERYMENU: */
		{
			retval = -EINVAL;
			break;
		}
	case VIDIOC_ENUMINPUT:
		{
			struct v4l2_input i;
			unsigned index;

			if (copy_from_user(&i, arg, sizeof(i))) {
				retval = -EFAULT;
				break;
			}

			index=i.index;
			memset(&i, 0, sizeof(i));
			i.index=index;
			i.type=V4L2_INPUT_TYPE_CAMERA;
			switch (i.index) {
				case 0:
					strcpy(i.name, "Bayer Grid 1");
					break;
				case 1:
					strcpy(i.name, "Bayer Grid 2");
					break;
				case 2:
					strcpy(i.name, "Bayer Grid 3");
					break;
				case 3:
					strcpy(i.name, "Bayer Grid 4");
					break;
				case 4:
					strcpy(i.name, "Bayer Grid 5");
					break;
				case 5:
					strcpy(i.name, "Bayer Grid 6");
					break;
				case 6:
					strcpy(i.name, "Bayer Grid 7");
					break;
				case 7:
					strcpy(i.name, "Bayer Grid 8");
					break;
				default:
					retval = -EINVAL;
					break;
			}


			if (copy_to_user(arg, &i, sizeof(i)))
				retval = -EFAULT;
			break;
		}
	case VIDIOC_G_INPUT:
		{
			int index=cam->channel;

			if (copy_to_user(arg, &index, sizeof(index))) {
				retval = -EFAULT;
			}

			break;
		}
	case VIDIOC_S_INPUT:
		{
			int index=cam->channel;

			if (copy_from_user(&index, arg, sizeof(index))) {
				retval = -EFAULT;
				break;
			}

			if (index > 7)
				retval = -EINVAL;
			else
				cam->channel=index;
			
			break;
		}
	case VIDIOC_G_FMT:
		{
			struct v4l2_format format;
			struct v4l2_pix_format pfmt;

			if (copy_from_user(&format, arg, sizeof(format))) {
				retval = -EFAULT;
				break;
			}
			
			if (format.type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
				retval = -EINVAL;
				break;
			}

			pfmt.width = 320;
			pfmt.height = 240;
			pfmt.pixelformat = V4L2_PIX_FMT_BGR24;
			pfmt.field = V4L2_FIELD_NONE;
			pfmt.bytesperline = 320 * 3;
			pfmt.sizeimage = SQCAM_MAX_FRAME_SIZE;
			pfmt.colorspace = V4L2_COLORSPACE_SRGB;
			memcpy(&(format.fmt.pix), &pfmt, sizeof(pfmt));

			if (copy_to_user(arg, &format, sizeof(format)))
				retval = -EFAULT;

			break;
		}
	case VIDIOC_S_FMT:
		{
			struct v4l2_format format;
			struct v4l2_pix_format pfmt;
			char s[5] = {'-', '-', '-',  '-', 0};
			__u32 mode;
			int i;

			if (copy_from_user(&format, arg, sizeof(format))) {
				retval = -EFAULT;
				break;
			}

			if (format.type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
				retval = -EINVAL;
				break;
			}
			pfmt = format.fmt.pix;
			mode = pfmt.pixelformat;
			for (i = 0; i < 4; i++) {
				s[i] = mode & 0xff;
				mode >>= 8;
			}

			printk(KERN_INFO "SQcam VIDIOC_S_FMT: width=%u, "
					 "height=%u, pixelformat=%s, "
					 "colorspace=%u\n", pfmt.width,
					 pfmt.height, s, pfmt.colorspace);
			printk(KERN_INFO "                    field=%u, "
					 "bytesperline=%u, sizeimage=%u\n",
					 pfmt.field, pfmt.bytesperline,
					 pfmt.sizeimage);

			if (pfmt.width != 320)
				format.fmt.pix.width = 320;
			if (pfmt.height != 240)
				format.fmt.pix.height = 240;
			if (pfmt.pixelformat != V4L2_PIX_FMT_BGR24)
				format.fmt.pix.pixelformat = V4L2_PIX_FMT_BGR24;
			format.fmt.pix.sizeimage = SQCAM_MAX_FRAME_SIZE;

			if (copy_to_user(arg, &format, sizeof(format)))
				retval = -EFAULT;

			break;
		}
	case VIDIOC_G_CTRL:
		{
			struct v4l2_control ctrl;
			if (copy_from_user(&ctrl, arg, sizeof(ctrl))) {
				retval = -EFAULT;
				break;
			}
			printk(KERN_INFO "SQcam VIDIOC_G_CTRL 0x%x\n", ctrl.id);
			switch (ctrl.id)
			{
				case V4L2_CID_AUDIO_VOLUME:
				case V4L2_CID_AUDIO_BALANCE:
				case V4L2_CID_AUDIO_BASS:
				case V4L2_CID_AUDIO_TREBLE:
				case V4L2_CID_AUDIO_MUTE:
				case V4L2_CID_AUDIO_LOUDNESS:
					printk(KERN_INFO "SQcam VIDIOC_G_CTRL "
                                                         " not supported\n");
					retval = -EINVAL;
					break;
				case V4L2_CID_BRIGHTNESS:
				case V4L2_CID_CONTRAST:
				case V4L2_CID_SATURATION:
				case V4L2_CID_HUE:
				case V4L2_CID_BLACK_LEVEL:
				case V4L2_CID_AUTO_WHITE_BALANCE:
				case V4L2_CID_DO_WHITE_BALANCE:
				case V4L2_CID_RED_BALANCE:
				case V4L2_CID_BLUE_BALANCE:
				case V4L2_CID_GAMMA:
				case V4L2_CID_EXPOSURE:
				case V4L2_CID_AUTOGAIN:
				case V4L2_CID_GAIN:
				case V4L2_CID_HFLIP:
				case V4L2_CID_VFLIP:
				case V4L2_CID_HCENTER:
				case V4L2_CID_VCENTER:
					break;
				default:
					break;
			}
			break;
		}
	case VIDIOC_REQBUFS:
		{
			/*struct v4l2_requestbuffers *rb = arg;
			printk(KERN_INFO "SQcam VIDIOC_REQBUFS: memory=%u, "
					 "type=%u", rb->memory, rb->type);
			if (rb->memory == V4L2_MEMORY_MMAP)
				printk(", count=%u", rb->count);
			printk("\n");*/
			retval = -EINVAL; /* Memory mapped IO is not supported. */
			break;
		}
	case VIDIOC_QUERYBUF:
		{
			struct v4l2_buffer b;

			if (copy_from_user(&b, arg, sizeof(b))) {
				up(&cam->busy_lock);
				return -EFAULT;
			}

			printk(KERN_INFO "SQcam VIDIOC_QUERYBUF: type=%u, "
					 "index=%u, memory=0x%x\n", b.type,
					 b.index, b.memory);
			// if (b.type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
			// 	return -EINVAL;

			/* memcpy(&b, &cam->frame[b.index].buf, sizeof(b));

			if (cam->frame[b.index].vma_use_count)
				b.flags |= V4L2_BUF_FLAG_MAPPED;

			if (cam->frame[b.index].state == F_DONE)
				b.flags |= V4L2_BUF_FLAG_DONE;
			else if (cam->frame[b.index].state != F_UNUSED)
				b.flags |= V4L2_BUF_FLAG_QUEUED;

			if (copy_to_user(arg, &b, sizeof(b)))
				return -EFAULT;

			up(&cam->busy_lock);
			return -ENOIOCTLCMD; */
			retval = -EINVAL; /* Memory mapped IO is not supported. */
			break;
		}
	/* get/set video source - we are a camera and nothing else */
	case VIDIOCGCHAN:
		{
			struct video_channel v;

			DBG("VIDIOCGCHAN\n");
			if (copy_from_user(&v, arg, sizeof (v))) {
				retval = -EFAULT;
				break;
			}
			if (v.channel != 0 ) {
				retval = -EINVAL;
				break;
			}

			/* v.channel = 0;*/
                        cam->channel = v.channel;
			strcpy(v.name, "Camera");
			v.tuners = 0;
			v.flags = 0;
			v.type = VIDEO_TYPE_CAMERA;
			v.norm = 0;

			if (copy_to_user(arg, &v, sizeof (v)))
				retval = -EFAULT;
			break;
		}

	case VIDIOCSCHAN:
		{
			int v;

			if (copy_from_user(&v, arg, sizeof (v)))
				retval = -EFAULT;
			DBG("VIDIOCSCHAN %d\n", v);

			if (retval == 0 && v != 0)
				retval = -EINVAL;
                         cam->channel = v;
			break;
		}

		/* image properties */
	case VIDIOCGPICT:
		{
			struct video_picture vp;
			DBG("VIDIOCGPICT\n");
			memset(&vp, 0, sizeof (struct video_picture));

			vp.depth = 24;
			vp.palette = VIDEO_PALETTE_RGB24;
			if (copy_to_user
			    (arg, &vp, sizeof (struct video_picture)))
				retval = -EFAULT;
			break;
		}

	case VIDIOCSPICT:
		{
			struct video_picture vp;
			
			if(copy_from_user(&vp, (struct video_picture *) arg,
				sizeof(struct video_picture)))
				retval = -EFAULT;

			else
			{
				DBG("VIDIOCSPICT depth = %d, pal = %d\n", vp.depth,
				    vp.palette);


				if (vp.depth != 24
				    || vp.palette != VIDEO_PALETTE_RGB24)
					retval = -EINVAL;
			}

			break;
		}

		/* get/set capture window */
	case VIDIOCGWIN:
		{
			struct video_window vw;
			vw.x = 0;
			vw.y = 0;
			vw.width = 320;
			vw.height = 240;
			vw.chromakey = 0;
			vw.flags = 0;
			vw.clips = NULL;
			vw.clipcount = 0;

			DBG("VIDIOCGWIN\n");

			if (copy_to_user
			    ((void *) arg, (void *) &vw, sizeof (vw)))
				retval = -EFAULT;

			// I'm not sure what the deal with a capture window is, it is very poorly described
			// in the doc.  So I won't support it now.
			break;
		}

	case VIDIOCSWIN:
		{

			struct video_window *vw = (struct video_window *) arg;
			DBG("VIDIOCSWIN %d x %d\n", vw->width, vw->height);

			if ( vw->width != 320 || vw->height != 240 )
				retval = -EFAULT;
			
			break;
		}

		/* mmap interface */
	case VIDIOCGMBUF:
		{
			struct video_mbuf vm;
			int i;

			DBG("VIDIOCGMBUF\n");
			memset(&vm, 0, sizeof (vm));
			vm.size =
			    SQCAM_MAX_FRAME_SIZE * SQCAM_FRAMES;
			vm.frames = SQCAM_FRAMES;
			for (i = 0; i < SQCAM_FRAMES; i++)
				vm.offsets[i] = SQCAM_MAX_FRAME_SIZE * i;

			if (copy_to_user
			    ((void *) arg, (void *) &vm, sizeof (vm)))
				retval = -EFAULT;

			break;
		}

	case VIDIOCMCAPTURE:
		{
			struct video_mmap vm;
			// int video_size;

			if (copy_from_user
			    ((void *) &vm, (void *) arg, sizeof (vm))) {
				retval = -EFAULT;
				break;
			}

			DBG("VIDIOCMCAPTURE frame=%d, height=%d, width=%d, format=%d.\n",vm.frame,vm.width,vm.height,vm.format);

			if ( vm.frame >= SQCAM_FRAMES || vm.format != VIDEO_PALETTE_RGB24 )
				retval = -EINVAL;

			// in theory right here we'd start the image capturing
			// (fill in a bulk urb and submit it asynchronously)
			//
			// Instead we're going to do a total hack job for now and
			// retrieve the frame in VIDIOCSYNC

			break;
		}

	case VIDIOCSYNC:
		{
			int frame;

			if (copy_from_user((void *) &frame, arg, sizeof (int))) {
				retval = -EFAULT;
				break;
			}
			DBG("VIDIOCSYNC: %d\n", frame);
			read_frame(cam, frame);

			break;
		}

		/* pointless to implement overlay with this camera */
	case VIDIOCCAPTURE:
	case VIDIOCGFBUF:
	case VIDIOCSFBUF:
	case VIDIOCKEY:
		retval = -EINVAL;
		break;

		/* tuner interface - we have none */
	case VIDIOCGTUNER:
	case VIDIOCSTUNER:
	case VIDIOCGFREQ:
	case VIDIOCSFREQ:
		retval = -EINVAL;
		break;

		/* audio interface - we have none */
	case VIDIOCGAUDIO:
	case VIDIOCSAUDIO:
		retval = -EINVAL;
		break;
	default:
		retval = -EINVAL;
		break;
	}

	up(&cam->busy_lock);
	return retval;
}

static int
sqcam_open(struct inode *inode, struct file *file)
{
    	struct video_device *dev = video_devdata(file);
	struct sqcam_camera *cam = dev_get_drvdata(dev->dev);
	unsigned char siz;
	int n;
	int intr;
	DBG("open\n");
	
	intr = down_interruptible(&cam->busy_lock);
	if (intr)
		return -EINTR;

	if (cam->is_opened) {
		printk(KERN_INFO
		       "sqcam_open called on already opened camera");
		up(&cam->busy_lock);
		return -EBUSY;
	}

	cam->raw_image = kmalloc(SQCAM_MAX_RAW_SIZE, GFP_KERNEL);
	if (!cam->raw_image) {
		up(&cam->busy_lock);
		return -ENOMEM;
	}

	cam->framebuf =
		    usbvideo_rvmalloc(SQCAM_MAX_FRAME_SIZE * SQCAM_FRAMES);
	if (!cam->framebuf) {
		kfree(cam->raw_image);
		up(&cam->busy_lock);
		return -ENOMEM;
	}

	if (!cam->is_initialized) {
		n = initialize_camera(cam);
            if (n >= 0)
		cam->is_initialized = 1;
	}
	siz = 0x00;
	if ((n = send_control_msg(1, cam, 0x0c, 0xc0, 0, &siz, 0)) < 0)
		{
		printk(KERN_ERR "sqcam_open: message 1, error %d.\n",n);
		kfree(cam->raw_image);
		up(&cam->busy_lock);
		return n;
		}
	siz = 0x61; // 0x60 -> 160x120, 0x61 -> 320x240
	if (n >= 0) n = send_control_msg(1, cam, 0x0c, 0x06, siz, NULL, 0);
		if (n < 0)
		{
		printk(KERN_ERR "sqcam_open: message 2, error %d.\n",n);
		kfree(cam->raw_image);
		up(&cam->busy_lock);
		return n;
		}
	siz = 0x00;
	if (n >= 0) n = send_control_msg(0, cam, 0x0c, 0x07, 0x00, &siz, 1);
		if (n < 0)
		{
		printk(KERN_ERR "sqcam_open: message 3, error %d.\n",n);
		kfree(cam->raw_image);
		up(&cam->busy_lock);
		return n;
		}
        if (n < 0) {
		cam->is_initialized = 0;
		kfree(cam->raw_image);
		up(&cam->busy_lock);
		return -EIO;
        }
	cam->is_opened = 1;

	up(&cam->busy_lock);

	return 0;
}

static int 
sqcam_close(struct inode *inode, struct file *file)
{
	struct video_device *dev = video_devdata(file);
	struct sqcam_camera *cam = dev_get_drvdata(dev->dev);
	struct usb_device *udev;
	DBG("close\n");


	/* it's not the end of the world if
	 * we fail to turn the camera off.
	 */


	down(&cam->busy_lock);
	reset_camera(cam);
	kfree(cam->raw_image);
	usbvideo_rvfree(cam->framebuf, SQCAM_MAX_FRAME_SIZE * SQCAM_FRAMES);
	cam->is_opened = 0;
   

	udev = cam->udev;

	up(&cam->busy_lock);

	if (!udev) {
		kfree(cam);
	}

	return 0;
}


#define DATA_HEADER_SIZE 64

void sqcam_decode_bayer(const u8 *data, u8 *rgb, struct sqcam_camera *cam)
{
	int j;
	unsigned char b;
	unsigned char *src;
	unsigned char *dst;
	unsigned char swap_buff[320];

	src = (u8 *) data + DATA_HEADER_SIZE;
	dst = rgb;

	/* Reverse the data, else the pic is upside down and possibly
	   mirror imaged too depending on model. */
        if (cam->model == SQ_MODEL_DEFAULT) {
		for (j = 0; j < 320*240 / 2; j++) {
			b = src[j];
			src[j] = src[320*240 - 1 - j];
			src[320*240 - 1 - j] = b;
		}
	} else {
		for (j = 0; j < 120; j++) {
			memcpy(swap_buff, src + (j * 320), 320);
			memcpy(src + (j * 320), src + ((239 - j) * 320), 320);
			memcpy(src + ((239 - j) * 320), swap_buff, 320);
		}
	}
	gp_bayer_decode(src, 320, 240, dst, cam->channel);
       gamma_correct_single (gamma_table, dst  , 320 * 240);

}
/*
 * precondition: should be called with busy_lock held
 */
static void
read_frame(struct sqcam_camera *cam, int framenum)
{
	unsigned char siz, msg;
	int n, read;
	int actual_length;


	if (!cam->udev) {
		return;
	}
	msg = 0x03; 
	        read = 320 * 240 + 64;
	// read 'read' bytes in SQCAM_MAX_READ_SIZE chunks

		
	do {
		siz = 0x50;
		n = send_control_msg(1, cam, 0x0c,0x03, 
				(read > SQCAM_MAX_READ_SIZE) ? SQCAM_MAX_READ_SIZE : read, 
				&siz, 1);
		if (n < 0) {
			printk(KERN_ERR
		       		" sqcam_read_frame: Problem sending frame capture control message");
			goto done;
		}
		if (cam->udev == NULL)
			goto done;
		up(&cam->busy_lock);
		n = usb_bulk_msg(cam->udev,
			 usb_rcvbulkpipe(cam->udev, cam->bulkEndpoint),
			 cam->raw_image + (320*240+64 - read),
			 (read > SQCAM_MAX_READ_SIZE) ? SQCAM_MAX_READ_SIZE : read,
			 &actual_length, 1500);
		read -= actual_length;
		DBG("blk msg:: size: %d\n", actual_length);
		if (n < 0) {
			printk(KERN_ERR "Problem during bulk read of frame data: %d\n", n);
			read = 0;
		if (down_interruptible(&cam->busy_lock))
			goto done;
		}
	} while (read);	
	
	sqcam_decode_bayer(cam->raw_image,
			 cam->framebuf +
			 framenum * SQCAM_MAX_FRAME_SIZE,
                         cam );
	cam->framebuf_size =
	    320 * 240 * SQCAM_BYTES_PER_PIXEL;
	cam->framebuf_read_start = 0;
 done: 
  DBG("sqcam_read_frame: exiting \n");
}

static ssize_t
sqcam_read( struct file *file, char *buf, size_t count, loff_t *ppos )
{
	struct video_device *dev = video_devdata(file);
	struct sqcam_camera *cam = dev_get_drvdata(dev->dev);
	
	int intr = down_interruptible(&cam->busy_lock);
	if (intr)
		return -EINTR;


	DBG("read %d bytes.\n", (int) count);

	if (*ppos >= SQCAM_MAX_FRAME_SIZE) {
		*ppos = 0;
		return 0;
	}

	if (*ppos == 0) {
		read_frame(cam, 0);
	}

	count = min_t(size_t, count, SQCAM_MAX_FRAME_SIZE - *ppos);

	if (copy_to_user(buf, &cam->framebuf[*ppos], count)) {
		count = -EFAULT;
	} else {
		*ppos += count;
	}

	if (count == SQCAM_MAX_FRAME_SIZE) {
		*ppos = 0;
	}
	DBG("read %d bytes. finished\n", (int) count);

	up(&cam->busy_lock);
	return count;
}






static int
sqcam_mmap(struct file *file, struct vm_area_struct *vma)
{
	// TODO: allocate the raw frame buffer if necessary
	unsigned long start = vma->vm_start;
	unsigned long size  = vma->vm_end-vma->vm_start;
	unsigned long page, pos;
	struct video_device *dev = video_devdata(file);
	struct sqcam_camera *cam = dev_get_drvdata(dev->dev);

	DBG("sqcam_mmap: %ld\n", size);

	/* We let mmap allocate as much as it wants because Linux was adding 2048 bytes
	 * to the size the application requested for mmap and it was screwing apps up.
	 if (size > SQCAM_FRAMES*SQCAM_MAX_FRAME_SIZE)
	 return -EINVAL;
	 */
	
	if (!cam)
		return -ENODEV;

        
	if (down_interruptible(&cam->busy_lock))
		return -EINTR;


	if (!cam->framebuf) {
		cam->framebuf =
		    usbvideo_rvmalloc(SQCAM_MAX_FRAME_SIZE * SQCAM_FRAMES);
		if (!cam->framebuf) {
			up(&cam->busy_lock);
			return -ENOMEM;
		}
	}

	pos = (unsigned long) (cam->framebuf);
	while (size > 0) {
		page = usbvideo_kvirt_to_pa(pos);
#ifdef HAS_REMAP_PAGE_RANGE
		if (remap_page_range(vma, start, page, PAGE_SIZE,
                     PAGE_SHARED)) {
#else
		if (remap_pfn_range(vma, start, page >> PAGE_SHIFT, PAGE_SIZE,
                    PAGE_SHARED)) {
#endif
			up(&cam->busy_lock);
			return -EAGAIN;
		}
		start += PAGE_SIZE;
		pos += PAGE_SIZE;
		if (size > PAGE_SIZE)
			size -= PAGE_SIZE;
		else
			size = 0;
	}

	up(&cam->busy_lock);

	return 0;
}




/* table of devices that work with this driver */
static struct usb_device_id sqcam_table[] = {
	{USB_DEVICE(USB_SQCAM_VENDOR_ID, USB_SQCAM_PRODUCT_ID)},
	{}			/* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, sqcam_table);

static struct file_operations sqcam_fops = {
	.owner =  THIS_MODULE,
	.open =   sqcam_open,
	.release = sqcam_close,
	.read =   sqcam_read,
	.mmap =  sqcam_mmap,
	.ioctl = sqcam_ioctl,
	.llseek = no_llseek,
};

static struct usb_driver sqcam_driver = {
        .name           = "sqcam",
        .probe          = sqcam_probe,
        .disconnect     = sqcam_disconnect,
        .id_table       = sqcam_table,
};



static void sqcam_exclusive_release(struct video_device *vdev)
{
	struct sqcam_camera *cam = dev_get_drvdata(vdev->dev);
/*FIXME: this could be called after sqcam_close or sqcam_disconnect and print an error message */	
	if (!cam){
	 printk(KERN_ERR"sqcam_release: invalid data");
                return ;
	}
       down(&cam->busy_lock);

	if (cam->is_opened) {
		cam->is_removed = 1;
	} 
	 printk(KERN_ERR"sqcam_release: usb driver release\n");
       usb_driver_release_interface(&sqcam_driver,
                                     cam->udev->actconfig->interface[0]);
	 printk(KERN_ERR"sqcam_release: usb driver release returned\n");
	up(&cam->busy_lock);

	
	vdev->users--;
	return ;
}




static struct video_device sqcam_template = {
	.owner = THIS_MODULE,
	.name  = "SQcam-based USB Camera",
	.type   = VID_TYPE_CAPTURE,
	.release = &sqcam_exclusive_release,
        .fops  =  &sqcam_fops,
	.minor = -1,
};

/**
 *	sqcam_probe
 *
 *	Called by the usb core when a new device is connected that it thinks
 *	this driver might be interested in.
 */
static int
sqcam_probe(struct usb_interface *intf,
		  const struct usb_device_id *id)
{
	int nas, bulkEndpoint = 0;
        struct usb_device *dev = interface_to_usbdev(intf);
	const struct usb_host_interface *interface;
	const struct usb_endpoint_descriptor *endpoint;
	struct sqcam_camera *cam;
	__u8 ifnum = intf->altsetting->desc.bInterfaceNumber;

	/* See if the device offered us matches what we can accept */
	if ((dev->descriptor.idVendor != USB_SQCAM_VENDOR_ID) ||
	    (dev->descriptor.idProduct != USB_SQCAM_PRODUCT_ID)) {
		return -ENODEV;
	}

	printk(KERN_INFO "SQcam based webcam connected\n");

	nas = dev->actconfig->interface[ifnum]->num_altsetting;
	if (nas != 1) {
		printk(KERN_WARNING
		       "Expected only one alternate setting for this camera!\n");
		return -EINVAL;
	}

	interface = &dev->actconfig->interface[ifnum]->altsetting[0];
	DBG(KERN_DEBUG "Interface %d. has %u. endpoints!\n",
	       interface->desc.bInterfaceNumber, (unsigned) (interface->desc.bNumEndpoints));
	endpoint = &interface->endpoint[0].desc;

	if ((endpoint->bEndpointAddress & 0x80) &&
	    ((endpoint->bmAttributes & 3) == 0x02)) {
		/* we found a bulk in endpoint */
		bulkEndpoint = endpoint->bEndpointAddress;
	} else {
		printk(KERN_ERR
		       "No bulk in endpoint was found ?! (this is bad)\n");
	}

	if ((cam =
	     kmalloc(sizeof (struct sqcam_camera), GFP_KERNEL)) == NULL) {
		printk(KERN_WARNING
		       "could not allocate kernel memory for sqcam_camera struct\n");
		return -ENOMEM;
	}

	memset(cam, 0, sizeof (struct sqcam_camera));

	init_MUTEX(&cam->busy_lock);

	memcpy(&cam->vdev, &sqcam_template,
	       sizeof (sqcam_template));

	cam->udev = dev;
	cam->bulkEndpoint = bulkEndpoint;
	cam->is_initialized = 0;
	cam->is_opened = 0;
	cam->gamma = 0.65;
	cam->vdev.dev = &dev->dev;
        gamma_fill_table (gamma_table, cam->gamma);

	if (video_register_device(&cam->vdev, VFL_TYPE_GRABBER, -1) == -1) {
		kfree(cam);
		printk(KERN_WARNING "video_register_device failed\n");
		return -EIO;
	}

	printk(KERN_INFO "SQcam webcam driver v0.1b now controlling video device %d\n",cam->vdev.minor);
	if (cam->vdev.dev != NULL) {
		dev_set_drvdata(cam->vdev.dev,cam);
	}
	else
	{
		printk(KERN_WARNING "dev pointer in device structure not yet initialised\n");
	}
	usb_set_intfdata (intf, cam);


	return 0;
}

static void
sqcam_disconnect(struct usb_interface *intf)
{
    struct sqcam_camera *cam = usb_get_intfdata(intf);


        if (!cam)
                return;
       down(&cam->busy_lock);
	reset_camera(cam);
	up(&cam->busy_lock);
        usb_set_intfdata(intf, NULL);
	video_unregister_device(&cam->vdev);
       down(&cam->busy_lock);

	cam->udev = NULL;


	/* the only thing left to do is synchronize with
	 * our close/release function on who should release
	 * the camera memory. if there are any users using the
	 * camera, it's their job. if there are no users,
	 * it's ours.
	 */

	up(&cam->busy_lock);

	if (!cam->is_opened) {
		kfree(cam);
	}
	printk(KERN_DEBUG "SQCam-based WebCam disconnected\n");
	
}

static int __init
usb_sqcam_init(void)
{
	int retval;
	DBG(KERN_INFO "SQcam-based WebCam driver startup\n");

	retval = usb_register(&sqcam_driver);
	if (retval)
		printk(KERN_WARNING "usb_register failed!\n");
	return retval;
}

static void __exit
usb_sqcam_exit(void)
{
	DBG(KERN_INFO
	       "SQcam-based WebCam driver shutdown\n");

	usb_deregister(&sqcam_driver);
}

module_init(usb_sqcam_init);
module_exit(usb_sqcam_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
/* bayer.c
 *
 * Copyright © 2001 Lutz Müller <urc8@rz.uni-karlsruhe.de>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details. 
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/* This is from libgphoto2 */


static int tile_colors[8][4] = {
	{0, 1, 1, 2},
	{1, 0, 2, 1},
	{2, 1, 1, 0},
	{1, 2, 0, 1},
	{0, 1, 1, 2},
	{1, 0, 2, 1},
	{2, 1, 1, 0},
	{1, 2, 0, 1}};

#define RED 0
#define GREEN 1
#define BLUE 2

static int
gp_bayer_expand (unsigned char *input, int w, int h, unsigned char *output,
		 int tile)
{
	int x, y, i;
	int colour, bayer;
	char *ptr = input;

	switch (tile) {

		case 0:
		case 1:
		case 2:
		case 3: 

			for (y = 0; y < h; ++y)
				for (x = 0; x < w; ++x, ++ptr) {
					bayer = (x&1?0:1) + (y&1?0:2);

					colour = tile_colors[tile][bayer];

					i = (y * w + x) * 3;

					output[i+RED]    = 0;
					output[i+GREEN]  = 0;
					output[i+BLUE]   = 0;
					output[i+colour] = *ptr;
				}
			break;

		case 4: //BAYER_TILE_RGGB_INTERLACED:
		case 5: //BAYER_TILE_GRBG_INTERLACED:
		case 6: //BAYER_TILE_BGGR_INTERLACED:
		case 7: //BAYER_TILE_GBRG_INTERLACED:
 

			for (y = 0; y < h; ++y, ptr+=w)
				for (x = 0; x < w; ++x) {
					bayer = (x&1?0:1) + (y&1?0:2);
	
					colour = tile_colors[tile][bayer];
	
					i = (y * w + x) * 3;

					output[i+RED]    = 0;
					output[i+GREEN]  = 0;
					output[i+BLUE]   = 0;
					output[i+colour] = (x&1)? ptr[x>>1]:ptr[(w>>1)+(x>>1)];
				}
			break;
	}

	return (0);
}

#define AD(x, y, w) ((y)*(w)*3+3*(x))

static int
gp_bayer_interpolate (unsigned char *image, int w, int h, int tile)
{
	int x, y, bayer;
	int p0, p1, p2, p3;
	int div, value;


	switch (tile) {
	default:
	case 0: //BAYER_TILE_RGGB:
	case 4: //BAYER_TILE_RGGB_INTERLACED:
		p0 = 0; p1 = 1; p2 = 2; p3 = 3;
		break;
	case 1: //BAYER_TILE_GRBG:
	case 5: //BAYER_TILE_GRBG_INTERLACED:
		p0 = 1; p1 = 0; p2 = 3; p3 = 2;
		break;
	case 2: //BAYER_TILE_BGGR:
	case 6: //BAYER_TILE_BGGR_INTERLACED:
		p0 = 3; p1 = 2; p2 = 1; p3 = 0;
		break;
	case 3: //BAYER_TILE_GBRG:
	case 7: //BAYER_TILE_GBRG_INTERLACED:	
		p0 = 2; p1 = 3; p2 = 0; p3 = 1;
		break;
	}


	//p0 = 3; p1 = 2; p2 = 1; p3 = 0; // 2
	//p0 = 0; p1 = 1; p2 = 2; p3 = 3;


	for (y = 0; y < h; y++)
		for (x = 0; x < w; x++) {
			bayer = (x&1?0:1) + (y&1?0:2);
			if ( bayer == p0 ) {
				
				/* red. green lrtb, blue diagonals */
				div = value = 0;
				if (y) {
					value += image[AD(x,y-1,w)+GREEN];
					div++;
				}
				if (y < (h - 1)) {
					value += image[AD(x,y+1,w)+GREEN];
					div++;
				}
				if (x) {
					value += image[AD(x-1,y,w)+GREEN];
					div++;
				}
				if (x < (w - 1)) {
					value += image[AD(x+1,y,w)+GREEN];
					div++;
				}
				image[AD(x,y,w)+GREEN] = value / div;

				div = value = 0;
				if ((y < (h - 1)) && (x < (w - 1))) {
					value += image[AD(x+1,y+1,w)+BLUE];
					div++;
				}
				if ((y) && (x)) {
					value += image[AD(x-1,y-1,w)+BLUE];
					div++;
				}
				if ((y < (h - 1)) && (x)) {
					value += image[AD(x-1,y+1,w)+BLUE];
					div++;
				}
				if ((y) && (x < (w - 1))) {
					value += image[AD(x+1,y-1,w)+BLUE];
					div++;
				}
				image[AD(x,y,w)+BLUE] = value / div;

			} else if (bayer == p1) {
				
				/* green. red lr, blue tb */
				div = value = 0;
				if (x < (w - 1)) {
					value += image[AD(x+1,y,w)+RED];
					div++;
				}
				if (x) {
					value += image[AD(x-1,y,w)+RED];
					div++;
				}
				image[AD(x,y,w)+RED] = value / div;

				div = value = 0;
				if (y < (h - 1)) {
					value += image[AD(x,y+1,w)+BLUE];
					div++;
				}
				if (y) {
					value += image[AD(x,y-1,w)+BLUE];
					div++;
				}
				image[AD(x,y,w)+BLUE] = value / div;

			} else if ( bayer == p2 ) {
				
				/* green. blue lr, red tb */
				div = value = 0;
				
				if (x < (w - 1)) {
					value += image[AD(x+1,y,w)+BLUE];
					div++;
				}
				if (x) {
					value += image[AD(x-1,y,w)+BLUE];
					div++;
				}
				image[AD(x,y,w)+BLUE] = value / div;

				div = value = 0;
				if (y < (h - 1)) {
					value += image[AD(x,y+1,w)+RED];
					div++;
				}
				if (y) {
					value += image[AD(x,y-1,w)+RED];
					div++;
				}
				image[AD(x,y,w)+RED] = value / div;

			} else {
				
				/* blue. green lrtb, red diagonals */
				div = value = 0;

				if (y) {
					value += image[AD(x,y-1,w)+GREEN];
					div++;
				}
				if (y < (h - 1)) {
					value += image[AD(x,y+1,w)+GREEN];
					div++;
				}
				if (x) {
					value += image[AD(x-1,y,w)+GREEN];
					div++;
				}
				if (x < (w - 1)) {
					value += image[AD(x+1,y,w)+GREEN];
					div++;
				}
				image[AD(x,y,w)+GREEN] = value / div;

				div = value = 0;
				if ((y < (h - 1)) && (x < (w - 1))) {
					value += image[AD(x+1,y+1,w)+RED];
					div++;
				}
				if ((y) && (x)) {
					value += image[AD(x-1,y-1,w)+RED];
					div++;
				}
				if ((y < (h - 1)) && (x)) {
					value += image[AD(x-1,y+1,w)+RED];
					div++;
				}
				if ((y) && (x < (w - 1))) {
					value += image[AD(x+1,y-1,w)+RED];
					div++;
				}
				image[AD(x,y,w)+RED] = value / div;
			}
		}

	return (0);
}

static int
gp_bayer_decode (unsigned char *input, int w, int h, unsigned char *output,
		 int tile)
{
	gp_bayer_expand (input, w, h, output, tile);
	gp_bayer_interpolate (output, w, h, tile);

	return (0);
}

static void *usbvideo_rvmalloc(unsigned long size)
{
	void *mem;
	unsigned long adr;

	size = PAGE_ALIGN(size);
	mem = vmalloc_32(size);
	if (!mem)
		return NULL;

	memset(mem, 0, size); /* Clear the ram out, no junk to the user */
	adr = (unsigned long) mem;
	while (size > 0) {
		SetPageReserved(vmalloc_to_page((void *)adr));
		adr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}

	return mem;
}

static void usbvideo_rvfree(void *mem, unsigned long size)
{
	unsigned long adr;

	if (!mem)
		return;

	adr = (unsigned long) mem;
	while ((long) size > 0) {
		ClearPageReserved(vmalloc_to_page((void *)adr));
		adr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}
	vfree(mem);
}

 unsigned long usbvideo_kvirt_to_pa(unsigned long adr)
{
	unsigned long kva, ret;

	kva = (unsigned long) page_address(vmalloc_to_page((void *)adr));
	kva |= adr & (PAGE_SIZE-1); /* restore the offset */
	ret = __pa(kva);
	return ret;
}


static int
gamma_correct_triple (unsigned char *table_red,
		      unsigned char *table_green,
		      unsigned char *table_blue,
		      unsigned char *data, unsigned int size)
{
	int x;

	for (x = 0; x < (size * 3); x += 3) {
		data[x + 0] = table_red  [data[x + 0]];
		data[x + 1] = table_green[data[x + 1]];
		data[x + 2] = table_blue [data[x + 2]];
	}

	return (0);
}

static int
gamma_correct_single (unsigned char *table, unsigned char *data,
			 unsigned int size)
{
	return (gamma_correct_triple (table, table, table, data, size));
}

  
  


static int
gamma_fill_table (unsigned char *table, double g)
{
/*	int x;

	for (x = 0; x < 256; x++)

		table[x] = 255 * pow ((double) x/255., g);
*/
	return (0);
}

