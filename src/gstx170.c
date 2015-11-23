/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support 
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include "gstx170.h"

#define GSTX170_INBUF_SIZE	0x200000		/* input buffering */
#define GSTX170_INBUF_THRESH	0x50000		/* decode threshold, needs to contain a full frame */
#define GSTX170_OUTBUF_SIZE	0x200000		/* output buffering */

#define PROP_DEFAULT_DISABLE_OUTPUT_REORDERING FALSE
#define PROP_DEFAULT_INTRA_FREEZE_CONCEALMENT FALSE
#define PROP_DEFAULT_USE_DISPLAY_SMOOTHING FALSE
#define PROP_DEFAULT_NUM_FRAME_BUFFS 0
#define PROP_DEFAULT_DPB_FLAGS DEC_DPB_ALLOW_FIELD_ORDERING

static GstElementDetails gstx170_details = {
	"Hantro x170 Video Decoder",
	"Codec/Decoder/Video",
	"Decodes Various Video Formats",
	"Lead Tech Design <www.leadtechdesign.com>\n"
};

DWLLinearMem_t seq_header_mem_info;
static gchar *seq_header;
static gchar *seq_header_buf[256];
static guint seq_header_len = 0;

static GstStaticPadTemplate sink_template =
	GST_STATIC_PAD_TEMPLATE("sink",
				GST_PAD_SINK,
				GST_PAD_ALWAYS,
				GST_STATIC_CAPS(
					"image/jpeg;"
					"video/x-h264, "
						"width  = (int) [16, 4096], "
						"height = (int) [16, 4096], "
						"framerate = (fraction) [ 0, MAX ]; "
					"video/mpeg, "
						"width  = (int) [16, 4096], "
						"height = (int) [16, 4096], "
						"framerate = (fraction) [ 0, MAX ], "
						"mpegversion = (int) 1, "
						"systemstream = (boolean) FALSE; "
					"video/mpeg, "
						"width  = (int) [16, 4096], "
						"height = (int) [16, 4096], "
						"framerate = (fraction) [ 0, MAX ], "
						"mpegversion = (int) 2, "
						"systemstream = (boolean) FALSE; "
					"video/mpeg, "
						"width  = (int) [16, 4096], "
						"height = (int) [16, 4096], "
						"framerate = (fraction) [ 0, MAX ], "
						"mpegversion = (int) 4, "
						"systemstream = (boolean) FALSE; "
					"video/x-xvid, "
						"width  = (int) [16, 4096], "
						"height = (int) [16, 4096], "
						"framerate = (fraction) [ 0, MAX ]; "
					"video/x-divx, "
						"width  = (int) [16, 4096], "
						"height = (int) [16, 4096], "
						"framerate = (fraction) [ 0, MAX ], "
						"divxversion = (int) {4,5}; "
					"video/x-wmv, "
						"width  = (int) [16, 4096], "
						"height = (int) [16, 4096], "
						"framerate = (fraction) [ 0, MAX ], "
						"wmvversion = (int) {3};")				
				);

static GstStaticPadTemplate src_template =
	GST_STATIC_PAD_TEMPLATE("src",
				GST_PAD_SRC,
				GST_PAD_ALWAYS,
				GST_STATIC_CAPS(
					"video/x-raw-rgb, "
						"width  = (int) [1, 4096], " 
						"height = [ 1, 4096 ], "
						"framerate = (fraction) [ 0/1, 2147483647/1 ], "
						"bpp = (int) 16, "
						"depth = (int) 15, " 
						"red_mask = (int) 63488, " 
						"green_mask = (int) 2016, " 
						"blue_mask = (int) 31, " 
						"endianness = (int) 1234;" 
					"video/x-raw-rgb, "
						"width  = (int) [1, 4096], " 
						"height = [ 1, 4096 ], "
						"framerate = (fraction) [ 0/1, 2147483647/1 ], "
						"bpp = (int) 16, "
						"depth = (int) 16, " 
						"red_mask = (int) 63488, " 
						"green_mask = (int) 2016, " 
						"blue_mask = (int) 31, " 
						"endianness = (int) 1234;" 
					"video/x-raw-rgb, "
						"width  = (int) [1, 4096], " 
						"height = [ 1, 4096 ], "
						"framerate = (fraction) [ 0/1, 2147483647/1 ], "
						"bpp = (int) 32, "
						"depth = (int) 24, " 
						"red_mask = (int) 16711680, " 
						"green_mask = (int) 65280, " 
						"blue_mask = (int) 255, " 
						"endianness = (int) 4321;"
					"video/x-raw-yuv,"
						"width  = (int) [1, 4096], " 
						"height = [ 1, 4096 ], "
						"framerate = (fraction) [ 0/1, 2147483647/1 ], "
						"format = (fourcc) \"NV12\";")
						);

GST_BOILERPLATE(Gstx170, gst_x170, GstElement, GST_TYPE_ELEMENT);

static GstElementClass *parent_class;

static int x170_malloc_linear(Gstx170 *x170, int size, DWLLinearMem_t * info)
{
	int pgsize = getpagesize();
	MemallocParams params;

	size = (size + (pgsize - 1)) & (~(pgsize - 1));

	info->size = size;
	info->virtualAddress = MAP_FAILED;
	info->busAddress = 0;

	params.size = size;

	/* get memory linear memory buffers */
	ioctl(x170->fd_memalloc, MEMALLOC_IOCXGETBUFFER, &params);
	if(params.busAddress == 0)
		return DWL_ERROR;

	info->busAddress = params.busAddress;

	/* Map the bus address to virtual address */
	info->virtualAddress = (u32 *) mmap(0, info->size,
					    PROT_READ | PROT_WRITE,
					    MAP_SHARED, x170->fd_mem,
					    params.busAddress);

	if (info->virtualAddress == MAP_FAILED)
		return DWL_ERROR;
	return DWL_OK;
}

static void x170_free_linear(Gstx170 *x170, DWLLinearMem_t * info)
{
	if (info->busAddress != 0)
		ioctl(x170->fd_memalloc, MEMALLOC_IOCSFREEBUFFER,
		      &info->busAddress);

	if (info->virtualAddress != MAP_FAILED)
		munmap(info->virtualAddress, info->size);
}

static gboolean gst_x170_ppsetconfig(Gstx170 *x170, guint pixformat, guint *width, guint *height,
				     guint interlaced)
{
	PPConfig ppconfig;
	PPResult ppret;
	guint tmp;

	if ((ppret = PPGetConfig(x170->pp, &ppconfig)) != PP_OK) {
		GST_DEBUG_OBJECT(x170, "cannot retrieve PP settings (err=%d)",
				 ppret);
		return FALSE;
	}

	*width = (*width + 7) & ~7;
	*height = (*height + 7) & ~7;
	ppconfig.ppInImg.width = *width;
	ppconfig.ppInImg.height = *height;
	ppconfig.ppInImg.pixFormat = pixformat;
	ppconfig.ppInImg.videoRange = 1;
	ppconfig.ppInImg.vc1RangeRedFrm = 0;
	ppconfig.ppInImg.vc1MultiResEnable = 0;
	ppconfig.ppInImg.vc1RangeMapYEnable = 0;
	ppconfig.ppInImg.vc1RangeMapYCoeff = 0;
	ppconfig.ppInImg.vc1RangeMapCEnable = 0;
	ppconfig.ppInImg.vc1RangeMapCCoeff = 0;

	if (x170->crop_width != 0 || x170->crop_height != 0) {
		ppconfig.ppInCrop.enable = 1;
		ppconfig.ppInCrop.originX = x170->crop_x;
		ppconfig.ppInCrop.originY = x170->crop_y;
		ppconfig.ppInCrop.width = x170->crop_width;
		ppconfig.ppInCrop.height = x170->crop_height;
		*width = x170->crop_width;
		*height = x170->crop_height;
	}
	else
		ppconfig.ppInCrop.enable = 0;

	switch (x170->rotation) {
	case HT_ROTATION_0:
		ppconfig.ppInRotation.rotation = PP_ROTATION_NONE;
		break;
	case HT_ROTATION_90:
		ppconfig.ppInRotation.rotation = PP_ROTATION_RIGHT_90;
		tmp = *width;
		*width = *height;
		*height = tmp;
		break;
	case HT_ROTATION_180:
		ppconfig.ppInRotation.rotation = PP_ROTATION_180;
		break;
	case HT_ROTATION_270:
		ppconfig.ppInRotation.rotation = PP_ROTATION_LEFT_90;
		tmp = *width;
		*width = *height;
		*height = tmp;
		break;
	}

	switch (x170->output) {
	case HT_OUTPUT_UYVY:
		ppconfig.ppOutImg.pixFormat = PP_PIX_FMT_YCBCR_4_2_0_SEMIPLANAR;
		break;
	case HT_OUTPUT_RGB15:
		ppconfig.ppOutImg.pixFormat = PP_PIX_FMT_RGB16_5_5_5;
		break;
	case HT_OUTPUT_RGB16:
		ppconfig.ppOutImg.pixFormat = PP_PIX_FMT_RGB16_5_6_5;
		break;
	case HT_OUTPUT_RGB32:
		ppconfig.ppOutImg.pixFormat = PP_PIX_FMT_RGB32;
		break;
	}

	/* Check scaler configuration */
	if( (x170->output_width != 0) && (x170->output_height != 0) ) {
		/* We can not upscale width and downscale height */ 
		if( (x170->output_width > *width) && (x170->output_height < *height) ) 
			x170->output_width = *width;
		
		/* We can not downscale width and upscale height */ 
		if( (x170->output_width < *width) && (x170->output_height > *height) ) 
			x170->output_height = *height;

		/* Check width output value (min and max) */ 
		if( x170->output_width < 16 )
			x170->output_width = 16;
		if( x170->output_width > 1280 ) 
			x170->output_width = 1280;

		/* Check height output value (min and max) */ 
		if( x170->output_height < 16 )
			x170->output_height = 16;
		if( x170->output_height > 720 )
			x170->output_height = 720;

		/* Check width upscale factor (limited to 3x the input width) */ 
		if( x170->output_width  > (3*(*width)) )
			x170->output_width = (3*(*width));

		/* Check height upscale factor (limited to 3x the input width - 2 pixels) */ 
		if( x170->output_height  > ((3*(*height) - 2)) )
			x170->output_height = ((3*(*height) - 2));

		/* Re-align according to vertical and horizontal scaler steps (resp. 8 and 2) */
		x170->output_width  = (x170->output_width/8)*8;
		x170->output_height = (x170->output_height/2)*2;
		
		*width  = x170->output_width;
		*height = x170->output_height;
	} 
	
	ppconfig.ppOutImg.width = *width;
	ppconfig.ppOutImg.height = *height;

	GST_DEBUG_OBJECT(x170, "ppconfig.ppOutImg.width = %d, ppconfig.ppOutImg.height = %d", ppconfig.ppOutImg.width, ppconfig.ppOutImg.height);
	GST_DEBUG_OBJECT(x170, "x170->output_width = %d, x170->output_height = %d", x170->output_width, x170->output_height);

	ppconfig.ppOutImg.bufferBusAddr = x170->outbuf.busAddress;
	ppconfig.ppOutImg.bufferChromaBusAddr = x170->outbuf.busAddress +
						ppconfig.ppOutImg.width *
						ppconfig.ppOutImg.height;
	memset(x170->outbuf.virtualAddress, 0, x170->outbuf_size);

	ppconfig.ppOutRgb.alpha = 255;
	if (interlaced)
		ppconfig.ppOutDeinterlace.enable = 1;

	if ((ppret = PPSetConfig(x170->pp, &ppconfig)) != PP_OK) {
		GST_DEBUG_OBJECT(x170, "cannot init. PP settings (err=%d)",
				 ppret);
		return FALSE;
	}
	return TRUE;
}

static void gst_x170_free(Gstx170 *x170)
{
	GST_DEBUG_OBJECT(x170, "gst_x170_free");

	switch (x170->codec) {

	case HT_AUTO:
		break;

	case HT_IMAGE_JPEG:
		if (x170->pp) {
			PPDecCombinedModeDisable(x170->pp, x170->jpegdec);
			PPRelease(x170->pp);
		}
		x170->pp = NULL;
		if (x170->jpegdec)
			JpegDecRelease(x170->jpegdec);
		x170->jpegdec = NULL;
		break;

	case HT_VIDEO_H264:
		if (x170->pp) {
			PPDecCombinedModeDisable(x170->pp, x170->h264dec);
			PPRelease(x170->pp);
		}
		x170->pp = NULL;
		if (x170->h264dec)
			H264DecRelease(x170->h264dec);
		x170->h264dec = NULL;
		break;

	case HT_VIDEO_MPEG2:
		if (x170->pp) {
			PPDecCombinedModeDisable(x170->pp, x170->mpeg2dec);
			PPRelease(x170->pp);
		}
		x170->pp = NULL;
		if (x170->mpeg2dec)
			Mpeg2DecRelease(x170->mpeg2dec);
		x170->mpeg2dec = NULL;
		break;

	case HT_VIDEO_MPEG4:
		if (x170->pp) {
			PPDecCombinedModeDisable(x170->pp, x170->mpeg4dec);
			PPRelease(x170->pp);
		}
		x170->pp = NULL;
		if (x170->mpeg4dec)
			MP4DecRelease(x170->mpeg4dec);
		x170->mpeg4dec = NULL;
		break;

	case HT_VIDEO_VC1:
		if (x170->pp) {
			PPDecCombinedModeDisable(x170->pp, x170->vc1dec);
			PPRelease(x170->pp);
		}
		x170->pp = NULL;
		if (x170->vc1dec)
			VC1DecRelease(x170->vc1dec);
		x170->vc1dec = NULL;
		x170->vc1meta_inited = 0;
		break;
	}

	if (x170->inbuf.virtualAddress)
		x170_free_linear(x170, &x170->inbuf);
	x170->inbuf.virtualAddress = NULL;

	if (x170->fb_data)
		free(x170->fb_data);

	if (x170->outbuf.virtualAddress)
		x170_free_linear(x170, &x170->outbuf);
	x170->outbuf.virtualAddress = NULL;

#ifdef USE_THREAD
	x170->force_exit = 1;
	sem_post(&x170->full_sem);
	pthread_join(x170->img_push_thread, (void **)0);
	sem_destroy(&x170->full_sem);
	sem_destroy(&x170->empty_sem);
#endif

	if (x170->fd_memalloc != -1)
		close(x170->fd_memalloc);
	x170->fd_memalloc = -1;

	if (x170->fd_mem != -1)
		close(x170->fd_mem);
	x170->fd_mem = -1;
}

#ifdef USE_THREAD
void *img_push_thread(void *arg)
{
	Gstx170 *x170 = (Gstx170 *)arg;
	GstFlowReturn ret;

	for (;;) {
		sem_wait(&x170->full_sem);

		if (x170->force_exit)
			break;

		if (x170->frm_count == 0)
			continue;
		ret = gst_pad_push(x170->srcpad, x170->frames[x170->read_pos]);
		if (ret != GST_FLOW_OK)
			GST_DEBUG_OBJECT(x170, "gst_pad_push returned error\n");
		x170->read_pos = (x170->read_pos + 1) % X170_FRAME_BUFFER_SIZE;
		x170->frm_count--;
		sem_post(&x170->empty_sem);
	}

	return NULL;
}
#endif

static gboolean gst_x170_alloc(Gstx170 *x170)
{
	void *decoder;
	int pptype;
	PPResult ppret;

	GST_DEBUG_OBJECT(x170, "gst_x170_alloc");

#ifdef USE_THREAD
	x170->frm_count = 0;
	x170->write_pos = 0;
	x170->read_pos = 0;
	x170->force_exit = 0;
	sem_init(&x170->full_sem, 0, 0);
	sem_init(&x170->empty_sem, 0, X170_FRAME_BUFFER_SIZE);
	pthread_create(&x170->img_push_thread, 0, img_push_thread, x170);
#endif
	x170->fd_memalloc = open(DEV_MEMALLOC_PATH, O_RDWR | O_SYNC);
	if (x170->fd_memalloc < 0) {
		GST_DEBUG_OBJECT(x170, "cannot open %s", DEV_MEMALLOC_PATH);
		gst_x170_free(x170);
		return FALSE;
	}

	x170->fd_mem = open(DEV_MEM_PATH, O_RDWR | O_SYNC);
	if (x170->fd_mem < 0) {
		GST_DEBUG_OBJECT(x170, "cannot open %s", DEV_MEM_PATH);
		gst_x170_free(x170);
		return FALSE;
	}
	
	if (x170_malloc_linear(x170, 256, &seq_header_mem_info) != DWL_OK) {
		GST_DEBUG_OBJECT(x170, "Failed to alloc mem for seq header!");
		return FALSE;
	}
	seq_header = (gchar *)seq_header_mem_info.virtualAddress;
		
	switch (x170->codec) {

	case HT_AUTO:
		GST_DEBUG_OBJECT(x170, "HT_AUTO codec, delaying alloc");
		return TRUE;

	case HT_IMAGE_JPEG: {
		JpegDecRet ret;

		ret = JpegDecInit(&x170->jpegdec);
		if (ret != JPEGDEC_OK) {
			GST_DEBUG_OBJECT(x170, "cannot create JPEG instance");
			gst_x170_free(x170);
			return FALSE;
		}
		GST_DEBUG_OBJECT(x170, "JPEG instance created");
		decoder = (void *) x170->jpegdec;
		pptype = PP_PIPELINED_DEC_TYPE_JPEG;
		}
		break;

	case HT_VIDEO_H264: {
		H264DecRet ret;
		ret = H264DecInit(&x170->h264dec, PROP_DEFAULT_DISABLE_OUTPUT_REORDERING,
			PROP_DEFAULT_INTRA_FREEZE_CONCEALMENT,
                        PROP_DEFAULT_USE_DISPLAY_SMOOTHING,
                        PROP_DEFAULT_DPB_FLAGS );
		if (ret != H264DEC_OK) {
			GST_DEBUG_OBJECT(x170, "cannot create H264 instance");
			gst_x170_free(x170);
			return FALSE;
		}
		GST_DEBUG_OBJECT(x170, "H264 instance created");
		decoder = (void *) x170->h264dec;
		pptype = PP_PIPELINED_DEC_TYPE_H264;
		}
		break;

	case HT_VIDEO_MPEG2: {
		Mpeg2DecRet ret;
		ret = Mpeg2DecInit(&x170->mpeg2dec,
                             PROP_DEFAULT_INTRA_FREEZE_CONCEALMENT,
                             PROP_DEFAULT_NUM_FRAME_BUFFS,
                             PROP_DEFAULT_DPB_FLAGS);
		if (ret != MPEG2DEC_OK) {
			GST_DEBUG_OBJECT(x170,
					 "cannot create MPEG2 instance");
			gst_x170_free(x170);
			return FALSE;
		}
		GST_DEBUG_OBJECT(x170, "MPEG2 instance created");
		decoder = (void *) x170->mpeg2dec;
		pptype = PP_PIPELINED_DEC_TYPE_MPEG2;
		}
		break;

	case HT_VIDEO_MPEG4: {
		MP4DecRet ret;
//MP4DEC_MPEG4 MP4DEC_SORENSON MP4DEC_CUSTOM_1(DivX4 or DivX5)
		ret = MP4DecInit(&x170->mpeg4dec, x170->subtype,
			PROP_DEFAULT_INTRA_FREEZE_CONCEALMENT,
                         PROP_DEFAULT_NUM_FRAME_BUFFS,
                         PROP_DEFAULT_DPB_FLAGS);
		if (ret != MP4DEC_OK) {
			GST_DEBUG_OBJECT(x170,
					 "cannot create MPEG4 instance");
			gst_x170_free(x170);
			return FALSE;
		}
		GST_DEBUG_OBJECT(x170, "MPEG4 instance created");
		decoder = (void *) x170->mpeg4dec;
		pptype = PP_PIPELINED_DEC_TYPE_MPEG4;
		}
		break;

	case HT_VIDEO_VC1: {
		VC1DecInfo decInfo;
		VC1DecInput decIn;
		VC1DecOutput decOut;
		VC1DecRet decRet;
		
		decRet = VC1DecInit(&x170->vc1dec, &x170->vc1meta,
			PROP_DEFAULT_INTRA_FREEZE_CONCEALMENT,
			PROP_DEFAULT_NUM_FRAME_BUFFS,
			PROP_DEFAULT_DPB_FLAGS);
		if (decRet != VC1DEC_OK) {
			GST_DEBUG_OBJECT(x170, "cannot create VC1 instance");
			gst_x170_free(x170);
			return FALSE;
		}
		GST_DEBUG_OBJECT(x170, "VC1 instance created, profile: %d", x170->vc1meta.profile);
		decoder = (void *) x170->vc1dec;
		pptype = PP_PIPELINED_DEC_TYPE_VC1;
		if (x170->vc1_advanced) {
			memcpy(seq_header, seq_header_buf, seq_header_len);
			decIn.pStream = (u8 *)seq_header;
			decIn.streamBusAddress = seq_header_mem_info.busAddress;

			seq_header_len -= 1;

			decIn.streamSize = seq_header_len;
			decIn.picId = x170->picNumber;
			do {
				decRet = VC1DecDecode(x170->vc1dec, &decIn, &decOut);
				GST_DEBUG_OBJECT(x170, "++Decode header result: %d, dataLeft: %d\n", decRet, decOut.dataLeft);
				if (decOut.dataLeft) {
					decIn.pStream = decOut.pStreamCurrPos;
					decIn.streamSize = decOut.dataLeft;
				}
			} while (decOut.dataLeft != 0);

			decRet = VC1DecGetInfo(x170->vc1dec, &decInfo);
			x170->width = decInfo.maxCodedWidth;
			x170->height = decInfo.maxCodedHeight;
		}
		}
		break;
	default:
		GST_DEBUG_OBJECT(x170, "Invalid codec %d", x170->codec);
		return FALSE;
	}	

	if (x170_malloc_linear(x170, x170->inbuf_size,
			       &x170->inbuf) != DWL_OK) {
		GST_DEBUG_OBJECT(x170, "cannot allocate linear input buffer");
		gst_x170_free(x170);
		return FALSE;
	}
	memset(x170->inbuf.virtualAddress, 0, x170->inbuf_size);

	if (!x170->fb_data) {
		x170->fb_data = malloc(x170->inbuf_size);
		if (!x170->fb_data) {
			GST_DEBUG_OBJECT(x170, "cannot allocate plugin input buffer");
			gst_x170_free(x170);
			return FALSE;
		}
	}

	if (x170->rotation != HT_ROTATION_0 ||
	    x170->scaling != 1.0 ||
	    x170->crop_width != 0 ||
	    x170->crop_height != 0 ||
	    x170->output != HT_OUTPUT_UYVY) {
	    	int tmpw, tmph;

		GST_DEBUG_OBJECT(x170, "enabling PP");
		if ((ppret = PPInit(&x170->pp)) != PP_OK) {
			GST_DEBUG_OBJECT(x170,
					 "cannot create PP instance (err=%d)",
					 ppret);
			gst_x170_free(x170);
			return FALSE;
		}
		if ((ppret = PPDecCombinedModeEnable(x170->pp,
						     decoder,
						     pptype)) != PP_OK) {
			GST_DEBUG_OBJECT(x170,
					 "cannot combine PP instance (err=%d)",
					 ppret);
			gst_x170_free(x170);
			return FALSE;
		}

		if (x170_malloc_linear(x170, x170->outbuf_size,
				       &x170->outbuf) != DWL_OK) {
			GST_DEBUG_OBJECT(x170, "cannot allocate linear output buffer");
			gst_x170_free(x170);
			return FALSE;
		}
		memset(x170->outbuf.virtualAddress, 0, x170->outbuf_size);

		tmpw = 100;
		if (x170->crop_x + x170->crop_width > tmpw)
			tmpw = x170->crop_x + x170->crop_width;
		if (x170->crop_y + x170->crop_height > tmpw)
			tmpw = x170->crop_y + x170->crop_height;
		tmpw += 100;
		tmph = tmpw;
	}
	else
		GST_DEBUG_OBJECT(x170, "not enabling PP");

	return TRUE;
}

static void gst_x170_update_caps(Gstx170 *x170)
{
	guint bpp, depth, red_mask, green_mask, blue_mask, endianness;

	GST_DEBUG_OBJECT(x170, "gst_x170_update_caps");

	if (x170->caps)
		 gst_caps_unref(x170->caps);
	x170->caps = NULL;

	switch (x170->output) {
	case HT_OUTPUT_UYVY:
		x170->caps = gst_caps_new_simple("video/x-raw-yuv",
						 "format", GST_TYPE_FOURCC, x170->fourcc,
						 "width", G_TYPE_INT, x170->width,
						 "height", G_TYPE_INT, x170->height,
						 "framerate", GST_TYPE_FRACTION, x170->fps_n, x170->fps_d,
						 NULL);
		return;
	case HT_OUTPUT_RGB15:
		bpp = 16;
		depth = 15;
		red_mask = (int) 0x7c00;
		green_mask = (int) 0x03e0;
		blue_mask = (int) 0x001f;
		endianness = (int) 1234;
		break;
	case HT_OUTPUT_RGB16:
		bpp = 16;
		depth = 16;
		red_mask = (int) 0xf800;
		green_mask = (int) 0x07e0;
		blue_mask = (int) 0x001f;
		endianness = (int) 1234;
		break;
	case HT_OUTPUT_RGB32:
		bpp = 32;
		depth = 24;
		red_mask = (int)   0x0000ff00;
		green_mask = (int) 0x00ff0000;
		blue_mask = (int)  0xff000000;
		endianness = (int) 4321;		
		break;
	default:
		GST_DEBUG_OBJECT(x170, "Invalid output format.");
		return;
	}
	x170->caps = gst_caps_new_simple("video/x-raw-rgb",
					 "bpp", G_TYPE_INT, bpp,
					 "depth", G_TYPE_INT, depth,
					 "red_mask", G_TYPE_INT, red_mask,
					 "green_mask", G_TYPE_INT, green_mask,
					 "blue_mask", G_TYPE_INT, blue_mask,
					 "width", G_TYPE_INT, x170->width,
					 "height", G_TYPE_INT, x170->height,
					 "framerate", GST_TYPE_FRACTION, x170->fps_n, x170->fps_d,
					 "endianness", G_TYPE_INT, endianness,
					 NULL);
}

static GstBuffer *gst_x170_read_image(Gstx170 *x170, const void *data,
				      int width, int height,
				      GstClockTime timeCode,
				      GstClockTime duration)
{
	int length;

	if (x170->pp) {
		data = x170->outbuf.virtualAddress;
		switch (x170->output) {
		case HT_OUTPUT_UYVY:
			length = x170->width * x170->height * 3 / 2;
			break;
		case HT_OUTPUT_RGB15:
		case HT_OUTPUT_RGB16:
			length = x170->width * x170->height * 2;
			break;
		case HT_OUTPUT_RGB32:
			length = x170->width * x170->height * 4;
			break;
		default:
			GST_DEBUG_OBJECT(x170, "Invalid output format.");
			return NULL;
		}

		GST_DEBUG_OBJECT(x170, "gst_x170_read_image (%dx%d), pp status %d, length %d\n",
				 x170->width, x170->height, PPGetResult(x170->pp), length);
		g_assert(PPGetResult(x170->pp) == 0);
	}
	else {
		GST_DEBUG_OBJECT(x170, "gst_x170_read_image (%dx%d): no pp\n",
				 width, height);
		length = width * height * 3 / 2;
	}

	GstBuffer *image = NULL;
	gst_pad_alloc_buffer(x170->srcpad, 0, length, x170->caps, &image);
	memcpy(GST_BUFFER_DATA(image), data, length);
	GST_BUFFER_TIMESTAMP(image) = timeCode;
	GST_BUFFER_DURATION(image) = duration;

	return image;
}

static GstBuffer *gst_x170_read_jpeg_image(Gstx170 *x170, const void *dataY, const void *dataUV,
				      	   int sizeY, int sizeUV,
					   int width, int height,
				      	   GstClockTime timeCode,
					   GstClockTime duration)
{

	if (x170->pp)
		return gst_x170_read_image(x170, NULL, width, height, timeCode, duration);

	GstBuffer *image = gst_buffer_new_and_alloc(sizeY + sizeUV);
	memcpy(GST_BUFFER_DATA(image), dataY, sizeY);
	memcpy(GST_BUFFER_DATA(image) + sizeY, dataUV, sizeUV);
	GST_BUFFER_TIMESTAMP(image) = timeCode;
	GST_BUFFER_DURATION(image) = duration;
	gst_buffer_set_caps(image, x170->caps);

	return image;
}

#ifdef USE_THREAD
static GstFlowReturn gst_x170_push_image(Gstx170 *x170, GstBuffer *image, int id)
{
	sem_wait(&x170->empty_sem);
	x170->frids[x170->write_pos] = id;
	x170->frames[x170->write_pos] = image;
	x170->write_pos = (x170->write_pos + 1) % X170_FRAME_BUFFER_SIZE;
	x170->frm_count++;
	sem_post(&x170->full_sem);

	return GST_FLOW_OK;
}
#else
static GstFlowReturn gst_x170_push_image(Gstx170 *x170, GstBuffer *image,
					 int id)
{
	int i;
	GstFlowReturn ret = GST_FLOW_OK;

	for (i = 0; i < X170_FRAME_BUFFER_SIZE; i++) {
		if (!x170->frames[i]) {
			x170->frids[i] = id;
			x170->frames[i] = image;
			break;
		}
	}
	g_assert(i < X170_FRAME_BUFFER_SIZE);
again:
	for (i = 0; i < X170_FRAME_BUFFER_SIZE; i++) {
		if (x170->frames[i] && x170->frids[i] == x170->nextid) {
			GST_DEBUG_OBJECT(x170, "gst_x170_push_image: pushing out frame %d, timestamp %lld\n",
					 x170->nextid, GST_BUFFER_TIMESTAMP(x170->frames[i]));
			ret = gst_pad_push (x170->srcpad, x170->frames[i]);
			x170->frames[i] = NULL;
			x170->nextid++;
			goto again;
		}
	}
	return ret;
}
#endif

static GstFlowReturn gst_x170_play_jpeg(Gstx170 *x170, unsigned char *buffer,
					int *len, gboolean flush)
{
	JpegDecInput decIn;
	JpegDecOutput decOut;
	JpegDecImageInfo decInfo;
	JpegDecRet decRet;
	int sizeY, sizeUV;
	GstBuffer *image;

	g_assert(*len < x170->inbuf_size);
	memcpy(x170->inbuf.virtualAddress, buffer, *len);
	decIn.streamBuffer.pVirtualAddress = (u32 *) x170->inbuf.virtualAddress;
	decIn.streamBuffer.busAddress = x170->inbuf.busAddress;
	decIn.streamLength = *len;
	decIn.bufferSize = x170->inbuf_size;
	decIn.decImageType = 0;
	decIn.sliceMbSet = 0;
	decIn.pictureBufferY.pVirtualAddress = NULL;
	decIn.pictureBufferY.busAddress = 0;
	decIn.pictureBufferCbCr.pVirtualAddress = NULL;
	decIn.pictureBufferCbCr.busAddress = 0;

	/* Get image information of the JPEG image */
	decRet = JpegDecGetImageInfo(x170->jpegdec, &decIn, &decInfo);
	if(decRet != JPEGDEC_OK) {
		GST_DEBUG_OBJECT(x170,
				"gst_x170_play_jpeg: get image info error %d\n",
				decRet);
		return GST_FLOW_ERROR;
	}

	x170->width = decInfo.outputWidth;
	x170->height = decInfo.outputHeight;
	if (x170->pp)
		gst_x170_ppsetconfig(x170,
				     decInfo.outputFormat,
				     &x170->width, &x170->height, 0);
	switch (decInfo.outputFormat) {
	case JPEGDEC_YCbCr400:
		x170->fourcc = GST_MAKE_FOURCC ('Y', '8', '0', '0');
		sizeY = sizeUV = x170->width * x170->height;
		break;
	case JPEGDEC_YCbCr420_SEMIPLANAR:
		x170->fourcc = GST_MAKE_FOURCC ('Y', 'V', '1', '2');
		sizeY = x170->width * x170->height;
		sizeUV = x170->width * x170->height / 2;
		break;
	case JPEGDEC_YCbCr422_SEMIPLANAR:
		x170->fourcc = GST_MAKE_FOURCC ('Y', '4', '2', 'B');
		sizeY = sizeUV = x170->width * x170->height;
		sizeUV = x170->width * x170->height * 2;
		break;
	case JPEGDEC_YCbCr440:
		/* XXX something better ? */
		x170->fourcc = GST_MAKE_FOURCC ('Y', '4', '2', 'B');
		sizeY = sizeUV = x170->width * x170->height;
		break;
	}
	gst_x170_update_caps(x170);

	/* Decode JPEG image */
	decRet = JpegDecDecode(x170->jpegdec, &decIn, &decOut);
	if (decRet != JPEGDEC_FRAME_READY) {
		GST_DEBUG_OBJECT(x170,
				"gst_x170_play_jpeg: decode error %d\n",
				decRet);
		return GST_FLOW_ERROR;
	}
	GST_DEBUG_OBJECT(x170, "gst_x170_play_jpeg: decoded picture %dx%d, "
			 "format %x",
			 decInfo.outputWidth,
			 decInfo.outputHeight,
			 decInfo.outputFormat);
	image = gst_x170_read_jpeg_image(x170,
					 decOut.outputPictureY.pVirtualAddress,
					 decOut.outputPictureCbCr.pVirtualAddress,
					 sizeY, sizeUV,
					 x170->width,
					 x170->height,
					 GST_SECOND,
					 GST_SECOND);
	gst_x170_push_image(x170, image, 0);
	return GST_FLOW_OK;
}

static GstFlowReturn gst_x170_play_h264(Gstx170 *x170, unsigned char *buffer,
					int *len, gboolean flush)
{
	H264DecInput decIn;
	H264DecOutput decOut;
	H264DecInfo decInfo;
	H264DecPicture decPic;
	H264DecRet decRet;
	H264DecRet infoRet;
	GstBuffer *image;
	int forceflush = 0;

	g_assert(*len < x170->inbuf_size);
	memcpy(x170->inbuf.virtualAddress, buffer, *len);
	decIn.pStream = (u8 *) x170->inbuf.virtualAddress;
	decIn.streamBusAddress = x170->inbuf.busAddress;
	decIn.dataLen = *len;

donext:
	decIn.picId = x170->picNumber;
	decRet = H264DecDecode(x170->h264dec, &decIn, &decOut);
	switch (decRet) {
	case H264DEC_HDRS_RDY:
		/* read stream info */
		infoRet = H264DecGetInfo(x170->h264dec, &decInfo);
		GST_DEBUG_OBJECT(x170,
			"gst_x170_play_h264: H264 stream, %dx%d format %x\n",
			decInfo.picWidth, decInfo.picHeight, decInfo.outputFormat);
		x170->width = decInfo.picWidth;
		x170->height = decInfo.picHeight;
		if (x170->pp)
			gst_x170_ppsetconfig(x170,
					     decInfo.outputFormat,
					     &x170->width, &x170->height, 0);
		x170->fourcc = GST_MAKE_FOURCC ('N','V','1','2');
		gst_x170_update_caps(x170);
		break;
	case H264DEC_ADVANCED_TOOLS:
		/* the decoder has to reallocate ressources */
		break;
	case H264DEC_PIC_DECODED:
		/* a picture was decoded */
		x170->picNumber++;
doflush:
		while (H264DecNextPicture(
			x170->h264dec, &decPic, forceflush) == H264DEC_PIC_RDY) {

			GST_DEBUG_OBJECT(x170, "gst_x170_play_h264: "
					       "decoded picture %d\n",
					       x170->displayNumber);
			GstClockTime timeCode = (GST_SECOND * x170->displayNumber * x170->fps_d) / x170->fps_n;
			GstClockTime duration = (GST_SECOND * x170->fps_d) / x170->fps_n;
			image = gst_x170_read_image(x170,
						    decPic.pOutputPicture,
						    decPic.picWidth,
						    decPic.picHeight,
						    timeCode,
						    duration);
			gst_x170_push_image(x170, image, x170->displayNumber);
			x170->displayNumber++;
		}
		break;
	case H264DEC_STRM_PROCESSED:
		/* input stream processed but no picture ready */
		GST_DEBUG_OBJECT(x170, "gst_x170_play_h264: H264DEC_STRM_PROCESSED\n");
		break;
	case H264DEC_STRM_ERROR:
		/* input stream processed but no picture ready */
		GST_DEBUG_OBJECT(x170, "gst_x170_play_h264: H264DEC_STRM_ERROR\n");
		break;
	default:
		/* some kind of error, decoding cannot continue */
		GST_DEBUG_OBJECT(x170,
			"gst_x170_play_h264: decode error %d\n",
			decRet);
		return GST_FLOW_ERROR;
	}

	if (decOut.dataLeft > (flush ? 0 : x170->inbuf_thresh)) {
		decIn.dataLen = decOut.dataLeft;
		decIn.pStream = decOut.pStrmCurrPos;
		decIn.streamBusAddress = decOut.strmCurrBusAddress;
		goto donext;
	}

	if (flush && !forceflush) {
		forceflush = 1;
		goto doflush;
	}

	*len = (u8 *)decOut.pStrmCurrPos - (u8 *)x170->inbuf.virtualAddress;
	return GST_FLOW_OK;
}

static GstFlowReturn gst_x170_play_mpeg2(Gstx170 *x170, unsigned char *buffer,
					 int *len, gboolean flush)
{
	Mpeg2DecInput decIn;
	Mpeg2DecOutput decOut;
	Mpeg2DecInfo decInfo;
	Mpeg2DecPicture decPic;
	Mpeg2DecRet decRet;
	Mpeg2DecRet infoRet;
	GstBuffer *image;
	int forceflush = 0;

	g_assert(*len < x170->inbuf_size);
	memcpy(x170->inbuf.virtualAddress, buffer, *len);
	decIn.pStream = (u8 *) x170->inbuf.virtualAddress;
	decIn.streamBusAddress = x170->inbuf.busAddress;
	decIn.dataLen = *len;

donext:
	decIn.picId = x170->picNumber;
	decRet = Mpeg2DecDecode(x170->mpeg2dec, &decIn, &decOut);
	switch (decRet) {
	case MPEG2DEC_HDRS_RDY:
		/* read stream info */
		infoRet = Mpeg2DecGetInfo(x170->mpeg2dec, &decInfo);
		GST_DEBUG_OBJECT(x170,
			"gst_x170_play_mpeg2: MPEG stream, %dx%d interlaced %d format %x\n",
			decInfo.frameWidth, decInfo.frameHeight,
			decInfo.interlacedSequence, decInfo.outputFormat);
		x170->width = decInfo.frameWidth;
		x170->height = decInfo.frameHeight;
		if (x170->pp)
			gst_x170_ppsetconfig(x170,
					     decInfo.outputFormat,
					     &x170->width, &x170->height,
					     decInfo.interlacedSequence);
		x170->fourcc = GST_MAKE_FOURCC ('N','V','1','2');
		gst_x170_update_caps(x170);
		break;
	case MPEG2DEC_PIC_DECODED:
		/* a picture was decoded */
		x170->picNumber++;
doflush:
		while (Mpeg2DecNextPicture(
			x170->mpeg2dec, &decPic, forceflush) == MPEG2DEC_PIC_RDY) {

			if ((decPic.fieldPicture && !decPic.firstField) ||
			     !decPic.fieldPicture) {
				GST_DEBUG_OBJECT(x170, "gst_x170_play_mpeg2: "
						 "decoded picture %d, "
						 "mpeg2 timestamp: %d:%d:%d:%d\n",
						 decPic.picId, decPic.interlaced,
						 decPic.timeCode.hours, decPic.timeCode.minutes,
						 decPic.timeCode.seconds);
				GstClockTime timeCode = (GST_SECOND * decPic.picId * x170->fps_d) / x170->fps_n;
				GstClockTime duration = (GST_SECOND * x170->fps_d) / x170->fps_n;
				image = gst_x170_read_image(x170,
							    decPic.pOutputPicture,
							    decPic.frameWidth,
							    decPic.frameHeight,
							    timeCode,
							    duration);
				gst_x170_push_image(x170, image, decPic.picId);
			}
		}
		break;
	case MPEG2DEC_STRM_PROCESSED:
		/* input stream processed but no picture ready */
		break;
	case MPEG2DEC_STRM_ERROR:
		/* input stream processed but no picture ready */
		break;
	default:
		/* some kind of error, decoding cannot continue */
		GST_DEBUG_OBJECT(x170,
			"gst_x170_play_mpeg2: decode error %d\n",
			decRet);
		return GST_FLOW_ERROR;
	}

	if (decOut.dataLeft > (flush ? 0 : x170->inbuf_thresh)) {
		decIn.dataLen = decOut.dataLeft;
		decIn.pStream = decOut.pStrmCurrPos;
		decIn.streamBusAddress = decOut.strmCurrBusAddress;
		goto donext;
	}

	if (flush && !forceflush) {
		forceflush = 1;
		goto doflush;
	}

	*len = (u8 *)decOut.pStrmCurrPos - (u8 *)x170->inbuf.virtualAddress;
	return GST_FLOW_OK;
}

static GstFlowReturn gst_x170_play_mpeg4(Gstx170 *x170, unsigned char *buffer,
					 int *len, gboolean flush)
{
	MP4DecInput decIn;
	MP4DecOutput decOut;
	MP4DecInfo decInfo;
	MP4DecPicture decPic;
	MP4DecRet decRet;
	MP4DecRet infoRet;
	int secondField = 1;
	GstBuffer *image;
	int forceflush = 0;
	char *err_name;

	g_assert(*len < x170->inbuf_size);
	memcpy(x170->inbuf.virtualAddress, buffer, *len);
	decIn.pStream = (u8 *) x170->inbuf.virtualAddress;
	decIn.streamBusAddress = x170->inbuf.busAddress;
	decIn.dataLen = *len;

donext:
	decIn.picId = x170->picNumber;
	decRet = MP4DecDecode(x170->mpeg4dec, &decIn, &decOut);
	switch (decRet) {
	case MP4DEC_HDRS_RDY:
		/* read stream info */
		infoRet = MP4DecGetInfo(x170->mpeg4dec, &decInfo);
		GST_DEBUG_OBJECT(x170,
			"gst_x170_play_mpeg4: MPEG4 stream, %dx%d, interlaced %d, format %x\n",
			decInfo.frameWidth, decInfo.frameHeight,
			decInfo.interlacedSequence, decInfo.outputFormat);
		x170->width = decInfo.frameWidth;
		x170->height = decInfo.frameHeight;
		if (x170->pp) {
			gst_x170_ppsetconfig(x170,
					     decInfo.outputFormat,
					     &x170->width, &x170->height,
					     decInfo.interlacedSequence);
		}
		x170->fourcc = GST_MAKE_FOURCC ('N','V','1','2');
		gst_x170_update_caps(x170);
		break;
	case MP4DEC_PIC_DECODED:
		/* a picture was decoded */
		x170->picNumber++;
doflush:
		while (MP4DecNextPicture(
			x170->mpeg4dec, &decPic, forceflush) == MP4DEC_PIC_RDY) {

			if ((decPic.fieldPicture && secondField) ||
			     !decPic.fieldPicture) {
				GST_DEBUG_OBJECT(x170, "gst_x170_play_mpeg4: "
						 "decoded picture %d, mpeg4 timestamp: %d:%d:%d:%d (%d)\n",
						 x170->displayNumber,
						 decPic.timeCode.hours, decPic.timeCode.minutes,
						 decPic.timeCode.seconds, decPic.timeCode.timeIncr,
						 decPic.timeCode.timeRes);
				if (decPic.fieldPicture)
					secondField = 0;

				GstClockTime timeCode = 0;
				if (decPic.timeCode.timeRes)
					timeCode = decPic.timeCode.hours * GST_SECOND * 3600 +
						   decPic.timeCode.minutes * GST_SECOND * 60 +
						   decPic.timeCode.seconds * GST_SECOND +
						   GST_SECOND * decPic.timeCode.timeIncr / decPic.timeCode.timeRes;
				else
					timeCode = (GST_SECOND * decIn.picId * x170->fps_d) / x170->fps_n;

				GstClockTime duration = GST_CLOCK_TIME_NONE;
				image = gst_x170_read_image(x170,
							    decPic.pOutputPicture,
							    decPic.frameWidth,
							    decPic.frameHeight,
							    timeCode,
							    duration);
				gst_x170_push_image(x170, image, x170->displayNumber);
				x170->displayNumber++;
			}
			else if (decPic.fieldPicture)
					secondField = 1;
		}
		break;
	case MP4DEC_NONREF_PIC_SKIPPED:
		/* Skipped non-reference picture */
		break;
	case MP4DEC_STRM_PROCESSED:
		/* input stream processed but no picture ready */
		break;
	case MP4DEC_STRM_ERROR:
		/* input stream processed but no picture ready */
		break;
	case MP4DEC_VOS_END:
		/* end of stream */
		decOut.dataLeft = 0;
		break;
	default:
		/* some kind of error, decoding cannot continue */

		switch(decRet){
		case MP4DEC_PARAM_ERROR:
		    err_name = "MP4DEC_PARAM_ERROR";
		    break;
		case MP4DEC_STRM_ERROR:
		    err_name = "MP4DEC_STRM_ERROR";
		    break;
		case MP4DEC_NOT_INITIALIZED:
		    err_name = "MP4DEC_NOT_INITIALIZED";
		    break;
		case MP4DEC_MEMFAIL:
		    err_name = "MP4DEC_MEMFAIL";
		    break;
		case MP4DEC_INITFAIL:
		    err_name = "MP4DEC_INITFAIL";
		    break;
		case MP4DEC_FORMAT_NOT_SUPPORTED:
		    err_name = "MP4DEC_FORMAT_NOT_SUPPORTED";
		    break;
		case MP4DEC_STRM_NOT_SUPPORTED:
		    err_name = "MP4DEC_STRM_NOT_SUPPORTED";
		    break;
		case MP4DEC_HW_RESERVED:
		    err_name = "MP4DEC_HW_RESERVED";
		    break;
		case MP4DEC_HW_TIMEOUT:
		    err_name = "MP4DEC_HW_TIMEOUT";
		    break;
		case MP4DEC_HW_BUS_ERROR:
		    err_name = "MP4DEC_HW_BUS_ERROR";
		    break;
		case MP4DEC_SYSTEM_ERROR:
		    err_name = "MP4DEC_SYSTEM_ERROR";
		    break;
		case MP4DEC_DWL_ERROR:
		    err_name = "MP4DEC_DWL_ERROR";
		    break;
		default:
		    err_name = "MP4DEC_UNKNOWN_ERROR";
		}
		GST_DEBUG_OBJECT(x170,
			"gst_x170_play_mpeg4: decode error (%d) %s\n",
			decRet, err_name);
		return GST_FLOW_ERROR;
	}

	if (decOut.dataLeft > (flush ? 0 : x170->inbuf_thresh)) {
		decIn.dataLen = decOut.dataLeft;
		decIn.pStream = decOut.pStrmCurrPos;
		decIn.streamBusAddress = decOut.strmCurrBusAddress;
		goto donext;
	}

	if (flush && !forceflush) {
		forceflush = 1;
		goto doflush;
	}

	*len = (u8 *)decOut.pStrmCurrPos - (u8 *)x170->inbuf.virtualAddress;
	return GST_FLOW_OK;
}

static GstFlowReturn gst_x170_play_vc1(Gstx170 *x170, unsigned char *buffer,
				       int *len, gboolean flush)
{
	VC1DecInput decIn;
	VC1DecOutput decOut;
	VC1DecInfo decInfo;
	VC1DecPicture decPic;
	VC1DecRet decRet;
	VC1DecRet infoRet;
	GstBuffer *image;
	int forceflush = 0;

	// Use vc1meta_inited to indicate whether the PP has been initialized for
	// VC1
	if (!x170->vc1meta_inited) {
		x170->vc1meta_inited = 1;
		
		infoRet = VC1DecGetInfo(x170->vc1dec, &decInfo);
		if (x170->pp)
			gst_x170_ppsetconfig(x170,
						 decInfo.outputFormat,
						 &x170->width, &x170->height,
						 decInfo.interlacedSequence);
		x170->fourcc = GST_MAKE_FOURCC ('N','V','1','2');
		gst_x170_update_caps(x170);

	}

	/* late VC1 init, because metadata needs to be available first */
	if (!x170->vc1dec) {
		x170->vc1_advanced = ( (buffer[0] == 0x00) && (buffer[1] == 0x00) && (buffer[2] == 0x01) );
		if (x170->vc1_advanced) {
			/* advanced */
			g_assert(*len >= 3);	/* max size of metadata */
			x170->vc1meta.profile = 8;
			GST_DEBUG_OBJECT(x170, "gst_x170_play_vc1: VC1 advanced stream\n");
			x170->vc1meta_inited = 1;
			gst_x170_alloc(x170);
		}
		else {
			/* simple or main */
			g_assert(*len >= 44);	/* max size of metadata */
			unsigned long num_frames = (((unsigned long)buffer[0])) |
						   (((unsigned long)buffer[1]) << 8) |
						   (((unsigned long)buffer[2]) << 16);
			unsigned long v2 = (unsigned long)buffer[3];
			if (v2 & 0x40)
				x170->vc1_v2 = 1;
			else
				x170->vc1_v2 = 0;
			unsigned long height = (((unsigned long)buffer[12])) |
					       (((unsigned long)buffer[13]) << 8) |
					       (((unsigned long)buffer[14]) << 16) |
					       (((unsigned long)buffer[15]) << 24);
			unsigned long width = (((unsigned long)buffer[16])) |
					      (((unsigned long)buffer[17]) << 8) |
					      (((unsigned long)buffer[18]) << 16) |
					      (((unsigned long)buffer[19]) << 24);
			x170->vc1meta.maxCodedHeight = height;
			x170->vc1meta.maxCodedWidth = width;
			GST_DEBUG_OBJECT(x170, "gst_x170_play_vc1: VC1 main/simple stream, "
					 "frames=%lu, version=%lu, height=%lu, width=%lu\n",
					 num_frames,(unsigned long)x170->vc1_v2 + 1, height, width);

			decRet = VC1DecUnpackMetaData(buffer + 8, 4, &x170->vc1meta);
			if (decRet != VC1DEC_OK) {
				GST_DEBUG_OBJECT(x170,
					"gst_x170_play_vc1: unpack error %d\n",
					decRet);
				return GST_FLOW_ERROR;
			}
			x170->vc1meta_inited = 1;

			buffer += (5 + 4 * x170->vc1_v2) * 4 + 4 + 4 * x170->vc1_v2;
			*len -= (5 + 4 * x170->vc1_v2) * 4 + 4 + 4 * x170->vc1_v2;

			gst_x170_alloc(x170);

			x170->width = width;
			x170->height = height;
			if (x170->pp)
				gst_x170_ppsetconfig(x170,
						     VC1DEC_SEMIPLANAR_YUV420,
						     &x170->width, &x170->height, 0);
			x170->fourcc = GST_MAKE_FOURCC ('N','V','1','2');
			gst_x170_update_caps(x170);
		}
	}

	g_assert(*len < x170->inbuf_size);
	if (x170->vc1_advanced) {
		gchar *p = (gchar *)(x170->inbuf.virtualAddress);
		p[0] = 0x00;
		p[1] = 0x00;
		p[2] = 0x01;
		p[3] = 0x0d;
		memcpy(((gchar *)(x170->inbuf.virtualAddress)) + 4, buffer, *len);		
	} else
	memcpy(x170->inbuf.virtualAddress, buffer, *len);
	decIn.pStream = (u8 *) x170->inbuf.virtualAddress;
	decIn.streamBusAddress = x170->inbuf.busAddress;

	if (x170->vc1_advanced) {
		decIn.streamSize = *len + 4;
	} else {
		decIn.streamSize = *len;
	}

	*len = 0;

donext:
	decIn.picId = x170->picNumber;
	decRet = VC1DecDecode(x170->vc1dec, &decIn, &decOut);
	switch (decRet) {
	case VC1DEC_HDRS_RDY:
		/* read stream info */
		infoRet = VC1DecGetInfo(x170->vc1dec, &decInfo);
		GST_DEBUG_OBJECT(x170,
			"gst_x170_play_vc1: VC1 advanced stream, %dx%d, interlaced %d format %x\n",
			decInfo.maxCodedWidth, decInfo.maxCodedHeight,
			decInfo.interlacedSequence, decInfo.outputFormat);
		x170->width = decInfo.maxCodedWidth;
		x170->height = decInfo.maxCodedHeight;
		if (x170->pp)
			gst_x170_ppsetconfig(x170,
					     decInfo.outputFormat,
					     &x170->width, &x170->height,
					     decInfo.interlacedSequence);
		x170->fourcc = GST_MAKE_FOURCC ('N','V','1','2');
		gst_x170_update_caps(x170);
		break;
	case VC1DEC_PIC_DECODED:
		/* a picture was decoded */
		x170->picNumber++;
doflush:
		while (VC1DecNextPicture(
			x170->vc1dec, &decPic, forceflush) == VC1DEC_PIC_RDY) {

			if ((decPic.fieldPicture && !decPic.firstField) ||
			     !decPic.fieldPicture) {
				GST_DEBUG_OBJECT(x170, "gst_x170_play_vc1: "
						 "decoded picture %d (%dx%d) (%dx%d)\n",
						 x170->displayNumber,
						 decPic.frameWidth, decPic.frameHeight,
						 decPic.codedWidth, decPic.codedHeight);
				GstClockTime timeCode = (GST_SECOND * x170->displayNumber * x170->fps_d) / x170->fps_n;
				GstClockTime duration = (GST_SECOND * x170->fps_d) / x170->fps_n;
				image = gst_x170_read_image(x170,
							    decPic.pOutputPicture,
							    decPic.codedWidth,
							    decPic.codedHeight,
							    timeCode,
							    duration);
				gst_x170_push_image(x170, image, x170->displayNumber);
				x170->displayNumber++;
			}
		}
		break;
	case VC1DEC_STRM_PROCESSED:
		GST_DEBUG_OBJECT(x170,"gst_x170_play_vc1: VC1DEC_STRM_PROCESSED\n");
		/* input stream processed but no picture ready */
		break;
	case VC1DEC_STRM_ERROR:
		GST_DEBUG_OBJECT(x170,"gst_x170_play_vc1: VC1DEC_STRM_ERROR\n");
		/* input stream processed but no picture ready */
		break;
	case VC1DEC_END_OF_SEQ:
		GST_DEBUG_OBJECT(x170,"gst_x170_play_vc1: VC1DEC_END_OF_SEQ\n");
		/* end of sequence */
		decOut.dataLeft = 0;
		break;
	default:
		/* some kind of error, decoding cannot continue */
		GST_DEBUG_OBJECT(x170,
			"gst_x170_play_vc1: decode error %d\n",
			decRet);
		return GST_FLOW_ERROR;
	}

	if (decOut.dataLeft > 0) {
		decIn.streamSize = decOut.dataLeft;
		decIn.streamBusAddress += (decOut.pStreamCurrPos - decIn.pStream);
		decIn.pStream = decOut.pStreamCurrPos;
		goto donext;
	}

	if (flush && !forceflush) {
		forceflush = 1;
		goto doflush;
	}

	*len = (u8 *)decOut.pStreamCurrPos - (u8 *)x170->inbuf.virtualAddress;

	if (x170->vc1_advanced)
		*len = (*len) - 4;
	
	return GST_FLOW_OK;
}

static GstFlowReturn gst_x170_play(Gstx170 *x170, unsigned char *buffer,
				   int *len, gboolean flush)
{
	GstFlowReturn ret = GST_FLOW_OK;

	GST_DEBUG_OBJECT(x170, "gst_x170_play (%d bytes)", *len);

	switch (x170->codec) {
	case HT_AUTO:
		GST_DEBUG_OBJECT(x170, "gst_x170_play: codec not yet set");
		return GST_FLOW_ERROR;

	case HT_IMAGE_JPEG:
		ret = gst_x170_play_jpeg(x170, buffer, len, flush);
		break;

	case HT_VIDEO_H264:
		ret = gst_x170_play_h264(x170, buffer, len, flush);
		break;

	case HT_VIDEO_MPEG2:
		ret = gst_x170_play_mpeg2(x170, buffer, len, flush);
		break;

	case HT_VIDEO_MPEG4:
		ret = gst_x170_play_mpeg4(x170, buffer, len, flush);
		break;

	case HT_VIDEO_VC1:
		ret = gst_x170_play_vc1(x170, buffer, len, flush);
		break;
	}

	return ret;
}

static GstFlowReturn gst_x170_chain(GstPad *pad, GstBuffer *buffer)
{
	Gstx170 *x170 = GST_X170(gst_pad_get_parent(pad));
	GstFlowReturn ret = GST_FLOW_OK;
	int len;

	GST_DEBUG_OBJECT(x170, "gst_x170_chain");

	g_assert(x170->fb_data);
	g_assert(x170->fb_len + GST_BUFFER_SIZE(buffer) < x170->inbuf_size);
	memcpy(x170->fb_data + x170->fb_len,
	       GST_BUFFER_DATA(buffer), GST_BUFFER_SIZE(buffer));

	GST_DEBUG_OBJECT(x170, "gst_x170_chain: GST_BUFFER_SIZE(buffer) = %08X\n, x170->fb_len = %08X\n", GST_BUFFER_SIZE(buffer), x170->fb_len);

	x170->fb_len += GST_BUFFER_SIZE(buffer);
	gst_buffer_unref(buffer);

	GST_DEBUG_OBJECT(x170, "gst_x170_chain: x170->fb_len = %08X, x170->inbuf_thresh = %08X\n", x170->fb_len , x170->inbuf_thresh);

	/*
	 * JPEG decoding needs to have the image length available,
	 *  so wait for the end of the data and decode it in one step.
	 */
	if (x170->codec != HT_IMAGE_JPEG)
		if ((x170->fb_len > x170->inbuf_thresh) || (x170->codec == HT_VIDEO_VC1)) {
			len = x170->fb_len;
			ret = gst_x170_play(x170, x170->fb_data, &len, FALSE);
			memmove(x170->fb_data, x170->fb_data + len, x170->fb_len - len);
			x170->fb_len -= len;
		}

	return ret;
}

static gboolean gst_x170_event(GstPad *pad, GstEvent *event)
{
	Gstx170 *x170 = GST_X170(gst_pad_get_parent(pad));

	switch (GST_EVENT_TYPE (event)) {
	case GST_EVENT_EOS:
		/* end-of-stream, flush the decoder buffers */
		gst_x170_play(x170, x170->fb_data, &x170->fb_len, TRUE);
		break;
	default:
		break;
	}

	return gst_pad_push_event (x170->srcpad, event);
}

static GstStateChangeReturn gst_x170_change_state(GstElement *element,
						  GstStateChange transition)
{
	GstStateChangeReturn result;
	Gstx170 *x170 = GST_X170(element);

	switch (transition) {
	case GST_STATE_CHANGE_NULL_TO_READY:
		if (gst_x170_alloc(x170) == FALSE)
			return GST_STATE_CHANGE_FAILURE;
		GST_DEBUG_OBJECT(x170, "state changed from NULL to READY");
		break;
	case GST_STATE_CHANGE_READY_TO_PAUSED:
		GST_DEBUG_OBJECT(x170, "state changed from READY to PAUSED");
		break;
	default:
		break;
	}

	result = GST_ELEMENT_CLASS(parent_class)->change_state(element,
							       transition);

	switch (transition) {
	case GST_STATE_CHANGE_PAUSED_TO_READY:
		GST_DEBUG_OBJECT(x170, "state changed from PAUSED to READY");
		break;
	case GST_STATE_CHANGE_READY_TO_NULL:
		gst_x170_free(x170);
		GST_DEBUG_OBJECT(x170, "state changed from READY to NULL");
		break;
	default:
		break;
	}

	return result;
}

static GType gst_x170_get_codec_type(void)
{
	static GType x170_codec_type;

	if (!x170_codec_type) {
		static GEnumValue codec_types[] = {
			{HT_AUTO,        "AUTO",  "obtained from GStreamer"},
			{HT_IMAGE_JPEG,  "JPEG",  "JPEG raw bit stream"},
			{HT_VIDEO_H264,  "H264",  "H264 raw bit stream"},
			{HT_VIDEO_MPEG2, "MPEG2", "MPEG2 raw bit stream"},
			{HT_VIDEO_MPEG4, "MPEG4", "MPEG4 raw bit stream"},
			{HT_VIDEO_VC1,   "VC1",   "VC1 raw bit stream"},
			{0, NULL, NULL}
		};

		x170_codec_type = g_enum_register_static("X170CodecName",
							 codec_types);
	}

	return x170_codec_type;
}

static GType gst_x170_get_rotation_type(void)
{
	static GType x170_rotation_type;

	if (!x170_rotation_type) {
		static GEnumValue rotation_types[] = {
			{HT_ROTATION_0,     "0",   "0 degrees clockwise"},
			{HT_ROTATION_90,   "90",  "90 degrees clockwise"},
			{HT_ROTATION_180, "180", "180 degrees clockwise"},
			{HT_ROTATION_270, "270", "270 degrees clockwise"},
			{0, NULL, NULL}
		};

		x170_rotation_type = g_enum_register_static("X170RotationType",
							    rotation_types);
	}

	return x170_rotation_type;
}

static GType gst_x170_get_output_type(void)
{
	static GType x170_output_type;

	if (!x170_output_type) {
		static GEnumValue output_types[] = {
			{HT_OUTPUT_UYVY,  "UYVY",  "UYVY video format"},
			{HT_OUTPUT_RGB15, "RGB15", "RGB15 (555) video format"},
			{HT_OUTPUT_RGB16, "RGB16", "RGB16 (565) video format"},
			{HT_OUTPUT_RGB32, "RGB32", "RGB32 video format"},
			{0, NULL, NULL}
		};

		x170_output_type = g_enum_register_static("X170OutputType",
							  output_types);
	}

	return x170_output_type;
}

static GstCaps *gst_x170_sink_getcaps(GstPad *pad)
{
	Gstx170 *x170 = GST_X170(GST_PAD_PARENT(pad));

	GST_DEBUG_OBJECT(x170, "gst_x170_sink_getcaps");

	switch (x170->codec) {
	case HT_AUTO:
	     return gst_static_caps_get(&sink_template.static_caps);
	case HT_IMAGE_JPEG:
	     return gst_caps_new_simple("image/jpeg", NULL);
	case HT_VIDEO_H264:
	     return gst_caps_new_simple("video/x-h264", NULL);
	case HT_VIDEO_MPEG4: {
	     GstCaps *all, *one;
	     all = gst_caps_new_simple("video/mpeg",
					"mpegversion", G_TYPE_INT, 4,
					"systemstream", G_TYPE_BOOLEAN, FALSE,
					NULL);
	     one = gst_caps_new_simple("video/x-xvid",
					NULL);
	     gst_caps_append(all, one);
	     one = gst_caps_new_simple("video/x-divx",
					"divxversion", G_TYPE_INT, 5,
					NULL);
	     gst_caps_append(all, one);
	     return all;
	}
	case HT_VIDEO_VC1:
	     return gst_caps_new_simple("video/x-wmv",
					"wmvversion", G_TYPE_INT, 3,
					NULL);
	case HT_VIDEO_MPEG2:
	     return gst_caps_new_simple("video/mpeg",
					"mpegversion", G_TYPE_INT, 2,
					"systemstream", G_TYPE_BOOLEAN, FALSE,
					NULL);
	}
	return NULL;
}

static gboolean gst_x170_sink_setcaps(GstPad *pad, GstCaps *caps)
{
	Gstx170 *x170 = GST_X170(GST_PAD_PARENT(pad));
	GstStructure *structure;
	gint numerator, denominator, width, height;
	const gchar *codec;

	GST_DEBUG_OBJECT(x170, "gst_x170_sink_setcaps");

	structure = gst_caps_get_structure (caps, 0);
	codec = gst_structure_get_name(structure);

	gst_x170_free(x170);

	/* default values */
	x170->fps_n = 25;
	x170->fps_d = 1;

	if (!strcmp(codec, "image/jpeg")) {
		x170->codec = HT_IMAGE_JPEG;
		x170->fps_n = 1;
		x170->fps_d = 1;
		GST_DEBUG_OBJECT(x170, "gst_x170_sink_setcaps: setting codec to JPEG");
	}
	else if (!strcmp(codec, "video/x-h264")) {
		x170->codec = HT_VIDEO_H264;
		GST_DEBUG_OBJECT(x170, "gst_x170_sink_setcaps: setting codec to H264");
	}
	else if (!strcmp(codec, "video/mpeg")) {
		gint version;
		gst_structure_get_int (structure, "mpegversion", &version);
		if ((version == 2) || (version == 1))
			x170->codec = HT_VIDEO_MPEG2;
		else{
			x170->codec = HT_VIDEO_MPEG4;
			x170->subtype = MP4DEC_MPEG4;
		}
		GST_DEBUG_OBJECT(x170, "gst_x170_sink_setcaps: setting codec to MPEG%d", version);
	}
	else if (!strcmp(codec, "video/x-xvid")) {
		x170->codec = HT_VIDEO_MPEG4;
		x170->subtype = MP4DEC_MPEG4;//TODO check
		GST_DEBUG_OBJECT(x170, "gst_x170_sink_setcaps: setting codec to MPEG4 (xvid)");
	}
	else if (!strcmp(codec, "video/x-divx")) {
		x170->codec = HT_VIDEO_MPEG4;
		x170->subtype = MP4DEC_CUSTOM_1;
		GST_DEBUG_OBJECT(x170, "gst_x170_sink_setcaps: setting codec to MPEG4 (divx)");
	}
	else if (!strcmp(codec, "video/x-wmv")) {
		GstBuffer *codec_data_buffer;
		const GValue *codec_data;
		guint fourcc = 0;
		guint8 *buf;
		guint size;

		codec_data = gst_structure_get_value(structure, "codec_data");
		if (codec_data != NULL) {
			GST_DEBUG_OBJECT(x170, ".. Get codec_data from demuxer: ");
			codec_data_buffer = gst_value_get_buffer(codec_data);
			size = GST_BUFFER_SIZE(codec_data_buffer);
			buf  = GST_BUFFER_DATA(codec_data_buffer);
			
			/* Discard the 1st byte if it's not ZERO.
			 * It seems On2 can correctly handle the situation if the 1st byte
			 * is not zero, but anyway let it be at the moment.
			 */
			if (buf[0] != 0) {
				memcpy(seq_header_buf, buf + 1, size - 1);
				seq_header_len = size - 1;
			} else {
				memcpy(seq_header_buf, buf, size);
				seq_header_len = size;
			}
		} else {
			return FALSE;
		}

		if (fourcc == GST_MAKE_FOURCC('W', 'V', 'C', '1')) {
			GST_DEBUG_OBJECT(x170, "VC1 Advanced profile detected!\n");
			x170->vc1_advanced = 1;
			x170->vc1meta.profile = 8;
		} else {
			GST_DEBUG_OBJECT(x170, "VC1 Simple/Main profile detected!\n");
			if (VC1DecUnpackMetaData(buf, size, &x170->vc1meta) != VC1DEC_OK) {
				GST_DEBUG_OBJECT(x170, "Failed to Unpack meta data\n");
			}
			GST_DEBUG_OBJECT(x170, "OK Unpacking meta data\n");

			if (gst_structure_get_int (structure, "width", &width)) {
				x170->width = width;
				GST_DEBUG_OBJECT(x170, "gst_x170_sink_setcaps: setting width to %d",
						 width);
			}

			if (gst_structure_get_int (structure, "height", &height)) {
				x170->height = height;
				GST_DEBUG_OBJECT(x170, "gst_x170_sink_setcaps: setting height to %d",
						 height);
			}
			x170->vc1meta.maxCodedHeight = height;
			x170->vc1meta.maxCodedWidth = width;
			GST_DEBUG_OBJECT(x170, "maxCodedHeight: %d, maxCodedWidth: %d\n", height, width);

			x170->vc1_advanced = 0;
		}
		x170->codec = HT_VIDEO_VC1;
		GST_DEBUG_OBJECT(x170, "gst_x170_sink_setcaps: setting codec to VC1");
	}

	if (gst_structure_get_fraction (structure, "framerate", &numerator, &denominator)) {
		x170->fps_n = numerator;
		x170->fps_d = denominator;
		GST_DEBUG_OBJECT(x170, "gst_x170_sink_setcaps: setting framerate to %d/%d",
				 x170->fps_n, x170->fps_d);
	}

	if (gst_structure_get_int (structure, "width", &width)) {
		x170->width = width;
		GST_DEBUG_OBJECT(x170, "gst_x170_sink_setcaps: setting width to %d",
				 width);
	}

	if (gst_structure_get_int (structure, "height", &height)) {
		x170->height = height;
		GST_DEBUG_OBJECT(x170, "gst_x170_sink_setcaps: setting height to %d",
				 height);
	}

	return gst_x170_alloc(x170);
}


static GstCaps *gst_x170_src_getcaps(GstPad *pad)
{
	Gstx170 *x170 = GST_X170(GST_PAD_PARENT(pad));

	GST_DEBUG_OBJECT(x170, "gst_x170_src_getcaps");

	if (x170->caps) {
		GST_DEBUG_OBJECT(x170, "x170 caps returned %" GST_PTR_FORMAT,
				x170->caps);
		return gst_caps_copy (x170->caps);
	}
	else {
        	GST_DEBUG_OBJECT(x170, "x170 caps returned %" GST_PTR_FORMAT,
				&src_template.static_caps);
		return gst_static_caps_get(&src_template.static_caps);
	}
//		return gst_caps_copy (gst_pad_get_pad_template_caps (pad));

}

static gboolean gst_x170_src_setcaps(GstPad *pad, GstCaps *caps)
{
	Gstx170 *x170 = GST_X170(GST_PAD_PARENT(pad));
	GstCaps *intersection;

	GST_DEBUG_OBJECT(x170, "gst_x170_src_setcaps");
        
	GST_DEBUG_OBJECT(x170, "intersection requested %" GST_PTR_FORMAT,
			                         caps);

	intersection = gst_caps_intersect(gst_pad_get_caps(pad),
					  caps);

        GST_DEBUG_OBJECT(x170, "gst_x170_src_setcaps");
	GST_DEBUG_OBJECT(x170, "intersection returned %" GST_PTR_FORMAT,
			 intersection);

	if (gst_caps_is_empty(intersection)) {
		GST_DEBUG_OBJECT(x170, "caps empty error");
		return FALSE;
	}

	gst_caps_unref(intersection);

	return TRUE;
}

static void gst_x170_set_property(GObject *object, guint prop_id,
				  const GValue *value,
				  GParamSpec *pspec)
{
	Gstx170 *x170 = GST_X170(object);

	GST_DEBUG_OBJECT(x170, "gst_x170_set_property (%d)", prop_id);

	switch (prop_id) {
	case ARG_CODEC:
		x170->codec = g_value_get_enum(value);
		if (x170->codec == HT_IMAGE_JPEG)
			x170->fps_n = 1;
		else
			x170->fps_n = 25;
		x170->fps_d = 1;
		GST_DEBUG_OBJECT(x170, "codec set to %d", x170->codec);
		break;
	case ARG_ROTATION:
		x170->rotation = g_value_get_enum(value);
		GST_DEBUG_OBJECT(x170, "rotation set to %d", x170->rotation);
		break;
	case ARG_SCALING:
		x170->scaling = g_value_get_float(value);
		GST_DEBUG_OBJECT(x170, "scaling set to %f", x170->scaling);
		break;
	case ARG_OUTPUT_WIDTH:
		x170->output_width = g_value_get_uint(value);
		GST_DEBUG_OBJECT(x170, "output width set to %d", x170->output_width);
	case ARG_OUTPUT_HEIGHT:
		x170->output_height = g_value_get_uint(value);
		GST_DEBUG_OBJECT(x170, "output height set to %d", x170->output_height);
	case ARG_CROP_X:
		x170->crop_x = (g_value_get_uint(value) + 15) & ~15;
		GST_DEBUG_OBJECT(x170, "crop_x set to %d", x170->crop_x);
		break;
	case ARG_CROP_Y:
		x170->crop_y = (g_value_get_uint(value) + 15) & ~15;
		GST_DEBUG_OBJECT(x170, "crop_y set to %d", x170->crop_y);
		break;
	case ARG_CROP_WIDTH:
		x170->crop_width = (g_value_get_uint(value) + 15) & ~15;
		GST_DEBUG_OBJECT(x170, "crop_width set to %d",
				 x170->crop_width);
		break;
	case ARG_CROP_HEIGHT:
		x170->crop_height = (g_value_get_uint(value) + 15) & ~15;
		GST_DEBUG_OBJECT(x170, "crop_height set to %d",
				 x170->crop_height);
		break;
	case ARG_OUTPUT:
		x170->output = g_value_get_enum(value);
		GST_DEBUG_OBJECT(x170, "output set to %d", x170->output);
		break;
	case ARG_INBUF_SIZE:
		x170->inbuf_size = g_value_get_uint(value);
		GST_DEBUG_OBJECT(x170, "inbuf_size set to %d", x170->inbuf_size);
		break;
	case ARG_INBUF_THRESH:
		x170->inbuf_thresh = g_value_get_uint(value);
		GST_DEBUG_OBJECT(x170, "inbuf_thresh set to %d", x170->inbuf_thresh);
		break;
	case ARG_OUTBUF_SIZE:
		x170->outbuf_size = g_value_get_uint(value);
		GST_DEBUG_OBJECT(x170, "outbuf_size set to %d", x170->outbuf_size);
		break;
	default:
		G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
		GST_DEBUG_OBJECT(x170, "invalid property id");
		break;
	}
}

static void gst_x170_get_property(GObject *object, guint prop_id,
				  GValue *value,
				  GParamSpec *pspec)
{
	Gstx170 *x170 = GST_X170(object);

	GST_DEBUG_OBJECT(x170, "gst_x170_get_property (%d)", prop_id);

	switch (prop_id) {
	case ARG_CODEC:
		g_value_set_enum(value, x170->codec);
		break;
	case ARG_ROTATION:
		g_value_set_enum(value, x170->rotation);
		break;
	case ARG_SCALING:
		g_value_set_float(value, x170->scaling);
		break;
	case ARG_OUTPUT_WIDTH:
		g_value_set_uint(value, x170->output_width);
		break;
	case ARG_OUTPUT_HEIGHT:
		g_value_set_uint(value, x170->output_height);
		break;
	case ARG_CROP_X:
		g_value_set_uint(value, x170->crop_x);
		break;
	case ARG_CROP_Y:
		g_value_set_uint(value, x170->crop_y);
		break;
	case ARG_CROP_WIDTH:
		g_value_set_uint(value, x170->crop_width);
		break;
	case ARG_CROP_HEIGHT:
		g_value_set_uint(value, x170->crop_height);
		break;
	case ARG_OUTPUT:
		g_value_set_enum(value, x170->output);
		break;
	case ARG_INBUF_SIZE:
		g_value_set_uint(value, x170->inbuf_size);
		break;
	case ARG_INBUF_THRESH:
		g_value_set_uint(value, x170->inbuf_thresh);
		break;
	case ARG_OUTBUF_SIZE:
		g_value_set_uint(value, x170->outbuf_size);
		break;
	default:
		G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
		GST_DEBUG_OBJECT(x170, "invalid property id");
		break;
	}
}

static void gst_x170_base_init(gpointer gclass)
{
	GstElementClass *element_class = GST_ELEMENT_CLASS(gclass);

	gst_element_class_add_pad_template(
		element_class,
		gst_static_pad_template_get(&sink_template));
	gst_element_class_add_pad_template(
		element_class,
		gst_static_pad_template_get(&src_template));
	gst_element_class_set_details(element_class, &gstx170_details);
}

static void gst_x170_class_init(Gstx170Class *klass)
{
	GObjectClass *object_class = (GObjectClass *) (klass);
	GstElementClass *gstelement_class = (GstElementClass *) klass;

	parent_class = g_type_class_ref(GST_TYPE_ELEMENT);
	gstelement_class->change_state = gst_x170_change_state;
	object_class->set_property = gst_x170_set_property;
	object_class->get_property = gst_x170_get_property;

	g_object_class_install_property(
		object_class, ARG_CODEC,
		g_param_spec_enum("codec", "codec",
				  "Set codec type",
				  GST_TYPE_X170_CODEC,
				  0,
				  G_PARAM_READWRITE));
	g_object_class_install_property(
		object_class, ARG_ROTATION,
		g_param_spec_enum("rotation", "rotation",
				  "Rotate output",
				  GST_TYPE_X170_ROTATION,
				  0,
				  G_PARAM_READWRITE));
	g_object_class_install_property(
		object_class, ARG_SCALING,
		g_param_spec_float("scaling", "scaling",
				   "Scale output",
				   0.0, 3.0, 0.0,
				   G_PARAM_READWRITE));
	g_object_class_install_property(
		object_class, ARG_OUTPUT_WIDTH,
		g_param_spec_uint("output_width", "output_width",
				  "Width output",
				  16, 1280, DEFAULT_WIDTH,
				  G_PARAM_READWRITE));
	g_object_class_install_property(
		object_class, ARG_OUTPUT_HEIGHT,
		g_param_spec_uint("output_height", "output_height",
				  "Height output",
				  16, 720, DEFAULT_HEIGHT,
				  G_PARAM_READWRITE));
	g_object_class_install_property(
		object_class, ARG_CROP_X,
		g_param_spec_uint("crop_x", "crop_x",
				  "Crop output (x coordinate)",
				  0, G_MAXUINT32, 0,
				  G_PARAM_READWRITE));
	g_object_class_install_property(
		object_class, ARG_CROP_Y,
		g_param_spec_uint("crop_y", "crop_y",
				  "Crop output (y coordinate)",
				  0, G_MAXUINT32, 0,
				  G_PARAM_READWRITE));
	g_object_class_install_property(
		object_class, ARG_CROP_WIDTH,
		g_param_spec_uint("crop_width", "crop_width",
				  "Crop output (width)",
				  0, G_MAXUINT32, 0,
				  G_PARAM_READWRITE));
	g_object_class_install_property(
		object_class, ARG_CROP_HEIGHT,
		g_param_spec_uint("crop_height", "crop_height",
				  "Crop output (height)",
				  0, G_MAXUINT32, 0,
				  G_PARAM_READWRITE));
	g_object_class_install_property(
		object_class, ARG_OUTPUT,
		g_param_spec_enum("output", "output",
				  "Output video format",
				  GST_TYPE_X170_OUTPUT,
				  0,
				  G_PARAM_READWRITE));
	g_object_class_install_property(
		object_class, ARG_INBUF_SIZE,
		g_param_spec_uint("inbuf_size", "inbuf_size",
				  "Size of the VDEC input buffer",
				  0, G_MAXUINT32, GSTX170_INBUF_SIZE,
				  G_PARAM_READWRITE));
	g_object_class_install_property(
		object_class, ARG_INBUF_THRESH,
		g_param_spec_uint("inbuf_thresh", "inbuf_thresh",
				  "Buffering threshold of the VDEC input buffer",
				  0, G_MAXUINT32, GSTX170_INBUF_THRESH,
				  G_PARAM_READWRITE));
	g_object_class_install_property(
		object_class, ARG_OUTBUF_SIZE,
		g_param_spec_uint("outbuf_size", "outbuf_size",
				  "Size of the VDEC output buffer",
				  0, G_MAXUINT32, GSTX170_OUTBUF_SIZE,
				  G_PARAM_READWRITE));
}

/*
gboolean gst_x170_src_acceptcaps(GstPad *pad, GstCaps *caps)
{
	Gstx170 *x170 = GST_X170(GST_PAD_PARENT(pad));

	GST_DEBUG_OBJECT(x170, "gst_x170_src_acceptcaps %" GST_PTR_FORMAT "\n", caps);
	return TRUE;
}
*/

static void gst_x170_init(Gstx170 *x170, Gstx170Class *gclass)
{
	int i;

	GST_DEBUG_OBJECT(x170, "gst_x170_init");

#ifdef USE_THREAD
	GST_DEBUG_OBJECT(x170, "Using Multi-Threaded version\n");
#endif

	x170->sinkpad = gst_pad_new_from_static_template(&sink_template, "sink");
	gst_element_add_pad(GST_ELEMENT(x170), x170->sinkpad);
	gst_pad_set_chain_function(x170->sinkpad, gst_x170_chain);
	gst_pad_set_event_function(x170->sinkpad, gst_x170_event);

	gst_pad_set_getcaps_function(x170->sinkpad, gst_x170_sink_getcaps);
	gst_pad_set_setcaps_function(x170->sinkpad, gst_x170_sink_setcaps);

	x170->srcpad = gst_pad_new_from_static_template(&src_template, "src");
	gst_element_add_pad(GST_ELEMENT(x170), x170->srcpad);

	gst_pad_set_getcaps_function(x170->srcpad, gst_x170_src_getcaps);
	gst_pad_set_setcaps_function(x170->srcpad, gst_x170_src_setcaps);
//	gst_pad_set_acceptcaps_function (x170->srcpad, gst_x170_src_acceptcaps);

	x170->fb_data = NULL;
	x170->fb_len = 0;

	x170->caps = NULL;
	x170->fourcc = 0;
	x170->width = 0;
	x170->height = 0;
	x170->fps_n = 25;
	x170->fps_d = 1;

	x170->nextid = 0;
	for (i = 0; i < X170_FRAME_BUFFER_SIZE; i++) {
		x170->frids[i] = 0;
		x170->frames[i] = NULL;
	}

	x170->codec = HT_AUTO;
	x170->rotation = HT_ROTATION_0;
	x170->scaling = 1.0;
	x170->output_width  = DEFAULT_WIDTH;
	x170->output_height = DEFAULT_HEIGHT;
	x170->crop_x = 0;
	x170->crop_y = 0;
	x170->crop_width = 0;
	x170->crop_height = 0;
	/* for the time being we skip the color convertion -> output is RGB16.
	 * It should be set to HT_OUTPUT_UYVY.
	 */
	x170->output = HT_OUTPUT_RGB16;
	x170->fd_memalloc = -1;
	x170->fd_mem = -1;
	x170->inbuf.virtualAddress = NULL;
	x170->inbuf_size = GSTX170_INBUF_SIZE;
	x170->inbuf_thresh = GSTX170_INBUF_THRESH;
	x170->outbuf.virtualAddress = NULL;
	x170->outbuf_size = GSTX170_OUTBUF_SIZE;
	x170->picNumber = 0;
	x170->displayNumber = 0;
	x170->pp = NULL;
	x170->jpegdec = NULL;
	x170->h264dec = NULL;
	x170->mpeg2dec = NULL;
	x170->mpeg4dec = NULL;
	x170->vc1dec = NULL;
	memset(&x170->vc1meta, 0, sizeof(x170->vc1meta));
	x170->vc1meta_inited = 0;
}

static gboolean x170_init(GstPlugin *x170)
{
	GST_DEBUG_CATEGORY_INIT(gstx170_debug, "x170", 0,
				"Hantro x170 video decoder");
	return gst_element_register(x170, "x170", GST_RANK_SECONDARY,
				    GST_TYPE_X170);
}

GST_PLUGIN_DEFINE(GST_VERSION_MAJOR,
		  GST_VERSION_MINOR,
		  "x170",
		  "Hantro x170 video decoder",
		  x170_init,
		  VERSION,
		  GST_LICENSE_UNKNOWN,
		  "x170",
		  "http://www.leadtechdesign.com/")


