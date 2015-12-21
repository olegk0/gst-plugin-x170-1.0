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

#ifndef __GST_X170_H__
#define __GST_X170_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>
#include <getopt.h>
#include <errno.h>
#include <glib.h>
#include <gst/gst.h>
#include <gst/base/gstadapter.h>

#include <basetype.h>
#include <decapicommon.h>
#include <dwl.h>
#include <h264decapi.h>
#include <jpegdecapi.h>
#include <mp4decapi.h>
#include <mpeg2decapi.h>
#include <ppapi.h>
#include <vc1decapi.h>

#include <memalloc.h>

GST_DEBUG_CATEGORY_STATIC(gstx170_debug);
#define GST_CAT_DEFAULT gstx170_debug

#define X170_INIT_STATE		0
#define X170_PROCESS_STATE	1

#define X170_FRAME_BUFFER_SIZE	32
#define USE_THREAD

#define DEFAULT_WIDTH  480 
#define DEFAULT_HEIGHT 272

/* Codec Type Enumeration */
typedef enum {
	HT_AUTO,
	HT_IMAGE_JPEG,
	HT_VIDEO_H264,
	HT_VIDEO_MPEG2,
	HT_VIDEO_MPEG4,
	HT_VIDEO_VC1,
} CODECS;

/* Rotation Enumeration */
typedef enum {
	HT_ROTATION_0,
	HT_ROTATION_90,
	HT_ROTATION_180,
	HT_ROTATION_270,
} ROTATIONS;

/* Output Enumeration */
typedef enum {
	HT_OUTPUT_UYVY,
	HT_OUTPUT_RGB15,
	HT_OUTPUT_RGB16,
	HT_OUTPUT_RGB32,
} OUTPUTS;

/* Properties Type Enumeration */
typedef enum {
	PROP_0,		/* prop_id should be greater than 0 */
	ARG_CODEC,
	ARG_ROTATION,
	ARG_SCALING,
	ARG_OUTPUT_WIDTH,
	ARG_OUTPUT_HEIGHT,
	ARG_CROP_X,
	ARG_CROP_Y,
	ARG_CROP_WIDTH,
	ARG_CROP_HEIGHT,
	ARG_OUTPUT,
	ARG_INBUF_SIZE,
	ARG_INBUF_THRESH,
	ARG_OUTBUF_SIZE,
} PROPERTIES;

/* Begin Declaration */
G_BEGIN_DECLS

#define GST_TYPE_X170			(gst_x170_get_type())
#define GST_TYPE_X170_CODEC		(gst_x170_get_codec_type())
#define GST_TYPE_X170_ROTATION		(gst_x170_get_rotation_type())
#define GST_TYPE_X170_OUTPUT		(gst_x170_get_output_type())
#define GST_X170(obj)			(G_TYPE_CHECK_INSTANCE_CAST((obj),  \
					 GST_TYPE_X170, Gstx170))
#define GST_X170_CLASS(klass)		(G_TYPE_CHECK_CLASS_CAST((klass),   \
					 GST_TYPE_X170, Gstx170Class))
#define GST_X170_GET_CLASS(klass)	(G_TYPE_INSTANCE_GET_CLASS((klass), \
					 GST_TYPE_X170, Gstx170Class))
#define GST_IS_X170(obj)		(G_TYPE_CHECK_INSTANCE_TYPE((obj),  \
					 GST_TYPE_X170))
#define GST_IS_X170_CLASS(obj)		(G_TYPE_CHECK_CLASS_TYPE((klass),   \
					 GST_TYPE_X170))

struct _Gstx170 {
	GstElement parent;

	GstPad *sinkpad;
	GstPad *srcpad;
	GstCaps *caps;

	unsigned char *fb_data;
	int fb_len;

	guint nextid;
	guint frids[X170_FRAME_BUFFER_SIZE];
	GstBuffer *frames[X170_FRAME_BUFFER_SIZE];

	guint32 fourcc;
	guint width;
	guint height;
	guint fps_n;
	guint fps_d;

	guint codec;
	guint subtype;
	guint rotation;
	guint output_width;
	guint output_height;
	gfloat scaling;
	guint crop_x;
	guint crop_y;
	guint crop_width;
	guint crop_height;
	guint output;

	/* common */
	DWLLinearMem_t inbuf;
	guint32 inbuf_size;
	guint32 inbuf_thresh;
	DWLLinearMem_t outbuf;
	guint32 outbuf_size;
	guint32 picNumber;
	guint32 displayNumber;
	PPInst pp;

	/* JPEG */
	JpegDecInst jpegdec;

	/* H264 */
	H264DecInst h264dec;

	/* MPEG2 */
	Mpeg2DecInst mpeg2dec;

	/* MPEG4 */
	MP4DecInst mpeg4dec;

	/* VC1 */
	VC1DecInst vc1dec;
	VC1DecMetaData vc1meta;
	guint vc1meta_inited;
	guint vc1_advanced;
	guint vc1_v2;

	DWLLinearMem_t streamMem;
	void *DWLinstance;
#ifdef USE_THREAD
	pthread_t img_push_thread;
	sem_t full_sem;
	sem_t empty_sem;
	unsigned long frm_count, read_pos, write_pos;
	int force_exit;
#endif
};
typedef struct _Gstx170 Gstx170;

struct _Gstx170Class {
	GstElementClass parent_class;
};
typedef struct _Gstx170Class Gstx170Class;

GType gst_x170_get_type(void);

G_END_DECLS

#endif /* __GST_X170_H__ */
