/****************************************************************************
 * dnnrt_face_tracking/dnnrt_face_tracking_main.c
 *
 *   Copyright 2018 Sony Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Corporation nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>

#include <sys/time.h>
#include <sys/ioctl.h>

#include <nuttx/board.h>

#include <video/video.h>
#include <imageproc/imageproc.h>
#include <dnnrt/runtime.h>

#include "nximage.h"
#include "loader_nnb.h"


/****************************************************************************
 * Type Definition
 ****************************************************************************/
struct v_buffer {
  uint32_t             *start;
  uint32_t             length;
};
typedef struct v_buffer v_buffer_t;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define DNN_NNB_PATH    "/mnt/sd0/face-tracking.nnb"
#define IMAGE_WIDTH_PX  (32)
#define IMAGE_HEIGHT_PX (32)

#define IMAGE_YUV_SIZE  (320*240*2) /* QVGA YUV422 */
#define VIDEO_BUFNUM    (3)

#ifndef CONFIG_EXAMPLES_CAMERA_LCD_DEVNO
#  define CONFIG_EXAMPLES_CAMERA_LCD_DEVNO 0
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/
static float s_img_buffer[IMAGE_WIDTH_PX * IMAGE_HEIGHT_PX];

static struct v_buffer  *buffers_video;
static unsigned int     n_buffers;

struct nximage_data_s g_nximage =
{
  NULL,          /* hnx */
  NULL,          /* hbkgd */
  0,             /* xres */
  0,             /* yres */
  false,         /* havpos */
  { 0 },         /* sem */
  0              /* exit code */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline int nximage_initialize(void)
{
  FAR NX_DRIVERTYPE *dev;
  nxgl_mxpixel_t color;
  int ret;

  /* Initialize the LCD device */

  printf("nximage_initialize: Initializing LCD\n");
  ret = board_lcd_initialize();
  if (ret < 0)
    {
      printf("nximage_initialize: board_lcd_initialize failed: %d\n", -ret);
      return ERROR;
    }

  /* Get the device instance */

  dev = board_lcd_getdev(CONFIG_EXAMPLES_CAMERA_LCD_DEVNO);
  if (!dev)
    {
      printf("nximage_initialize: board_lcd_getdev failed, devno=%d\n",
             CONFIG_EXAMPLES_CAMERA_LCD_DEVNO);
      return ERROR;
    }

  /* Turn the LCD on at 75% power */

  (void)dev->setpower(dev, ((3*CONFIG_LCD_MAXPOWER + 3)/4));

  /* Then open NX */

  printf("nximage_initialize: Open NX\n");
  g_nximage.hnx = nx_open(dev);
  if (!g_nximage.hnx)
    {
      printf("nximage_initialize: nx_open failed: %d\n", errno);
      return ERROR;
    }

  /* Set background color to black */

  color = 0;
  nx_setbgcolor(g_nximage.hnx, &color);
  ret = nx_requestbkgd(g_nximage.hnx, &g_nximagecb, NULL);
  if (ret < 0)
    {
      printf("nximage_initialize: nx_requestbkgd failed: %d\n", errno);
      nx_close(g_nximage.hnx);
      return ERROR;
    }

  while (!g_nximage.havepos)
    {
      (void) sem_wait(&g_nximage.sem);
    }
  printf("nximage_initialize: Screen resolution (%d,%d)\n",
         g_nximage.xres, g_nximage.yres);

  return 0;
}

static int camera_prepare(int                fd,
                          enum v4l2_buf_type type,
                          uint32_t           buf_mode,
                          uint32_t           pixformat,
                          uint16_t           hsize,
                          uint16_t           vsize,
                          uint8_t            buffernum)
{
  int ret;
  int cnt;
  uint32_t fsize;
  struct v4l2_format         fmt = {0};
  struct v4l2_requestbuffers req = {0};
  struct v4l2_buffer         buf = {0};
  struct v_buffer  *buffers;

  /* VIDIOC_REQBUFS initiate user pointer I/O */
  req.type   = type;
  req.memory = V4L2_MEMORY_USERPTR;
  req.count  = buffernum;
  req.mode   = buf_mode;
  
  ret = ioctl(fd, VIDIOC_REQBUFS, (unsigned long)&req);
  if (ret < 0)
    {
      printf("Failed to VIDIOC_REQBUFS: errno = %d\n", errno);
      return ret;
    }

  /* VIDIOC_S_FMT set format */
  fmt.type                = type;
  fmt.fmt.pix.width       = hsize;
  fmt.fmt.pix.height      = vsize;
  fmt.fmt.pix.field       = V4L2_FIELD_ANY;
  fmt.fmt.pix.pixelformat = pixformat;

  ret = ioctl(fd, VIDIOC_S_FMT, (unsigned long)&fmt);
  if (ret < 0)
    {
      printf("Failed to VIDIOC_S_FMT: errno = %d\n", errno);
      return ret;
    }

  /* VIDIOC_QBUF enqueue buffer */
  if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
    {
      buffers_video = malloc(sizeof(v_buffer_t) * buffernum);
      buffers = buffers_video;
    }
  else
    {
      return ERROR;
    }

  if (!buffers)
    {
      printf("Out of memory\n");
      return ret;
    }

  if (pixformat == V4L2_PIX_FMT_UYVY)
    {
      fsize = IMAGE_YUV_SIZE;
    }
  else
    {
      return ERROR;
    }

  for (n_buffers = 0; n_buffers < buffernum; ++n_buffers)
    {
      buffers[n_buffers].length = fsize;

      /* Note: VIDIOC_QBUF set buffer pointer. */
      /*       Buffer pointer must be 32bytes aligned. */

      buffers[n_buffers].start  = memalign(32, fsize);
      if (!buffers[n_buffers].start)
        {
          printf("Out of memory\n");
          return ret;
        }
    }

  for (cnt = 0; cnt < n_buffers; cnt++)
    {
      memset(&buf, 0, sizeof(v4l2_buffer_t));
      buf.type = type;
      buf.memory = V4L2_MEMORY_USERPTR;
      buf.index = cnt;
      buf.m.userptr = (unsigned long)buffers[cnt].start;
      buf.length = buffers[cnt].length;

      ret = ioctl(fd, VIDIOC_QBUF, (unsigned long)&buf);
      if (ret)
        {
          printf("Fail QBUF %d\n", errno);
          return ret;;
        }
    }

  /* VIDIOC_STREAMON start stream */

  ret = ioctl(fd, VIDIOC_STREAMON, (unsigned long)&type);
  if (ret < 0)
    {
      printf("Failed to VIDIOC_STREAMON: errno = %d\n", errno);
      return ret;
    }

  return OK;
}

static void free_buffer(struct v_buffer  *buffers, uint8_t bufnum)
{
  uint8_t cnt;
  if (buffers)
    {
      for (cnt = 0; cnt < bufnum; cnt++)
        {
          if (buffers[cnt].start)
            {
              free(buffers[cnt].start);
            }
        }

      free(buffers);
    }
}

/****************************************************************************
 * dnnrt_face_tracking_main
 ****************************************************************************/
#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int dnnrt_face_tracking_main(int argc, char *argv[])
#endif
{
  int ret, v_fd, i, j;
  float *output_buffer, proc_time;
  const void *inputs[1] = { s_img_buffer };
  dnn_runtime_t rt;
  nn_network_t *network;
  struct timeval begin, end;
  v4l2_buffer_t buf;

  ret = nximage_initialize();
  if (ret < 0)
    {
      printf("camera_main: Failed to get NX handle: %d\n", errno);
      return ERROR;
    }

  imageproc_initialize();

  ret = video_initialize("/dev/video");
  if (ret != 0)
    {
      printf("ERROR: Failed to initialize video: errno = %d\n", errno);
      goto errout_with_nx;
    }

  v_fd = open("/dev/video", 0);
  if (v_fd < 0)
    {
      printf("ERROR: Failed to open video.errno = %d\n", errno);
      goto errout_with_video_init;
    }

  /* Prepare VIDEO_CAPTURE */
  ret = camera_prepare(v_fd,
                       V4L2_BUF_TYPE_VIDEO_CAPTURE,
                       V4L2_BUF_MODE_RING,
                       V4L2_PIX_FMT_UYVY,
                       VIDEO_HSIZE_QVGA,
                       VIDEO_VSIZE_QVGA,
                       VIDEO_BUFNUM);
  if (ret < 0)
    {
      goto errout_with_buffer;
    }

  /* load an nnb file, which holds a network structure and weight values,
     into a heap memory */
  network = alloc_nnb_network(DNN_NNB_PATH);
  if (network == NULL)
    {
      printf("load nnb file failed\n");
      goto pgm_error;
    }

  /* Step-A: initialize the whole dnnrt subsystem */
  ret = dnn_initialize(NULL);
  if (ret)
    {
      printf("dnn_initialize() failed due to %d", ret);
      goto dnn_error;
    }

  /* Step-B: instantiate a neural network defined
             by nn_network_t as a dnn_runtime_t object */
  ret = dnn_runtime_initialize(&rt, network);
  if (ret)
    {
      printf("dnn_runtime_initialize() failed due to %d\n", ret);
      goto rt_error;
    }

  for (i = 0; i < IMAGE_HEIGHT_PX; i++)
    {
      for (j = 0; j < IMAGE_WIDTH_PX; j++)
        {
          s_img_buffer[i * IMAGE_WIDTH_PX + j] = 0.5;
        }
    }

  /* Step-C: perform inference after feeding inputs */
  printf("start dnn_runtime_forward()\n");
  gettimeofday(&begin, 0);
  ret = dnn_runtime_forward(&rt, inputs, 1);
  gettimeofday(&end, 0);
  if (ret)
    {
      printf("dnn_runtime_forward() failed due to %d\n", ret);
      goto fin;
    }

  /* Step-D: obtain the output from this dnn_runtime_t */
  output_buffer = dnn_runtime_output_buffer(&rt, 0u);

  /* show the classification result and its processing time */
  for (i = 0; i < 2; i++)
    {
      printf("output[%u]=%.6f\n", i, output_buffer[i]);
    }
  proc_time = (float)end.tv_sec + (float)end.tv_usec / 1.0e6;
  proc_time -= (float)begin.tv_sec + (float)begin.tv_usec / 1.0e6;
  printf("inference time=%.3f\n", proc_time);

  for (;;)
    {
      memset(&buf, 0, sizeof(v4l2_buffer_t));
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_USERPTR;

      ret = ioctl(v_fd, VIDIOC_DQBUF, (unsigned long)&buf);
      if (ret)
        {
          printf("Fail DQBUF %d\n", errno);
          goto errout_with_buffer;
        }

      /* Convert YUV color format to RGB565 */
      imageproc_convert_yuv2rgb((void *)buf.m.userptr,
                       VIDEO_HSIZE_QVGA,
                       VIDEO_VSIZE_QVGA);
      
      nximage_image(g_nximage.hbkgd, (void *)buf.m.userptr);

      ret = ioctl(v_fd, VIDIOC_QBUF, (unsigned long)&buf);
      if (ret)
        {
          printf("Fail QBUF %d\n", errno);
          goto errout_with_buffer;
        }
    }

fin:
  /* Step-F: free memories allocated to dnn_runtime_t */
  dnn_runtime_finalize(&rt);
rt_error:
  /* Step-G: finalize the whole dnnrt subsystem */
  dnn_finalize();
dnn_error:
  /* just call free() */
  destroy_nnb_network(network);
pgm_error:
errout_with_buffer:
  close(v_fd);

  free_buffer(buffers_video, VIDEO_BUFNUM);

errout_with_video_init:

  video_uninitialize();

errout_with_nx:
  imageproc_finalize();

  nx_close(g_nximage.hnx);
  return ret;
}
