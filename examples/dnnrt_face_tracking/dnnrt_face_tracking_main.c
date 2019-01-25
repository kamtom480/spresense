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
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <sys/time.h>
#include <nuttx/config.h>
#include <dnnrt/runtime.h>
#include "loader_nnb.h"

/****************************************************************************
 * Type Definition
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define DNN_NNB_PATH    "/mnt/sd0/face-tracking.nnb"
#define IMAGE_WIDTH_PX  (32)
#define IMAGE_HEIGHT_PX (32)

/****************************************************************************
 * Private Data
 ****************************************************************************/
static float s_img_buffer[IMAGE_WIDTH_PX * IMAGE_HEIGHT_PX];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * dnnrt_face_tracking_main
 ****************************************************************************/
#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int dnnrt_face_tracking_main(int argc, char *argv[])
#endif
{
  int ret, i, j;
  float *output_buffer, proc_time;
  const void *inputs[1] = { s_img_buffer };
  dnn_runtime_t rt;
  nn_network_t *network;
  struct timeval begin, end;

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
  return ret;
}
