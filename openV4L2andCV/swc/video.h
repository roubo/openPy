#ifndef VIDEO_H

#define VIDEO_H

#include <linux/videodev2.h>

struct buffer {

  struct v4l2_requestbuffers req; /* ���� */

  struct v4l2_buffer query;      /* ��ȡ */

  struct {              /* ���� */

    void *start;

    size_t length;

  } *buf;

};

struct video {

  int fd;

  struct v4l2_format format;    /* ��Ƶ֡��ʽ */

  struct buffer buffer;        /* ��Ƶ���� */

};

void video_init();

void video_quit();

void buffer_enqueue(int index);

void buffer_dequeue(int index);

#endif