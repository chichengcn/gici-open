/**
* @Function: Utilities 
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include "gici/stream/formator.h"
#include "gici/gnss/gnss_common.h"

namespace gici {

#define USE_4BIT_TAG 1
#define TIMETAGH_LEN 64          /* time tag file header length */
#define FILE_MODE_MASTER 1
#define FILE_MODE_CLIENT 2

typedef struct {            /* file control type */
    FILE *fp;               /* file pointer */
    FILE *fp_tag;           /* file pointer of tag file */
    FILE *fp_tmp;           /* temporary file pointer for swap */
    FILE *fp_tag_tmp;       /* temporary file pointer of tag file for swap */
    char path[MAXSTRPATH];  /* file path */
    char openpath[MAXSTRPATH]; /* open file path */
    int mode;               /* file mode */
    int timetag;            /* time tag flag (0:off,1:on) */
    int repmode;            /* replay mode (0:master,1:slave) */
    int offset;             /* time offset (ms) for slave */
    int size_fpos;          /* file position size (bytes) */
    gtime_t time;           /* start time */
    gtime_t wtime;          /* write time */
    uint32_t tick;          /* start tick */
    uint32_t tick_f;        /* start tick in file */
    long fpos_n;            /* next file position */
    uint32_t tick_n;        /* next tick */
    double start;           /* start offset (s) */
    double speed;           /* replay speed (time factor) */
    double swapintv;        /* swap interval (hr) (0: no swap) */
    lock_t lock;            /* lock flag */
} file_t;

file_t *openfile(const char *path, int mode);
void closefile(file_t *file);
int getProperBufLength(const YAML::Node& node);
double getTimestamp(const std::shared_ptr<DataCluster>& data, const YAML::Node& node);

}
