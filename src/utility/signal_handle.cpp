/**
* @Function: Handle error signals
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/utility/signal_handle.h"

#include <glog/logging.h>

namespace gici {

// Handle broken pipe exception from TCP/IP
static void handlePipe(int sig)
{
  LOG(INFO) << "Received a pipe exception!";
}

// Handle invalid access to storage
static void handleSegv(int sig) 
{
  LOG(FATAL) << "Received a segment fault exception!";
  exit(EXIT_FAILURE);
}

// Initialize all signal handles
extern void initializeSignalHandles()
{
  struct sigaction sa_pipe;
  struct sigaction sa_segv;

  sa_pipe.sa_handler = handlePipe;
  sigemptyset(&sa_pipe.sa_mask);
  sa_pipe.sa_flags = 0;
  sigaction(SIGPIPE, &sa_pipe, NULL);

  sa_segv.sa_handler = handleSegv;
  sigemptyset(&sa_segv.sa_mask);
  sa_segv.sa_flags = 0;
  sigaction(SIGSEGV, &sa_segv, NULL);
}

}