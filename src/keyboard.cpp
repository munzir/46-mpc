/*
 * Copyright (c) 2018, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @file keyboard.cpp
 * @author Munzir Zafar
 * @date Oct 30, 2018
 * @brief code for thread reading keybaord input
 */

#include "keyboard.h"

#include <iostream>
#include <pthread.h>

/* ********************************************************************************************* */
/// Sits waiting for keyboard character input. Then raises a global flag to let other
// threads know
void *kbhit(void *arg) {

  struct kbShared *kb_shared = (struct kbShared *)arg;

  char input;
  bool other_thread_has_read_char_received = false;
	while(true){
		input = std::cin.get();

    pthread_mutex_lock(&kb_shared->kb_mutex);
      kb_shared->kb_char_input = input;
      kb_shared->kb_char_received = true; // this is a global that lets other threads know that
                               // a chracter is received by this thread
      other_thread_has_read_char_received = false;
    pthread_mutex_unlock(&kb_shared->kb_mutex);

    // Wait for kb_char_received to go false, indicating that the character was
    // read by the other thread
    while(!other_thread_has_read_char_received) {
      pthread_mutex_lock(&kb_shared->kb_mutex);
        if(kb_shared->kb_char_received == false)
          other_thread_has_read_char_received = true;
      pthread_mutex_unlock(&kb_shared->kb_mutex);
    }
  }
}

/* ********************************************************************************************* */
// The function to be called by other threads to read the character input, if received
bool kbCharReceived(kbShared& kb_shared, char* input) {

  bool char_received = false;

  pthread_mutex_lock(&kb_shared.kb_mutex);
  if(kb_shared.kb_char_received) {
    char_received = true;
    *input = kb_shared.kb_char_input;

    kb_shared.kb_char_received = false; // to let kbhit thread know that we have read the input
  }
  pthread_mutex_unlock(&kb_shared.kb_mutex);

  return char_received;
}
