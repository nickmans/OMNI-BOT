/*
 * shared_mem.h
 *
 *  Created on: Jul 18, 2025
 *      Author: nicks
 */

#ifndef SHARED_MEM_H
#define SHARED_MEM_H
#include <stdint.h>

typedef struct {
    float   control_u[4];       // 4 × 4 B = 16 B
    double  tag_pos[3];  // 3 × 8 B = 24 B
    int 	anchordis[3];
    int     flagm7;             // 4 B
    int		flagm4;				// 4 B
} SharedMem_t;

// The mailbox lives at 0x3002_0000
#define SHARED_MEM_BASE  ((uintptr_t)0x30020000U)
#define SHARED_MEM       ((volatile SharedMem_t*)SHARED_MEM_BASE)
