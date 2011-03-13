/* arch/arm/mach-msm/include/mach/memory.h
 *
 * Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

/* physical offset of RAM */
#if defined(CONFIG_ARCH_QSD8X50) && defined(CONFIG_MSM_SOC_REV_A)
#define PHYS_OFFSET		UL(0x00000000)
#elif defined(CONFIG_ARCH_QSD8X50)
#define PHYS_OFFSET		UL(0x20000000)
#define RESET_VECTOR		UL(0x00000000)
#elif defined(CONFIG_MSM_AMSS_SUPPORT_256MB_EBI1)
#define PHYS_OFFSET             UL(0x19200000)
#define RESET_VECTOR            UL(0x00000000)
#elif defined(CONFIG_ARCH_MSM7X30)
#define PHYS_OFFSET		UL(0x00200000)
#define RESET_VECTOR		UL(0x00000000)
#else
#if defined(CONFIG_MSM_AMSS_RADIO2708_MEMMAP)
#define PHYS_OFFSET		UL(0x02000000)
#define RESET_VECTOR		UL(0x00000000)
#else
#define PHYS_OFFSET		UL(0x10000000)
#define RESET_VECTOR		UL(0x00000000)
#endif
#endif

#define HAS_ARCH_IO_REMAP_PFN_RANGE

#define CONSISTENT_DMA_SIZE (4*SZ_1M)

#ifndef __ASSEMBLY__
void *alloc_bootmem_aligned(unsigned long size, unsigned long alignment);
void clean_and_invalidate_caches(unsigned long, unsigned long, unsigned long);
void clean_caches(unsigned long, unsigned long, unsigned long);
void invalidate_caches(unsigned long, unsigned long, unsigned long);

#ifdef CONFIG_ARCH_MSM_ARM11
void write_to_strongly_ordered_memory(void);

#include <asm/mach-types.h>

#if defined(CONFIG_ARCH_MSM7227)
#define arch_barrier_extra() do \
       { \
               write_to_strongly_ordered_memory(); \
       } while (0)
#else
#define arch_barrier_extra() do {} while (0)
#endif

#ifdef CONFIG_CACHE_L2X0
extern void l2x0_cache_sync(void);
extern void l2x0_cache_flush_all(void);
#define finish_arch_switch(prev)     do { l2x0_cache_sync(); } while (0)
#endif

#endif
#endif

#ifdef CONFIG_ARCH_MSM_SCORPION
#define arch_has_speculative_dfetch()  1
#endif

#endif

