#ifndef CUSTOM_DWT_H_
#define CUSTOM_DWT_H_


uint32_t dwt_StartCnt;              // DWT start cycle counter value
uint32_t dwt_StopCnt;               // DWT stop cycle counter value
uint32_t DWT_cnt_val;				// dwt_StopCnt - dwt_StartCnt

#define DEMCR_TRCENA    0x01000000

/* Core Debug registers */
#define DEMCR           (*((volatile uint32_t *)0xE000EDFC))
#define DWT_CTRL        (*(volatile uint32_t *)0xe0001000)
#define CYCCNTENA       (1<<0)
#define DWT_CYCCNT      ((volatile uint32_t *)0xE0001004)
#define CPU_CYCLES      *DWT_CYCCNT

#define dwt_start 		{*DWT_CYCCNT = 0; dwt_StartCnt = *((volatile unsigned int *)0xE0001004);}
#define dwt_stop  		{dwt_StopCnt = *((volatile unsigned int *)0xE0001004); DWT_cnt_val = dwt_StopCnt - dwt_StartCnt;}


#endif /* CUSTOM_DWT_H_ */
