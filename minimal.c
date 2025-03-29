#include <stdint.h>
#include "CH592SFR.h"

#define SLEEPTIME_MS 300
#ifndef DEEP_SLEEP // 'make deepsleep' will set this to 1
#define DEEP_SLEEP   0 // go into deep sleep instead of just waiting for SysTick (this will make the debug interface inoperable)
#endif

#define __HIGH_CODE __attribute__((section(".highcode")))

#define SYS_SAFE_ACCESS(a)  do { R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG1; \
								 R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG2; \
								 asm volatile ("nop\nnop"); \
								 {a} \
								 R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG0; \
								 asm volatile ("nop\nnop"); } while(0)
#define RTC_WAIT_TICKS(t)      uint32_t rtcset = R32_RTC_CNT_32K +(t); while(R32_RTC_CNT_32K <= rtcset)
#define RTC_MAX_COUNT          0xA8C00000
#define RTC_FREQ               32000 // LSI
// #define RTC_FREQ               32768 // LSE
#define CLK_PER_US             (1.0 / ((1.0 / RTC_FREQ) * 1000 * 1000))
#define CLK_PER_MS             (CLK_PER_US * 1000)
#define US_TO_RTC(us)          ((uint32_t)((us) * CLK_PER_US + 0.5))
#define MS_TO_RTC(ms)          ((uint32_t)((ms) * CLK_PER_MS + 0.5))
#define RTC_WAIT_TICKS(t)      uint32_t rtcset = R32_RTC_CNT_32K +(t); while(R32_RTC_CNT_32K <= rtcset)
#define SLEEP_RTC_MIN_TIME     US_TO_RTC(1000)
#define SLEEP_RTC_MAX_TIME     (RTC_MAX_COUNT - 1000 * 1000 * 30)
#define WAKE_UP_RTC_MAX_TIME   US_TO_RTC(1600)

// For debug writing to the debug interface.
#define DMDATA0 			   (*((PUINT32V)0xe0000380))

#define GPIO_Pin_8             (0x00000100)
#define GPIOA_ResetBits(pin)   (R32_PA_CLR |= (pin))
#define GPIOA_SetBits(pin)     (R32_PA_OUT |= (pin))
#define GPIOA_ModeCfg_Out(pin) R32_PA_PD_DRV &= ~(pin); R32_PA_DIR |= (pin)

#define EEPROM_PAGE_SIZE    256                       // Flash-ROM & Data-Flash page size for writing
#define EEPROM_BLOCK_SIZE   4096                      // Flash-ROM & Data-Flash block size for erasing
#define EEPROM_MIN_ER_SIZE  EEPROM_PAGE_SIZE          // Data-Flash minimal size for erasing
//#define EEPROM_MIN_ER_SIZE  EEPROM_BLOCK_SIZE         // Flash-ROM  minimal size for erasing
#define EEPROM_MIN_WR_SIZE  1                         // Data-Flash minimal size for writing
#define EEPROM_MAX_SIZE     0x8000                    // Data-Flash maximum size, 32KB
#define FLASH_MIN_WR_SIZE   4                         // Flash-ROM minimal size for writing
#define FLASH_ROM_MAX_SIZE  0x070000                  // Flash-ROM maximum program size, 448KB

#define CMD_FLASH_ROM_START_IO	0x00		// start FlashROM I/O, without parameter
#define CMD_FLASH_ROM_SW_RESET	0x04		// software reset FlashROM, without parameter
#define CMD_GET_ROM_INFO		0x06		// get information from FlashROM, parameter @Address,Buffer
#define CMD_GET_UNIQUE_ID		0x07		// get 64 bit unique ID, parameter @Buffer
#define CMD_FLASH_ROM_PWR_DOWN	0x0D		// power-down FlashROM, without parameter
#define CMD_FLASH_ROM_PWR_UP	0x0C		// power-up FlashROM, without parameter
#define CMD_FLASH_ROM_LOCK		0x08		// lock(protect)/unlock FlashROM data block, return 0 if success, parameter @StartAddr
// StartAddr: 0=unlock all, 1=lock boot code, 3=lock all code and data

#define CMD_EEPROM_ERASE		0x09		// erase Data-Flash block, return 0 if success, parameter @StartAddr,Length
#define CMD_EEPROM_WRITE		0x0A		// write Data-Flash data block, return 0 if success, parameter @StartAddr,Buffer,Length
#define CMD_EEPROM_READ			0x0B		// read Data-Flash data block, parameter @StartAddr,Buffer,Length
#define CMD_FLASH_ROM_ERASE		0x01		// erase FlashROM block, return 0 if success, parameter @StartAddr,Length
#define CMD_FLASH_ROM_WRITE		0x02		// write FlashROM data block, minimal block is dword, return 0 if success, parameter @StartAddr,Buffer,Length
#define CMD_FLASH_ROM_VERIFY	0x03		// read FlashROM data block, minimal block is dword, return 0 if success, parameter @StartAddr,Buffer,Length

#define ROM_CFG_MAC_ADDR	0x7F018			// address for MAC address information
#define ROM_CFG_BOOT_INFO	0x7DFF8			// address for BOOT information

#define flash_rom_start_io( )                       flash_eeprom_cmd( CMD_FLASH_ROM_START_IO, 0, 0, 0 )
#define flash_rom_sw_reset( )                       flash_eeprom_cmd( CMD_FLASH_ROM_SW_RESET, 0, 0, 0 )
#define get_mac_address(Buffer)                     flash_eeprom_cmd( CMD_GET_ROM_INFO, ROM_CFG_MAC_ADDR, Buffer, 0 )
#define get_boot_info(Buffer)                       flash_eeprom_cmd( CMD_GET_ROM_INFO, ROM_CFG_BOOT_INFO, Buffer, 0 )
#define get_unique_id(Buffer)                       flash_eeprom_cmd( CMD_GET_UNIQUE_ID, 0, Buffer, 0 )
#define flash_rom_pwr_down( )                       flash_eeprom_cmd( CMD_FLASH_ROM_PWR_DOWN, 0, 0, 0 )
#define flash_rom_pwr_up( )                         flash_eeprom_cmd( CMD_FLASH_ROM_PWR_UP, 0, 0, 0 )
#define eeprom_read(StartAddr,Buffer,Length)        flash_eeprom_cmd( CMD_EEPROM_READ, StartAddr, Buffer, Length )
#define eeprom_erase(StartAddr,Length)              flash_eeprom_cmd( CMD_EEPROM_ERASE, StartAddr, 0, Length )
#define eeprom_write(StartAddr,Buffer,Length)       flash_eeprom_cmd( CMD_EEPROM_WRITE, StartAddr, Buffer, Length )
#define flash_rom_erase(StartAddr,Length)           flash_eeprom_cmd( CMD_FLASH_ROM_ERASE, StartAddr, 0, Length )
#define flash_rom_write(StartAddr,Buffer,Length)    flash_eeprom_cmd( CMD_FLASH_ROM_WRITE, StartAddr, Buffer, Length )
#define flash_rom_verify(StartAddr,Buffer,Length)   flash_eeprom_cmd( CMD_FLASH_ROM_VERIFY, StartAddr, Buffer, Length )
#define flash_rom_read(StartAddr,Buffer,Length)     do { int k,l = (Length +3)>>2; for(k=0; k<l; k++) ((uint32_t*)(Buffer))[k] = ((uint32_t*)(StartAddr))[k]; } while(0)

#define __I  volatile const  /*!< defines 'read only' permissions     */
#define __O  volatile        /*!< defines 'write only' permissions     */
#define __IO volatile        /*!< defines 'read / write' permissions   */
/* memory mapped structure for Program Fast Interrupt Controller (PFIC) */
typedef struct
{
	__I uint32_t  ISR[8];           // 0
	__I uint32_t  IPR[8];           // 20H
	__IO uint32_t ITHRESDR;         // 40H
	uint8_t       RESERVED[4];      // 44H
	__O uint32_t  CFGR;             // 48H
	__I uint32_t  GISR;             // 4CH
	__IO uint8_t  VTFIDR[4];        // 50H
	uint8_t       RESERVED0[0x0C];  // 54H
	__IO uint32_t VTFADDR[4];       // 60H
	uint8_t       RESERVED1[0x90];  // 70H
	__O uint32_t  IENR[8];          // 100H
	uint8_t       RESERVED2[0x60];  // 120H
	__O uint32_t  IRER[8];          // 180H
	uint8_t       RESERVED3[0x60];  // 1A0H
	__O uint32_t  IPSR[8];          // 200H
	uint8_t       RESERVED4[0x60];  // 220H
	__O uint32_t  IPRR[8];          // 280H
	uint8_t       RESERVED5[0x60];  // 2A0H
	__IO uint32_t IACTR[8];         // 300H
	uint8_t       RESERVED6[0xE0];  // 320H
	__IO uint8_t  IPRIOR[256];      // 400H
	uint8_t       RESERVED7[0x810]; // 500H
	__IO uint32_t SCTLR;            // D10H
} PFIC_Type;

typedef struct
{
	__IO uint32_t CTLR;
	__IO uint32_t SR;
	__IO uint64_t CNT;
	__IO uint64_t CMP;
} SysTick_Type;

#define CORE_PERIPH_BASE           (0xE0000000) /* System peripherals base address in the alias region */
#define PFIC_BASE                  (CORE_PERIPH_BASE + 0xE000)
#define PFIC                       ((PFIC_Type *) PFIC_BASE)
#define NVIC                       PFIC

#define SysTick_BASE               (CORE_PERIPH_BASE + 0xF000)
#define SysTick                    ((SysTick_Type *) SysTick_BASE)
#define SysTick_LOAD_RELOAD_Msk    (0xFFFFFFFFFFFFFFFF)
#define SysTick_CTLR_SWIE          (1 << 31)
#define SysTick_CTLR_INIT          (1 << 5)
#define SysTick_CTLR_MODE          (1 << 4)
#define SysTick_CTLR_STRE          (1 << 3)
#define SysTick_CTLR_STCLK         (1 << 2)
#define SysTick_CTLR_STIE          (1 << 1)
#define SysTick_CTLR_STE           (1 << 0)
#define SysTick_SR_CNTIF           (1 << 0)

void NVIC_EnableIRQ(IRQn_Type IRQn)
{
	NVIC->IENR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
}

__attribute__((interrupt))
void RTC_IRQHandler(void) {
	R8_RTC_FLAG_CTRL = (RB_RTC_TMR_CLR | RB_RTC_TRIG_CLR);
}

void Clock60MHz() {
	SYS_SAFE_ACCESS(
		R8_PLL_CONFIG &= ~(1 << 5);
		R32_CLK_SYS_CFG = (1 << 6) | (0x48 & 0x1f) | RB_TX_32M_PWR_EN | RB_PLL_PWR_EN; // 60MHz = 0x48
	);
	asm volatile ("nop\nnop\nnop\nnop");
	R8_FLASH_CFG = 0X52;	
	SYS_SAFE_ACCESS(
		R8_PLL_CONFIG |= 1 << 7;
	);
}

void LSIEnable() {
	SYS_SAFE_ACCESS(
		R8_CK32K_CONFIG &= ~(RB_CLK_OSC32K_XT | RB_CLK_XT32K_PON); // turn off LSE
		R8_CK32K_CONFIG |= RB_CLK_INT32K_PON; // turn on LSI
	);
}

void DCDCEnable()
{
	SYS_SAFE_ACCESS(
		R16_AUX_POWER_ADJ |= RB_DCDC_CHARGE;
		R16_POWER_PLAN |= RB_PWR_DCDC_PRE;
	);

	RTC_WAIT_TICKS(2);

	SYS_SAFE_ACCESS(
		R16_POWER_PLAN |= RB_PWR_DCDC_EN;
	);
}

void SleepInit() {
	SYS_SAFE_ACCESS(
		R8_SLP_WAKE_CTRL |= RB_SLP_RTC_WAKE;
		R8_RTC_MODE_CTRL |= RB_RTC_TRIG_EN;
	);
	NVIC_EnableIRQ(RTC_IRQn);
}

void RTCInit() {
	SYS_SAFE_ACCESS(
		R32_RTC_TRIG = 0;
		R8_RTC_MODE_CTRL |= RB_RTC_LOAD_HI;
	);
	while((R32_RTC_TRIG & 0x3FFF) != (R32_RTC_CNT_DAY & 0x3FFF));
	SYS_SAFE_ACCESS(
		R32_RTC_TRIG = 0;
		R8_RTC_MODE_CTRL |= RB_RTC_LOAD_LO;
	);
}

void RTCTrigger(uint32_t cyc) {
	uint32_t t = R32_RTC_CNT_32K + cyc;
	if(t > RTC_MAX_COUNT) {
		t -= RTC_MAX_COUNT;
	}

	SYS_SAFE_ACCESS(
		R32_RTC_TRIG = t;
	);
}

void LowPowerIdle(uint32_t cyc)
{
	RTCTrigger(cyc);

	NVIC->SCTLR &= ~(1 << 2); // sleep
	NVIC->SCTLR &= ~(1 << 3); // wfi
	asm volatile ("wfi\nnop\nnop" );
}

void LowPowerSleep(uint32_t cyc, uint16_t power_plan) {
	RTCTrigger(cyc);

	SYS_SAFE_ACCESS(
		R8_BAT_DET_CTRL = 0;
		R8_XT32K_TUNE = (R16_RTC_CNT_32K > 0x3fff) ? (R8_XT32K_TUNE & 0xfc) | 0x01 : R8_XT32K_TUNE;
		R8_XT32M_TUNE = (R8_XT32M_TUNE & 0xfc) | 0x03;
	);

	NVIC->SCTLR |= (1 << 2); //deep sleep

	SYS_SAFE_ACCESS(
		R8_SLP_POWER_CTRL |= RB_RAM_RET_LV;
		R16_POWER_PLAN = RB_PWR_PLAN_EN | RB_PWR_CORE | power_plan;
		R8_PLL_CONFIG |= (1 << 5);
	);

	NVIC->SCTLR &= ~(1 << 3); // wfi
	asm volatile ("wfi\nnop\nnop" );

	SYS_SAFE_ACCESS(
		R16_POWER_PLAN &= ~RB_XT_PRE_EN;
		R8_PLL_CONFIG &= ~(1 << 5);
		R8_XT32M_TUNE = (R8_XT32M_TUNE & 0xfc) | 0x01;
	);
}

void LowPower(uint32_t time) {
	uint32_t time_sleep, time_curr;
	
	if (time <= WAKE_UP_RTC_MAX_TIME) {
		time = time + (RTC_MAX_COUNT - WAKE_UP_RTC_MAX_TIME);
	}
	else {
		time = time - WAKE_UP_RTC_MAX_TIME;
	}

	time_curr = R32_RTC_CNT_32K;
	if (time < time_curr) {
		time_sleep = time + (RTC_MAX_COUNT - time_curr);
	}
	else {
		time_sleep = time - time_curr;
	}
	
	if ((SLEEP_RTC_MIN_TIME < time_sleep) && (time_sleep < SLEEP_RTC_MAX_TIME)) {
		LowPowerSleep(time, (RB_PWR_RAM2K | RB_PWR_RAM24K | RB_PWR_EXTEND | RB_XT_PRE_EN) );
	}
	else {
		LowPowerIdle(time);
	}
	
	RTCInit();
}

void systick_delay_ms(int ms) {
	uint64_t targend = SysTick->CNT + (ms * 60 * 1000); // 60MHz clock
	while( ((int64_t)( SysTick->CNT - targend )) < 0 );
}

void DelayMs(int ms, int deepsleep) {
#if DEEP_SLEEP
	if(deepsleep) {
		LowPower(MS_TO_RTC(ms));
		DCDCEnable(); // Sleep disables DCDC
	}
	else {
		LowPowerIdle(MS_TO_RTC(ms));
	}
#else
	systick_delay_ms(ms);
#endif
}

__HIGH_CODE
void flash_rom_beg(uint8_t beg) {
	R8_FLASH_CTRL = 0;
	R8_FLASH_CTRL = 0x5;
	asm volatile ("nop\nnop");
	R8_FLASH_DATA = beg;
	if(beg == 0xff) {
		while((char)R8_FLASH_CTRL < 0);
		R8_FLASH_DATA = beg;
		while((char)R8_FLASH_CTRL < 0);
	}
}

__HIGH_CODE
void flash_rom_end() {
	while((char)R8_FLASH_CTRL < 0);
	R8_FLASH_CTRL = 0;
}

__HIGH_CODE
void flash_start() {
	SYS_SAFE_ACCESS(
		R32_GLOBAL_CONFIG |= 0xe0; // 0xe0 for writing, otherwise 0x20
	);

	R8_FLASH_CTRL = 0x4;
	flash_rom_beg(0xff);
	flash_rom_end();
}

__HIGH_CODE
void flash_rom_out(uint8_t val) {
	while((char)R8_FLASH_CTRL < 0);
	R8_FLASH_DATA = val;
}

__HIGH_CODE
uint8_t flash_rom_in() {
	while((char)R8_FLASH_CTRL < 0);
	return R8_FLASH_DATA;
}

__HIGH_CODE
void flash_rom_addr(uint8_t beg, uint32_t addr) {
	uint8_t repeat = 5;
	if((beg & 0xbf) != 0xb) {
		flash_rom_beg(0x6);
		flash_rom_end();
		repeat = 3;
	}
	flash_rom_beg(beg);
	for(int i = 0; i < repeat; i++) {
		flash_rom_out((uint8_t)(addr >> 0x10));
		addr <<= 8;
	}
}

__HIGH_CODE
uint8_t flash_rom_wait() {
	uint8_t b;
	flash_rom_end();
	for(int i = 0; i < 0x80000; i++) {
		flash_rom_beg(0x5);
		flash_rom_in();
		b = flash_rom_in();
		flash_rom_end();
		if((b & 1) == 0) {
			return b | 1;
		}
	}
	return 0;
}

__HIGH_CODE
void flash_eeprom_cmd( uint8_t cmd, uint32_t StartAddr, void *Buffer, uint32_t Length ) {
	uint8_t b = 0;
	uint32_t isr0 = NVIC->ISR[0];
	uint32_t isr1 = NVIC->ISR[1];
	NVIC->IRER[0] = 0xffffffff;
	NVIC->IRER[1] = 0xffffffff;

	switch(cmd) {
	case CMD_FLASH_ROM_SW_RESET:
		flash_start();
		flash_rom_beg(0x66);
		flash_rom_end();
		flash_rom_beg(0x99);
		flash_rom_end();
		break;
	case CMD_FLASH_ROM_WRITE:
		flash_start();
		do {
			Length >>= 2; // divide by 4, to write words instead of bytes
			flash_rom_addr(0x2, StartAddr);
			for(int i = 0; i < Length; i++) {
				R32_FLASH_DATA = ((uint32_t*)Buffer)[i];
				for(int j = 0; j < 4; j++) {
					while((char)R8_FLASH_CTRL < 0);
					R8_FLASH_CTRL = 0x15;
				}
			}
			b = flash_rom_wait();
		} while(!b);
		break;
	default:
		break;
	}

	SYS_SAFE_ACCESS(
		R32_GLOBAL_CONFIG &= 0x10;
	);

	NVIC->IENR[0] = isr0;
	NVIC->IENR[1] = isr1;
}

void blink(int n) {
	for(int i = n-1; i >= 0; i--) {
		GPIOA_ResetBits(GPIO_Pin_8);
		DelayMs(33, /*deepsleep*/FALSE);
		GPIOA_SetBits(GPIO_Pin_8);
		if(i) DelayMs(33, /*deepsleep*/FALSE);
	}
}

void char_debug(char c) {
	// this while is wasting clock ticks, but the easiest way to demo the debug interface
	while(DMDATA0 & 0xc0);
	DMDATA0 = 0x85 | (c << 8);
}

void print(char msg[], int size, int endl) {
	for(int i = 0; i < size; i++) {
		char_debug(msg[i]);
	}
	if(endl) {
		char_debug('\r');
		char_debug('\n');
	}
}

void print_bytes(uint8_t data[], int size) {
	char hex_digits[] = "0123456789abcdef";
	char hx[] = "0x00 ";
	for(int i = 0; i < size; i++) {
		hx[2] = hex_digits[(data[i] >> 4) & 0x0F];
		hx[3] = hex_digits[data[i] & 0x0F];
		print(hx, 5, /*endl*/FALSE);
	}
	print(0, 0, /*endl*/TRUE);
}


int main(void) {
	Clock60MHz();
	DCDCEnable();
	GPIOA_ModeCfg_Out(GPIO_Pin_8);
	GPIOA_SetBits(GPIO_Pin_8);
	LSIEnable();
	SysTick->CTLR = 5; // enable SysTick on HCLK
#if DEEP_SLEEP
	RTCInit();
	SleepInit();
#endif
	uint8_t TestBuf[1024];

	blink(5);

#if !DEEP_SLEEP
	flash_rom_read(0x69c, TestBuf, 4);
	char temp0 = TestBuf[0];
	TestBuf[0] = TestBuf[1];
	TestBuf[1] = TestBuf[2];
	TestBuf[2] = TestBuf[3];
	TestBuf[3] = temp0;
	flash_rom_write(0x69c, TestBuf, 4);
	print("No More EVT!", sizeof("No More EVT!"), /*endl=*/TRUE);
#endif

	while(1) {
		DelayMs(SLEEPTIME_MS -33, /*deepsleep=*/TRUE);
		blink(1); // 33 ms
	}
}
