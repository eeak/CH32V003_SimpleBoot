#include <stdbool.h>
#include <string.h>
#include <ch32v00x.h>
#include <ch32v00x_flash.h>

#define FLASH_PAGE_SIZE 64

#define UART_TIMEOUT    50 // 50 ms.
#define CHECK_TIMEOUT   1500 // 1.5 sec.

#define UART_SIGN_1     0x55
#define UART_SIGN_2     0xAA

#define IAP_CMD_HELO    0x80
#define IAP_CMD_ERASE   0x81
#define IAP_CMD_PROG    0x82
#define IAP_CMD_VERIFY  0x83
#define IAP_CMD_END     0x84

#define IAP_SUCCESS     0x00
#define IAP_ERROR       0x01
#define IAP_TIMEOUT     0x02
#define IAP_END         0x03

typedef struct __attribute__((packed)) {
    uint8_t cmd;
    uint8_t len;
    uint8_t payload[FLASH_PAGE_SIZE];
} iap_t;

static volatile uint32_t _ms = 0;

static void SYSTICK_Init(void) {
    NVIC_EnableIRQ(SysTicK_IRQn);
    SysTick->SR = 0;
    SysTick->CMP = FCPU / 1000 - 1;
    SysTick->CNT = 0;
    SysTick->CTLR = 0x0F;
}

static void SYSTICK_Done(void) {
    NVIC_DisableIRQ(SysTicK_IRQn);
    SysTick->CTLR = 0;
}

static inline __attribute__((always_inline)) uint32_t millis() {
    return _ms;
}

static void delay_ms(uint32_t ms) {
    ms += _ms;
    while ((int32_t)_ms < (int32_t)ms) {}
}

static void UART_Init(void) {
    RCC->APB2PCENR |= (RCC_USART1EN | RCC_IOPDEN);

    /* USART1 RX-->D6 */
    GPIOD->CFGLR &= ~((uint32_t)0x0F << (4 * 6));
    GPIOD->CFGLR |= ((uint32_t)0x04 << (4 * 6));

    USART1->BRR = 0xD0; // 13.0 (24000000 / 16 / 115200)
    USART1->CTLR2 = 0;
    USART1->CTLR3 = 0;
    USART1->CTLR1 = USART_CTLR1_UE | USART_CTLR1_RE;
}

static void UART_Reinit(void) {
    /* USART1 TX-->D5 */
    GPIOD->CFGLR &= ~((uint32_t)0x0F << (4 * 5));
    GPIOD->CFGLR |= ((uint32_t)0x0B << (4 * 5));

    USART1->CTLR1 |= USART_CTLR1_TE;
}

static void UART_Done(void) {
    GPIOD->CFGLR &= 0xF00FFFFF;
    GPIOD->CFGLR |= 0x04400000;

    USART1->CTLR1 = 0;

    RCC->APB2PCENR &= ~(RCC_USART1EN | RCC_IOPDEN);
}

static int16_t UART_ReadTimed(uint32_t timeout) {
    timeout += millis();
    while ((int32_t)millis() < (int32_t)timeout) {
        if (USART1->STATR & USART_STATR_RXNE)
            return USART1->DATAR;
    }
    return -1;
}

static void UART_Write(uint8_t data) {
    while (! (USART1->STATR & USART_STATR_TXE)) {}
    USART1->DATAR = data;
}

static void IAP_Program(uint32_t addr, const uint32_t* buf) {
    addr &= 0xFFFFFFC0;
    FLASH_BufReset();
    for (uint8_t i = 0; i < 16; ++i) {
        FLASH_BufLoad(addr + i * 4, buf[i]);
    }
    FLASH_ProgramPage_Fast(addr);
}

static bool IAP_Read(iap_t *frame, uint32_t timeout) {
    int16_t data;
    uint8_t crc = 0;

    if ((data = UART_ReadTimed(timeout)) == UART_SIGN_1) {
        if ((data = UART_ReadTimed(timeout)) == UART_SIGN_2) {
            data = UART_ReadTimed(timeout);
            if ((data >= IAP_CMD_HELO) && (data <= IAP_CMD_END)) {
                frame->cmd = data;
                crc += data;
                data = UART_ReadTimed(timeout);
                if ((data >= 0) && (data <= FLASH_PAGE_SIZE)) {
                    memset(frame->payload, 0xFF, FLASH_PAGE_SIZE);
                    frame->len = data;
                    crc += data;
                    for (uint8_t i = 0; i < frame->len; ++i) {
                        data = UART_ReadTimed(timeout);
                        if (data < 0)
                            return false;
                        frame->payload[i] = data;
                        crc += data;
                    }
                    data = UART_ReadTimed(timeout);
                    if ((data >= 0) && ((uint8_t)data == crc))
                        return true;
                }
            }
        }
    }
    return false; // Wrong data or timeout
}

static void IAP_Answer(uint8_t state) {
    UART_Write(UART_SIGN_2);
    UART_Write(UART_SIGN_1);
    UART_Write(state);
}

static bool IAP_Start(uint32_t timeout) {
    iap_t frame;

    timeout /= UART_TIMEOUT;
    while ((! IAP_Read(&frame, UART_TIMEOUT)) || (frame.cmd != IAP_CMD_HELO)) {
        if (! timeout--)
            return false;
    }
    return true;
}

static bool IAP_Process(void) {
    iap_t frame;
    uint32_t prog_addr = FLASH_BASE;
    uint8_t *verify_addr = (uint8_t*)FLASH_BASE;
    bool complete = false, erased = false;

    while (! IAP_Start(CHECK_TIMEOUT)) {
        if (*(uint32_t*)FLASH_BASE != 0xFFFFFFFF) // Flash not empty
            return false;
    }
    UART_Reinit();
    IAP_Answer(IAP_SUCCESS); // HELO
    while (! complete) {
        if (IAP_Read(&frame, UART_TIMEOUT)) {
            uint8_t state;

            switch (frame.cmd) {
                case IAP_CMD_HELO:
                    erased = false;
                    state = IAP_SUCCESS;
                    break;
                case IAP_CMD_ERASE:
                    FLASH_Unlock_Fast();
                    if (FLASH_EraseAllPages() == FLASH_COMPLETE) {
                        prog_addr = FLASH_BASE;
                        verify_addr = (uint8_t*)FLASH_BASE;
                        erased = true;
                        state = IAP_SUCCESS;
                    } else
                        state = IAP_ERROR;
                    break;
                case IAP_CMD_PROG:
                    if (erased) {
                        IAP_Program(prog_addr, (const uint32_t*)frame.payload);
                        prog_addr += FLASH_PAGE_SIZE;
                        state = IAP_SUCCESS;
                    } else
                        state = IAP_ERROR;
                    break;
                case IAP_CMD_VERIFY:
                    state = IAP_SUCCESS;
                    for (uint8_t i = 0; i < frame.len; ++i) {
                        if (*verify_addr++ != frame.payload[i]) {
                            state = IAP_ERROR;
                            break;
                        }
                    }
                    break;
                case IAP_CMD_END:
                    FLASH_Lock_Fast();
                    complete = true;
                    state = IAP_SUCCESS;
                    break;
                default:
                    state = IAP_ERROR;
                    break;
            }
            IAP_Answer(state);
        }
    }
    return true;
}

int main(void) {
    SYSTICK_Init();
    UART_Init();

    if (IAP_Process()) {
        delay_ms(50);
    }

    UART_Done();
    SYSTICK_Done();

    RCC->RSTSCKR |= RCC_RMVF;
    SystemReset_StartMode(Start_Mode_USER);
    NVIC_SystemReset();

    while (1) {}
}

void __attribute__((interrupt("WCH-Interrupt-fast"))) SysTick_Handler(void) {
    ++_ms;
    SysTick->SR = 0;
}
