/*
 * USBSS/USBHS EP2 <-> USART1 byte-stream bridge, for CH32H417 / Run_Core_V5F.
 * USART1 TX: DMA1 channel 4, DMAMUX request 85.
 * USART1 RX: DMA1 channel 5, DMAMUX request 86, circular DMA.
 * USBSS DMA and UART DMA buffers reside in the existing .da1 SRAM section.
 * No blocking UART work is performed in the USB interrupt.
 * USB reset / deconfiguration / CLEAR_HALT discard pending affected data.
 * This module deliberately does not treat an idle gap as a protocol frame.
 */
#include "usbss_uart1_bridge.h"
#include "ch32h417_usbss_device.h"
#include "ch32h417_usbhs_device.h"
#include "ch32h417_usb.h"
#include <string.h>
#if defined(Core_V5F)
#if UART1_BRIDGE_RX_SIZE != 8192U
#error Review the power-of-two ring and DMA service interval before changing size
#endif
static __attribute__((aligned(32), section(".da1")))
    uint8_t uart_rx_dma[UART1_BRIDGE_RX_SIZE];
static __attribute__((aligned(32), section(".da1")))
    uint8_t ep2_out_buf[UART1_BRIDGE_USB_MPS];
static __attribute__((aligned(32), section(".da1")))
    uint8_t ep2_in_buf[UART1_BRIDGE_USB_MPS];
volatile UART1_BridgeStats_t UART1_BridgeStats;
static volatile uint32_t rx_wraps;
static uint32_t rx_consumed;
static uint32_t rx_snapshot;
static volatile uint8_t initialized, configured, rx_fault;
static uint8_t usb_hs;
static uint16_t usb_mps = UART1_BRIDGE_USB_MPS;
static uint8_t out_halted(void)
{
    return usb_hs ? ((USBHSD->UEP2_RX_CTRL & USBHS_UEP_R_RES_MASK) == USBHS_UEP_R_RES_STALL)
                  : ((USBSSD->EP2_RX.UEP_RX_CR & USBSS_EP_RX_HALT) != 0);
}
static uint8_t in_halted(void)
{
    return usb_hs ? ((USBHSD->UEP2_TX_CTRL & USBHS_UEP_T_RES_MASK) == USBHS_UEP_T_RES_STALL)
                  : ((USBSSD->EP2_TX.UEP_TX_CR & USBSS_EP_TX_HALT) != 0);
}
static volatile uint8_t out_armed, tx_dma_busy, in_busy;
static volatile uint16_t out_length, in_length;
/* QingKe global interrupt state is in CSR 0x800, as in core_riscv.h.
 * Preserve the caller's state, including when called from a USB ISR.
 */
static uint32_t irq_save(void)
{
    uint32_t previous;
    __asm volatile("csrrc %0, 0x800, %1" : "=r"(previous) : "r"(0x88U) : "memory");
    return previous & 0x88U;
}
static void irq_restore(uint32_t previous)
{
    __asm volatile("csrs 0x800, %0" : : "r"(previous) : "memory");
}
static void dma_fence(void)
{
    __asm volatile("fence iorw, iorw" : : : "memory");
}
/* Called with CPU interrupts masked. DMA continues running.
 * Stable TC samples account for a wrap pending service by the DMA ISR.
 * Must be serviced at least once per ring period (8192 bytes x 10 bits / baud:
 * about 26 ms at 3.125 MBd, 13 ms at 6.25 MBd).
 */
static uint32_t rx_produced(void)
{
    uint32_t before, after, remaining, attempt;
    for(attempt = 0; attempt < 8U; ++attempt)
    {
        before = DMA1->INTFR & DMA1_FLAG_TC5;
        remaining = DMA1_Channel5->CNTR;
        after = DMA1->INTFR & DMA1_FLAG_TC5;
        if(before == after && remaining != 0U)
        {
            dma_fence();
            rx_snapshot = (rx_wraps + (after ? 1U : 0U)) * UART1_BRIDGE_RX_SIZE
                          + UART1_BRIDGE_RX_SIZE - remaining;
            break;
        }
    }
    /* Retry on the next Task call if the DMA reload was transiently unstable. */
    return rx_snapshot;
}
void DMA1_Channel5_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel5_IRQHandler(void)
{
    uint32_t key = irq_save();
    if(DMA1->INTFR & DMA1_FLAG_TE5)
    {
        DMA_Cmd(DMA1_Channel5, DISABLE);
        DMA_ClearFlag(DMA1, DMA1_FLAG_GL5);
        UART1_BridgeStats.dma_rx_errors++;
        rx_fault = 1U; /* Do not silently forward corrupted DMA data. */
        irq_restore(key);
        return;
    }
    if(DMA1->INTFR & DMA1_FLAG_TC5)
    {
        DMA_ClearFlag(DMA1, DMA1_FLAG_TC5);
        rx_wraps++;
    }
    irq_restore(key);
}
static void arm_out(void)
{
    if(!configured || out_armed || out_length || tx_dma_busy ||
       out_halted())
        return;
    if(usb_hs)
    {
        USBHSD->UEP2_RX_DMA = (uint32_t)ep2_out_buf;
        USBHSD->UEP2_RX_LEN = 0;
        out_armed = 1U;
        dma_fence();
        USBHSD->UEP2_RX_CTRL = (USBHSD->UEP2_RX_CTRL & USBHS_UEP_R_TOG_MASK) | USBHS_UEP_R_RES_ACK;
        return;
    }
    USBSSD->EP2_RX.UEP_RX_DMA = (uint32_t)ep2_out_buf;
    USBSSD->EP2_RX.UEP_RX_DMA_OFS = UART1_BRIDGE_USB_MPS;
    out_armed = 1U;
    dma_fence();
        /* Commit only ONE chain, with ONE packet. No second OUT buffer is armed. */
    USBSSD->EP2_RX.UEP_RX_CHAIN_MAX_NUMP = 1U;
}
static void stop_uart_tx(void)
{
    if(initialized)
    {
        DMA_Cmd(DMA1_Channel4, DISABLE);
        DMA_ClearFlag(DMA1, DMA1_FLAG_GL4);
    }
    tx_dma_busy = 0U;
    out_length = 0U;
    out_armed = 0U;
}
void UART1_Bridge_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio = {0};
    USART_InitTypeDef uart = {0};
    DMA_InitTypeDef dma = {0};
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO | RCC_HB2Periph_GPIOA |
                         RCC_HB2Periph_USART1, ENABLE);
    RCC_HBPeriphClockCmd(RCC_HBPeriph_DMA1, ENABLE);
    USART_Cmd(USART1, DISABLE);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF7);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF7);
    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);
    uart.USART_BaudRate = baudrate;
    uart.USART_WordLength = USART_WordLength_8b;
    uart.USART_StopBits = USART_StopBits_1;
    uart.USART_Parity = USART_Parity_No;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &uart);
    DMA_DeInit(DMA1_Channel4);
    DMA_DeInit(DMA1_Channel5);
    dma.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DATAR;
    dma.DMA_Memory0BaseAddr = (uint32_t)ep2_out_buf;
    dma.DMA_DIR = DMA_DIR_PeripheralDST;
    dma.DMA_BufferSize = 1;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Normal;
     dma.DMA_Priority = DMA_Priority_High;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel4, &dma);
    dma.DMA_Memory0BaseAddr = (uint32_t)uart_rx_dma;
    dma.DMA_DIR = DMA_DIR_PeripheralSRC;
    dma.DMA_BufferSize = UART1_BRIDGE_RX_SIZE;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_Init(DMA1_Channel5, &dma);
    DMA_MuxChannelConfig(DMA_MuxChannel4, 85U);
    DMA_MuxChannelConfig(DMA_MuxChannel5, 86U);
    DMA_ClearFlag(DMA1, DMA1_FLAG_GL4 | DMA1_FLAG_GL5);
    DMA_ITConfig(DMA1_Channel5, DMA_IT_TC | DMA_IT_TE, ENABLE);
    NVIC_SetPriority(DMA1_Channel5_IRQn, 0);
    NVIC_EnableIRQ(DMA1_Channel5_IRQn);
    rx_wraps = rx_consumed = rx_snapshot = 0;
    configured = rx_fault = out_armed = tx_dma_busy = in_busy = 0;
    out_length = in_length = 0;
    memset((void *)&UART1_BridgeStats, 0, sizeof(UART1_BridgeStats));
    initialized = 1U;
    DMA_Cmd(DMA1_Channel5, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);
    USART_Cmd(USART1, ENABLE);
}
/* Software-only EP reset. USB layer subsequently clears hardware chains. */
static void bridge_reset(void)
{
    uint32_t key = irq_save();
    configured = 0U;
    stop_uart_tx();
    in_busy = 0U;
    in_length = 0U;
    if(initialized && !rx_fault)
        rx_consumed = rx_produced();
    irq_restore(key);
}
void UART1_Bridge_USBReset(void)
{
    if(!usb_hs) bridge_reset();
}
void UART1_Bridge_HSReset(void)
{
    if(usb_hs) bridge_reset();
}
static void bridge_configure(uint8_t enabled, uint8_t high_speed_controller)
{
    uint32_t key = irq_save();
    configured = 0U;
    stop_uart_tx();
    in_busy = 0U;
    in_length = 0U;
    if(initialized && !rx_fault)
        rx_consumed = rx_produced();
    usb_hs = high_speed_controller;
    usb_mps = usb_hs ? USBHS_DevMaxPackLen : UART1_BRIDGE_USB_MPS;
    if(usb_hs)
    {
        USBHSD->UEP2_MAX_LEN = usb_mps;
        USBHSD->UEP2_RX_SIZE = usb_mps;
        USBHSD->UEP2_RX_LEN = 0;
        USBHSD->UEP2_TX_LEN = 0;
        USBHSD->UEP2_RX_CTRL = USBHS_UEP_R_RES_NAK;
        USBHSD->UEP2_TX_CTRL = USBHS_UEP_T_RES_NAK;
    }
    else
    {
        USBSSD->EP2_TX.UEP_TX_CFG = USBSS_EP_TX_CHAIN_AUTO |
            USBSS_EP_TX_ERDY_AUTO | USBSS_EP_TX_SEQ_AUTO;
        USBSSD->EP2_RX.UEP_RX_CFG = USBSS_EP_RX_CHAIN_AUTO |
            USBSS_EP_RX_ERDY_AUTO | USBSS_EP_RX_SEQ_AUTO;
        USBSSD->EP2_TX.UEP_TX_CR = USBSS_EP_TX_CLR | USBSS_EP_TX_CHAIN_CLR | 1U;
        USBSSD->EP2_RX.UEP_RX_CR = USBSS_EP_RX_CLR | USBSS_EP_RX_CHAIN_CLR | 1U;
    }
    configured = (enabled && initialized) ? 1U : 0U;
    arm_out();
    irq_restore(key);
}
void UART1_Bridge_SetConfigured(uint8_t enabled)
{
    bridge_configure(enabled, 0U);
}
void UART1_Bridge_HSSetConfigured(uint8_t enabled)
{
    bridge_configure(enabled, 1U);
}
void UART1_Bridge_ClearHalt(uint8_t endpoint_address)
{
    uint32_t key = irq_save();
    if(endpoint_address == 0x82U)
    {
        if(usb_hs)
        {
            USBHSD->UEP2_TX_CTRL = USBHS_UEP_T_RES_NAK;
            USBHSD->UEP2_TX_LEN = 0;
        }
        else USBSSD->EP2_TX.UEP_TX_CR = USBSS_EP_TX_CLR | USBSS_EP_TX_CHAIN_CLR | 1U;
        in_busy = 0U;
        in_length = 0U;
    }
    else if(endpoint_address == 0x02U)
    {
        stop_uart_tx();
        if(usb_hs)
        {
            USBHSD->UEP2_RX_CTRL = USBHS_UEP_R_RES_NAK;
            USBHSD->UEP2_RX_LEN = 0;
        }
        else USBSSD->EP2_RX.UEP_RX_CR = USBSS_EP_RX_CLR | USBSS_EP_RX_CHAIN_CLR | 1U;
        arm_out();
    }
    irq_restore(key);
}
void UART1_Bridge_OutIRQ(void)
{
    uint32_t count, length;
    if(usb_hs)
    {
        USBHSD->UEP2_RX_CTRL &= ~USBHS_UEP_R_DONE;
        if(!(USBHSD->UEP2_RX_CTRL & USBHS_UEP_R_TOG_MATCH)) return;
        length = USBHSD->UEP2_RX_LEN;
        count = 1U;
        USBHSD->UEP2_RX_CTRL = (USBHSD->UEP2_RX_CTRL & USBHS_UEP_R_TOG_MASK) | USBHS_UEP_R_RES_NAK;
        USBHSD->UEP2_RX_LEN = 0;
    }
    else
    {
        count = USBSSD->EP2_RX.UEP_RX_CHAIN_NUMP;
        length = USBSSD->EP2_RX.UEP_RX_CHAIN_LEN;
        USBSSD->EP2_RX.UEP_RX_CHAIN_ST |= USBSS_EP_RX_CHAIN_IF;
    }
    dma_fence();
    out_armed = 0U;
    if(count != 1U || length > usb_mps || out_length || tx_dma_busy)
    {
        UART1_BridgeStats.usb_out_errors++;
        return;
    }
    if(configured)
    {
        out_length = (uint16_t)length;
        UART1_BridgeStats.usb_out_bytes += length;
    }
    /* Do not rearm here: UART DMA must finish reading ep2_out_buf first.
     * A zero-length OUT packet is rearmed by Task without starting UART DMA.
     */
}
void UART1_Bridge_InIRQ(void)
{
    if(usb_hs)
    {
        USBHSD->UEP2_TX_CTRL = (USBHSD->UEP2_TX_CTRL & USBHS_UEP_T_TOG_MASK) | USBHS_UEP_T_RES_NAK;
        USBHSD->UEP2_TX_LEN = 0;
    }
    else USBSSD->EP2_TX.UEP_TX_CHAIN_ST |= USBSS_EP_TX_CHAIN_IF;
    if(in_busy)
        UART1_BridgeStats.usb_in_bytes += in_length;
    in_busy = 0U;
    in_length = 0U;
}
void UART1_Bridge_Task(void)
{
    uint32_t key, produced, available, count, i, after, uart_status;
    if(!initialized)
        return;
    key = irq_save();
    /* No RXNE/IDLE handler reads DATAR in normal operation: DMA owns RX.
     * Only recover the status-read/data-read sequence on a UART error.
     */
    uart_status = USART1->STATR;
    if(uart_status & (USART_STATR_ORE | USART_STATR_NE | USART_STATR_FE | USART_STATR_PE))
    {
        (void)USART1->DATAR;
        UART1_BridgeStats.uart_error_events++;
    }
    if(!configured || !(usb_hs ? USBHS_DevEnumStatus : USBSS_DevEnumStatus))
    {
        if(configured)
        {
            configured = 0U;
            stop_uart_tx();
            in_busy = 0U;
        }
        if(!rx_fault)
            rx_consumed = rx_produced();
        irq_restore(key);
        return;
    }
    if(tx_dma_busy)
    {
        if(DMA1->INTFR & DMA1_FLAG_TE4)
        {
            UART1_BridgeStats.dma_tx_errors++;
            stop_uart_tx();
        }
        else if(DMA1->INTFR & DMA1_FLAG_TC4)
        {
            UART1_BridgeStats.uart_tx_dma_bytes += out_length;
            /* TC4 means all buffer bytes reached USART DATAR. The buffer
             * can be reused; the last byte may still be on the wire.
             */
            stop_uart_tx();
        }
    }
    if(!tx_dma_busy && out_length)
    {
        DMA_Cmd(DMA1_Channel4, DISABLE);
        DMA_ClearFlag(DMA1, DMA1_FLAG_GL4);
        DMA1_Channel4->MADDR = (uint32_t)ep2_out_buf;
        DMA_SetCurrDataCounter(DMA1_Channel4, out_length);
        tx_dma_busy = 1U;
        dma_fence();
        DMA_Cmd(DMA1_Channel4, ENABLE);
    }
    arm_out();
    if(rx_fault)
    {
        irq_restore(key);
        return;
    }
    produced = rx_produced();
    available = produced - rx_consumed;
    if(available > UART1_BRIDGE_RX_SIZE)
    {
        /* Host stopped reading: retain the newest half-ring, record loss. */
        count = available - UART1_BRIDGE_RX_SIZE / 2U;
        UART1_BridgeStats.rx_dropped_bytes += count;
        rx_consumed += count;
        available -= count;
    }
    if(!in_busy && available &&
       !in_halted())
    {
        count = available >= usb_mps ? usb_mps - 1U : available;
        for(i = 0; i < count; ++i)
            ep2_in_buf[i] = ((volatile uint8_t *)uart_rx_dma)
                [(rx_consumed + i) & (UART1_BRIDGE_RX_SIZE - 1U)];
        after = rx_produced();
        if(after - rx_consumed > UART1_BRIDGE_RX_SIZE)
        {
            /* DMA overtook the source during the copy; do not send a mixed packet. */
            UART1_BridgeStats.rx_dropped_bytes += after - rx_consumed;
            rx_consumed = after;
            irq_restore(key);
            return;
        }
        if(usb_hs)
        {
            USBHSD->UEP2_TX_DMA = (uint32_t)ep2_in_buf;
            USBHSD->UEP2_TX_LEN = (uint16_t)count;
        }
        else
        {
            USBSSD->EP2_TX.UEP_TX_DMA = (uint32_t)ep2_in_buf;
            USBSSD->EP2_TX.UEP_TX_DMA_OFS = UART1_BRIDGE_USB_MPS;
            USBSSD->EP2_TX.UEP_TX_CHAIN_LEN = (uint16_t)count;
        }
        in_length = (uint16_t)count;
        in_busy = 1U;
        rx_consumed += count;
        dma_fence();
        /* Commit last, after the buffer and software state are ready. */
        if(usb_hs)
            USBHSD->UEP2_TX_CTRL = (USBHSD->UEP2_TX_CTRL & USBHS_UEP_T_TOG_MASK) | USBHS_UEP_T_RES_ACK;
        else USBSSD->EP2_TX.UEP_TX_CHAIN_EXP_NUMP = 1U;
    }
    irq_restore(key);
}
#endif /* Core_V5F: the V3F boot project must not own USART1 or DMA1 ch4/ch5. */