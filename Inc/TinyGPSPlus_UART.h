/*
 * TinyGPSPlus_UART.h  –  STM32 HAL UART integration helper
 *
 * Provides two reception strategies:
 *
 *  1. POLLING  (TinyGPSPlusUART_Polling)
 *     Call update() repeatedly from your main loop.
 *     Uses HAL_UART_Receive() with timeout = 0 (non-blocking).
 *     Simple but character-by-character.
 *
 *  2. INTERRUPT / DMA  (TinyGPSPlusUART_IRQ)
 *     Uses a circular DMA or idle-line interrupt buffer.
 *     You call feedBuffer() from your HAL callback, then
 *     call update() from the main loop to drain into the parser.
 *
 * Target: STM32F411 (STM32F4xx HAL)
 */

#pragma once

#include "TinyGPSPlus.h"
#include "stm32f4xx_hal.h"

/* ════════════════════════════════════════════════════════════════
 *  Strategy 1 – Polling (simplest, no interrupts needed)
 * ════════════════════════════════════════════════════════════════
 *
 *  Usage:
 *
 *    extern UART_HandleTypeDef huart1;
 *    TinyGPSPlusUART_Polling gpsUart(huart1);
 *
 *    // In main loop:
 *    while (true) {
 *        gpsUart.update();   // feed any waiting characters
 *
 *        if (gpsUart.gps.location.isUpdated()) {
 *            double lat = gpsUart.gps.location.lat();
 *            double lon = gpsUart.gps.location.lng();
 *        }
 *    }
 */
class TinyGPSPlusUART_Polling
{
public:
    TinyGPSPlus gps;

    explicit TinyGPSPlusUART_Polling(UART_HandleTypeDef &huart)
        : _huart(huart) {}

    /**
     * @brief Drain all immediately-available UART bytes into the parser.
     *        Call this as often as possible from your main loop.
     * @return Number of characters consumed this call.
     */
    uint32_t update()
    {
        uint32_t count = 0;
        uint8_t  ch;
        while (HAL_UART_Receive(&_huart, &ch, 1, 0) == HAL_OK)
        {
            gps.encode(static_cast<char>(ch));
            ++count;
        }
        return count;
    }

private:
    UART_HandleTypeDef &_huart;
};


/* ════════════════════════════════════════════════════════════════
 *  Strategy 2 – Interrupt / DMA ring-buffer
 * ════════════════════════════════════════════════════════════════
 *
 *  Setup (one-time):
 *
 *    // 1. Declare the object (global or static)
 *    static TinyGPSPlusUART_IRQ gpsUart(huart1, 256);
 *
 *    // 2. Start DMA reception (or UART RX interrupt)
 *    //    For idle-line + DMA (recommended):
 *    HAL_UARTEx_ReceiveToIdle_DMA(&huart1,
 *                                  gpsUart.rxBuf(),
 *                                  gpsUart.rxBufSize());
 *    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT); // optional
 *
 *  Callbacks (in stm32f4xx_it.c or user callback):
 *
 *    void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
 *                                    uint16_t Size)
 *    {
 *        if (huart->Instance == USART1)
 *            gpsUart.feedBuffer(Size);
 *    }
 *
 *  Main loop:
 *
 *    while (true) {
 *        gpsUart.update();           // drain ring buffer → parser
 *        if (gpsUart.gps.location.isUpdated()) { ... }
 *    }
 *
 *  NOTE: This implementation is single-core; ISR and main loop access
 *        shared state.  If your application uses RTOS, add a mutex or
 *        use a task notification instead.
 */
class TinyGPSPlusUART_IRQ
{
public:
    TinyGPSPlus gps;

    /**
     * @param huart      Reference to the HAL UART handle.
     * @param bufSize    Size of the internal ring buffer (bytes).
     *                   Must be a power of two for the mask trick to work,
     *                   but any size works with the modulo fallback.
     */
    TinyGPSPlusUART_IRQ(UART_HandleTypeDef &huart, uint16_t bufSize = 256)
        : _huart(huart)
        , _bufSize(bufSize)
        , _dmaPrevPos(0)
        , _writeIdx(0)
        , _readIdx(0)
    {
        _buf = new uint8_t[bufSize];
    }

    ~TinyGPSPlusUART_IRQ() { delete[] _buf; }

    /* ── accessors for the DMA receive buffer ─────────────────
     * Pass these to HAL_UARTEx_ReceiveToIdle_DMA() or
     * HAL_UART_Receive_DMA().
     */
    uint8_t *rxBuf()     { return _buf; }
    uint16_t rxBufSize() { return _bufSize; }

    /**
     * @brief Call from HAL_UARTEx_RxEventCallback() to record how many
     *        bytes the DMA has written so far.
     *
     *        The DMA writes linearly into _buf[0 … _bufSize-1].
     *        This function converts the absolute DMA position (Size) into
     *        new bytes and appends them to the logical ring.
     *
     * @param dmaTotalBytes  The 'Size' parameter from the HAL callback
     *                       (total bytes written since last HAL_UART… call).
     */
    void feedBuffer(uint16_t dmaTotalBytes)
    {
        /* Handle wrap of DMA position (circular DMA mode) */
        if (dmaTotalBytes >= _dmaPrevPos)
        {
            /* Normal case: new data from _dmaPrevPos to dmaTotalBytes */
            for (uint16_t i = _dmaPrevPos; i < dmaTotalBytes; ++i)
                _pushByte(_buf[i]);
        }
        else
        {
            /* DMA counter wrapped around */
            for (uint16_t i = _dmaPrevPos; i < _bufSize; ++i)
                _pushByte(_buf[i]);
            for (uint16_t i = 0; i < dmaTotalBytes; ++i)
                _pushByte(_buf[i]);
        }
        _dmaPrevPos = dmaTotalBytes;
    }

    /**
     * @brief Drain the ring buffer into the GPS parser.
     *        Call from your main loop (or a low-priority RTOS task).
     * @return Number of characters fed to the parser this call.
     */
    uint32_t update()
    {
        uint32_t count = 0;
        while (_readIdx != _writeIdx)
        {
            gps.encode(static_cast<char>(_ringBuf[_readIdx]));
            _readIdx = (_readIdx + 1) % RING_SIZE;
            ++count;
        }
        return count;
    }

private:
    UART_HandleTypeDef &_huart;
    uint16_t            _bufSize;

    /* DMA-receive staging buffer */
    uint8_t  *_buf;
    uint16_t  _dmaPrevPos;

    /* Software ring buffer (logical FIFO between ISR and main loop) */
    static constexpr uint16_t RING_SIZE = 512;
    volatile uint8_t  _ringBuf[RING_SIZE];
    volatile uint16_t _writeIdx;
    volatile uint16_t _readIdx;

    void _pushByte(uint8_t b)
    {
        uint16_t next = (_writeIdx + 1) % RING_SIZE;
        if (next != _readIdx)           /* drop on overflow */
        {
            _ringBuf[_writeIdx] = b;
            _writeIdx = next;
        }
    }
};
