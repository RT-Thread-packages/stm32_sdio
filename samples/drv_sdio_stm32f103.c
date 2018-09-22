#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"
#include "drv_sdio_stm32f103.h"

#include <stm32_sdio.h>

//#define RT_USING_SDCARD

/* config those macro for your board */

//#define SDIO_USE_DMA_INT
#define SDIO_DMA_CONTROLLER_BASE  (SDIO_BASE)
#define SDIO_DMA_CHANNEL_BASE     (DMA2_Channel4)
#define SDIO_CLOCK_FREQ           (72U * 1000 * 1000)

static struct rt_mmcsd_host *sdio_host = RT_NULL;

#ifdef SDIO_USE_DMA_INT
void stm32_hw_dma_irq_process(DMA_Channel_TypeDef * Instance)
{
    uint32_t channel_index;

    DMA_Channel_TypeDef *dma_channel = Instance;
    DMA_TypeDef *dma_controller = NULL;

#if defined (STM32F101xE) || defined (STM32F101xG) || defined (STM32F103xE) || defined (STM32F103xG) || defined (STM32F100xE) || defined (STM32F105xC) || defined (STM32F107xC)
    /* calculation of the channel index */
    if ((uint32_t)(dma_channel) < (uint32_t)(DMA2_Channel1))
    {
        /* DMA1 */
        channel_index = (((uint32_t)dma_channel - (uint32_t)DMA1_Channel1) / ((uint32_t)DMA1_Channel2 - (uint32_t)DMA1_Channel1)) << 2;
        dma_controller = DMA1;
    }
    else
    {        
        /* DMA2 */
        channel_index = (((uint32_t)dma_channel - (uint32_t)DMA2_Channel1) / ((uint32_t)DMA2_Channel2 - (uint32_t)DMA2_Channel1)) << 2;
        dma_controller = DMA2;
    }
#else
    /* DMA1 */
    channel_index = (((uint32_t)dma_channel - (uint32_t)DMA1_Channel1) / ((uint32_t)DMA1_Channel2 - (uint32_t)DMA1_Channel1)) << 2;
    dma_controller = DMA1;
#endif

    if((dma_controller->ISR & (DMA_FLAG_GL1 << channel_index)) != RESET)
    {
        /* Transfer Error Interrupt management ***************************************/
        if((dma_controller->ISR & (DMA_FLAG_TE1 << channel_index)) != RESET)
        {
            //rt_kprintf("TE \n");

            if((dma_channel->CCR & DMA_IT_TE) != RESET)
            {
                /* Disable the transfer error interrupt */
                CLEAR_BIT(dma_channel->CCR , DMA_IT_TE);

                /* Clear the transfer error flag */
                dma_controller->IFCR = (DMA_FLAG_TE1 << channel_index);
            }
        }

        /* Half Transfer Complete Interrupt management ******************************/
        if((dma_controller->ISR & (DMA_FLAG_HT1 << channel_index)) != RESET)
        {
            //rt_kprintf("HT \n");

            if((dma_channel->CCR & DMA_IT_HT) != RESET)
            {
                /* Disable the half transfer interrupt if the DMA mode is not CIRCULAR */
                if((dma_channel->CCR & DMA_CCR_CIRC) == RESET)
                {
                    /* Disable the half transfer interrupt */
                    CLEAR_BIT(dma_channel->CCR , DMA_IT_HT);
                }

                /* Clear the half transfer complete flag */
                dma_controller->IFCR = (DMA_FLAG_HT1 << channel_index);
            }
        }

        /* Transfer Complete Interrupt management ***********************************/
        if((dma_controller->ISR & (DMA_FLAG_TC1 << channel_index)) != RESET)
        {
            //rt_kprintf("CT \n");

            if((dma_channel->CCR & DMA_IT_TC) != RESET)
            {
                /* Disable the transfer complete interrupt if the DMA mode is not CIRCULAR */
                if((dma_channel->CCR & DMA_CCR_CIRC) == RESET)
                {
                    /* Disable the transfer complete interrupt */
                    CLEAR_BIT(dma_channel->CCR , DMA_IT_TC);
                }

                /* Clear the transfer complete flag */
                dma_controller->IFCR = (DMA_FLAG_TC1 << channel_index);
            }
        }
    }
}

void DMA2_Channel4_5_IRQHandler(void)
{
    rt_interrupt_enter();
    stm32_hw_dma_irq_process(SDIO_DMA_CHANNEL_BASE);
    rt_interrupt_leave();
}
#endif

void SDIO_IRQHandler(void)
{
    rt_interrupt_enter();
    rthw_sdio_irq_process(sdio_host);
    rt_interrupt_leave();
}

rt_err_t stm32_hw_sdio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /*****************************Configuration GPIO****************************************************/
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitStruct.Pin   = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*****************************Configuration DMA*****************************************************/
    __HAL_RCC_DMA2_CLK_ENABLE();

#ifdef SDIO_USE_DMA_INT
    HAL_NVIC_SetPriority(DMA2_Channel4_5_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(DMA2_Channel4_5_IRQn);
#endif

    /*****************************Configuration SDIO****************************************************/
    __HAL_RCC_SDIO_CLK_ENABLE();

    HAL_NVIC_SetPriority(SDIO_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(SDIO_IRQn);

    return RT_EOK;
}

rt_err_t stm32_hw_dma_txconfig(rt_uint32_t *src, rt_uint32_t *dst, int size)
{
    uint32_t channel_index;
    uint32_t tmpreg = 0;

    DMA_Channel_TypeDef *dma_channel = SDIO_DMA_CHANNEL_BASE;
    DMA_TypeDef *dma_controller = NULL;
    DMA_InitTypeDef Init;

#if defined (STM32F101xE) || defined (STM32F101xG) || defined (STM32F103xE) || defined (STM32F103xG) || defined (STM32F100xE) || defined (STM32F105xC) || defined (STM32F107xC)
    /* calculation of the channel index */
    if ((uint32_t)(dma_channel) < (uint32_t)(DMA2_Channel1))
    {
        /* DMA1 */
        channel_index = (((uint32_t)dma_channel - (uint32_t)DMA1_Channel1) / ((uint32_t)DMA1_Channel2 - (uint32_t)DMA1_Channel1)) << 2;
        dma_controller = DMA1;
    }
    else
    {        
        /* DMA2 */
        channel_index = (((uint32_t)dma_channel - (uint32_t)DMA2_Channel1) / ((uint32_t)DMA2_Channel2 - (uint32_t)DMA2_Channel1)) << 2;
        dma_controller = DMA2;
    }
#else
    /* DMA1 */
    channel_index = (((uint32_t)dma_channel - (uint32_t)DMA1_Channel1) / ((uint32_t)DMA1_Channel2 - (uint32_t)DMA1_Channel1)) << 2;
    dma_controller = DMA1;
#endif

    /* Disable the selected DMA Channelx */
#ifdef SDIO_USE_DMA_INT
    dma_channel->CCR &= ~(DMA_CCR_EN|DMA_CCR_TCIE|DMA_CCR_HTIE|DMA_CCR_TEIE);
#else
    dma_channel->CCR &= ~(DMA_CCR_EN);
#endif

    /* Clear all flags */
    dma_controller->IFCR = ((DMA_ISR_GIF1|DMA_ISR_TCIF1|DMA_ISR_HTIF1|DMA_ISR_TEIF1) << channel_index);

    /*--------------------------- DMAy Channelx CCR Configuration -----------------*/
    /* Set dma direction */
    Init.Direction           = DMA_MEMORY_TO_PERIPH;
    /* Set dma mode  */
    Init.Mode                = DMA_NORMAL;
    /* Set peripheral inc */
    Init.PeriphInc           = DMA_PINC_DISABLE;
    /* Set memory inc */
    Init.MemInc              = DMA_MINC_ENABLE;
    /* Set peripheral data size */
    Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    /* Set memory data size */
    Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    /* Set dma priority */
    Init.Priority            = DMA_PRIORITY_HIGH;

    /* Get the DMAy_Channelx CCR value */
    tmpreg = dma_channel->CCR;

    /* Clear PL, MSIZE, PSIZE, MINC, PINC, CIRC and DIR bits */
    tmpreg &= ((uint32_t)~(DMA_CCR_PL | DMA_CCR_MSIZE | DMA_CCR_PSIZE | \
        DMA_CCR_MINC | DMA_CCR_PINC | DMA_CCR_CIRC | DMA_CCR_DIR));

    /* Configure DMAy Channelx: data transfer, data size, priority level and mode */
    tmpreg |= Init.Direction | Init.Mode | \
              Init.PeriphInc | Init.MemInc | \
              Init.PeriphDataAlignment | Init.MemDataAlignment | \
              Init.Priority;
    
    /* Write to DMAy Channelx CCR */
    dma_channel->CCR = tmpreg;

    /*--------------------------- DMAy Channelx CNDTR Configuration ---------------*/
    /* Write to DMAy Channelx CNDTR */
    dma_channel->CNDTR = size / 4;

    /*--------------------------- DMAy Channelx CPAR Configuration ----------------*/
    /* Write to DMAy Channelx CPAR */
    dma_channel->CPAR = (uint32_t)dst;

    /*--------------------------- DMAy Channelx CMAR Configuration ----------------*/
    /* Write to DMAy Channelx CMAR */
    dma_channel->CMAR = (uint32_t)src;

    /* Enable the selected DMAy Channelx */
#ifdef SDIO_USE_DMA_INT
    dma_channel->CCR |= (DMA_CCR_EN|DMA_CCR_TCIE|DMA_CCR_HTIE|DMA_CCR_TEIE);
#else
    dma_channel->CCR |= (DMA_CCR_EN);
#endif

    return RT_EOK;
}

rt_err_t stm32_hw_dma_rxconfig(rt_uint32_t *src, rt_uint32_t *dst, int size)
{
    uint32_t channel_index;
    uint32_t tmpreg = 0;

    DMA_Channel_TypeDef *dma_channel = SDIO_DMA_CHANNEL_BASE;
    DMA_TypeDef *dma_controller = NULL;
    DMA_InitTypeDef Init;

#if defined (STM32F101xE) || defined (STM32F101xG) || defined (STM32F103xE) || defined (STM32F103xG) || defined (STM32F100xE) || defined (STM32F105xC) || defined (STM32F107xC)
    /* calculation of the channel index */
    if ((uint32_t)(dma_channel) < (uint32_t)(DMA2_Channel1))
    {
        /* DMA1 */
        channel_index = (((uint32_t)dma_channel - (uint32_t)DMA1_Channel1) / ((uint32_t)DMA1_Channel2 - (uint32_t)DMA1_Channel1)) << 2;
        dma_controller = DMA1;
    }
    else
    {
        /* DMA2 */
        channel_index = (((uint32_t)dma_channel - (uint32_t)DMA2_Channel1) / ((uint32_t)DMA2_Channel2 - (uint32_t)DMA2_Channel1)) << 2;
        dma_controller = DMA2;
    }
#else
    /* DMA1 */
    channel_index = (((uint32_t)dma_channel - (uint32_t)DMA1_Channel1) / ((uint32_t)DMA1_Channel2 - (uint32_t)DMA1_Channel1)) << 2;
    dma_controller = DMA1;
#endif

    /* Disable the selected DMA Channelx */
#ifdef SDIO_USE_DMA_INT
    dma_channel->CCR &= ~(DMA_CCR_EN|DMA_CCR_TCIE|DMA_CCR_HTIE|DMA_CCR_TEIE);
#else
    dma_channel->CCR &= ~(DMA_CCR_EN);
#endif

    /* Clear all flags */
    dma_controller->IFCR = ((DMA_ISR_GIF1|DMA_ISR_TCIF1|DMA_ISR_HTIF1|DMA_ISR_TEIF1) << channel_index);

    /*--------------------------- DMAy Channelx CCR Configuration -----------------*/
    /* Set dma direction */
    Init.Direction           = DMA_PERIPH_TO_MEMORY;
    /* Set dma mode  */
    Init.Mode                = DMA_NORMAL;
    /* Set peripheral inc */
    Init.PeriphInc           = DMA_PINC_DISABLE;
    /* Set memory inc */
    Init.MemInc              = DMA_MINC_ENABLE;
    /* Set peripheral data size */
    Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    /* Set memory data size */
    Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    /* Set dma priority */
    Init.Priority            = DMA_PRIORITY_HIGH;

    /* Get the DMAy_Channelx CCR value */
    tmpreg = dma_channel->CCR;

    /* Clear PL, MSIZE, PSIZE, MINC, PINC, CIRC and DIR bits */
    tmpreg &= ((uint32_t)~(DMA_CCR_PL | DMA_CCR_MSIZE | DMA_CCR_PSIZE | \
        DMA_CCR_MINC | DMA_CCR_PINC | DMA_CCR_CIRC | DMA_CCR_DIR));

    /* Configure DMAy Channelx: data transfer, data size, priority level and mode */
    tmpreg |= Init.Direction | Init.Mode | \
              Init.PeriphInc | Init.MemInc | \
              Init.PeriphDataAlignment | Init.MemDataAlignment | \
              Init.Priority;
    
    /* Write to DMAy Channelx CCR */
    dma_channel->CCR = tmpreg;

    /*--------------------------- DMAy Channelx CNDTR Configuration ---------------*/
    /* Write to DMAy Channelx CNDTR */
    dma_channel->CNDTR = size / 4;

    /*--------------------------- DMAy Channelx CPAR Configuration ----------------*/
    /* Write to DMAy Channelx CPAR */
    dma_channel->CPAR = (uint32_t)src;

    /*--------------------------- DMAy Channelx CMAR Configuration ----------------*/
    /* Write to DMAy Channelx CMAR */
    dma_channel->CMAR = (uint32_t)dst;

    /* Enable the selected DMAy Channelx */
#ifdef SDIO_USE_DMA_INT
    dma_channel->CCR |= (DMA_CCR_EN|DMA_CCR_TCIE|DMA_CCR_HTIE|DMA_CCR_TEIE);
#else
    dma_channel->CCR |= (DMA_CCR_EN);
#endif

    return RT_EOK;
}

rt_uint32_t stm32_hw_sdio_clk_get(struct stm32_sdio *hw_sdio)
{
    return SDIO_CLOCK_FREQ;
}

rt_err_t rt_stm32_hw_sdio_init(void)
{
    /* set io,clock and dma peripheral */
    if(stm32_hw_sdio_init() != RT_EOK)
    {
        rt_kprintf("stm32_hw_sdio_init error! \n");
        return RT_EIO;
    }

    /* create sdio driver */
    {
        struct stm32_sdio_des sdio_des;

        sdio_des.hw_sdio = (struct stm32_sdio *)SDIO_DMA_CONTROLLER_BASE;
        sdio_des.txconfig = &stm32_hw_dma_txconfig;
        sdio_des.rxconfig = &stm32_hw_dma_rxconfig;
        sdio_des.clk_get = &stm32_hw_sdio_clk_get;

        sdio_host = sdio_host_create(&sdio_des);
        if(RT_NULL == sdio_host)
        {
            rt_kprintf("sdio_host_create error! \n");
            return RT_EIO;
        }
    }

    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_stm32_hw_sdio_init);
