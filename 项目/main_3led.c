/******************************************************************************
 * 文件名 : main_3led.c
 * 芯片   : STM32F103C8T6 (Cortex-M3)
 * 内容   : 第(1)(2)问——用 GPIOA / GPIOB / GPIOC 三个端口的引脚分别控制
 *          3 只 LED（红/绿/蓝），轮流点亮，每只亮 1 秒，循环往复。
 * 方式   : 纯寄存器操作（直接读写 RCC / GPIO / SysTick 寄存器），
 *          不使用 HAL 库、不使用标准外设库，只包含 <stdint.h>。
 * 接线   : PA0  -> 红色 LED（高电平点亮：PA0 —电阻— LED阳极，LED阴极 — GND）
 *          PB0  -> 绿色 LED（高电平点亮）
 *          PC13 -> 蓝色 LED（低电平点亮：3.3V — LED阳极，LED阴极 —电阻— PC13）
 ******************************************************************************/

/* ======================= 模块 0：头文件 ======================= */
#include <stdint.h>              /* 只需要 uint8_t / uint32_t 类型 */


/* ======================= 模块 1：寄存器地址定义 ======================= */
/* STM32F103 存储空间映射（见《STM32F10x 参考手册》第 2 章 Memory Map）
 *   APB2 外设基地址 : 0x4001 0000     GPIOA/GPIOB/GPIOC 挂在 APB2 上
 *   AHB  外设基地址 : 0x4002 0000     RCC 挂在 AHB 上
 */
#define APB2_BASE        0x40010000UL
#define AHB_BASE         0x40020000UL

#define GPIOA_BASE       (APB2_BASE + 0x0800UL)   /* = 0x40010800 */
#define GPIOB_BASE       (APB2_BASE + 0x0C00UL)   /* = 0x40010C00 */
#define GPIOC_BASE       (APB2_BASE + 0x1000UL)   /* = 0x40011000 */
#define RCC_BASE         (AHB_BASE  + 0x1000UL)   /* = 0x40021000 */

/* SysTick 定时器属于 Cortex-M3 内核私有外设，地址固定 */
#define SYSTICK_BASE     0xE000E010UL


/* ======================= 模块 2：寄存器结构体（把偏移量结构化） =======================
 * GPIO 端口内部 7 个寄存器的偏移量（相对端口基地址）：
 *   偏移   名称   作用
 *   0x00   CRL   端口配置低寄存器，配置 Px0 ~ Px7
 *   0x04   CRH   端口配置高寄存器，配置 Px8 ~ Px15
 *   0x08   IDR   端口输入数据寄存器（只读）
 *   0x0C   ODR   端口输出数据寄存器
 *   0x10   BSRR  端口位设置/清除寄存器（低16位写1置位，高16位写1复位）
 *   0x14   BRR   端口位清除寄存器（写1复位）
 *   0x18   LCKR  端口配置锁定寄存器
 */
typedef struct
{
    volatile uint32_t CRL;      /* 0x00 */
    volatile uint32_t CRH;      /* 0x04 */
    volatile uint32_t IDR;      /* 0x08 */
    volatile uint32_t ODR;      /* 0x0C */
    volatile uint32_t BSRR;     /* 0x10 */
    volatile uint32_t BRR;      /* 0x14 */
    volatile uint32_t LCKR;     /* 0x18 */
} GPIO_TypeDef;

typedef struct
{
    volatile uint32_t CTRL;     /* 0x00 控制及状态寄存器 */
    volatile uint32_t LOAD;     /* 0x04 重装载数值寄存器 */
    volatile uint32_t VAL;      /* 0x08 当前数值寄存器 */
    volatile uint32_t CALIB;    /* 0x0C 校准数值寄存器 */
} SysTick_Type;

/* 把上面定义的地址"变成"可以直接点操作的寄存器 */
#define GPIOA            ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB            ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC            ((GPIO_TypeDef *) GPIOC_BASE)
#define SysTick          ((SysTick_Type *) SYSTICK_BASE)

/* ---- RCC 的 3 个寄存器 ----
 * RCC_CR   偏移 0x00：bit0 HSION、bit1 HSIRDY、bit16 HSEON、bit17 HSERDY、bit24 PLLON
 * RCC_CFGR 偏移 0x04：bit[1:0] SW 时钟源选择、bit[3:2] SWS 时钟源状态(只读)、
 *                     bit[7:4] HPRE AHB 预分频、bit[16] PLLSRC、bit[17] PLLXTPRE、
 *                     bit[21:18] PLLMUL
 * RCC_APB2ENR 偏移 0x18：bit2 IOPAEN、bit3 IOPBEN、bit4 IOPCEN
 * 注意：复位后所有外设时钟都是关闭的，不使能时钟就写 GPIO 寄存器是无效的！
 */
#define RCC_CR           (*(volatile uint32_t *)(RCC_BASE + 0x00UL))
#define RCC_CFGR         (*(volatile uint32_t *)(RCC_BASE + 0x04UL))
#define RCC_APB2ENR      (*(volatile uint32_t *)(RCC_BASE + 0x18UL))


/* ======================= 模块 3：LED 引脚定义 ======================= */
#define LED_R_PORT       GPIOA      /* 红灯 */
#define LED_R_PIN        0          /* PA0  */

#define LED_G_PORT       GPIOB      /* 绿灯 */
#define LED_G_PIN        0          /* PB0  */

#define LED_B_PORT       GPIOC      /* 蓝灯 */
#define LED_B_PIN        13         /* PC13 */


/* ======================= 模块 4：底层驱动函数 ======================= */

/******************************************************************************
 * 函数名 : GPIO_ConfigPin
 * 功能   : 把指定端口的指定引脚配置成输出（直接操作 CRL / CRH 寄存器）
 * 参数   : port 端口指针；pin 引脚号 0~15；cnf 配置位 2 位；mode 模式位 2 位
 * 说明   : 每个引脚在 CRL/CRH 中占 4 位：高 2 位 CNF + 低 2 位 MODE
 *            pin0~7  -> CRL 的 bit[4*pin+3 : 4*pin]
 *            pin8~15 -> CRH 的 bit[4*(pin-8)+3 : 4*(pin-8)]
 *          CNF = 00 通用推挽输出 / 01 通用开漏输出 / 10 复用推挽 / 11 复用开漏
 *          MODE= 01 输出最大 10MHz / 10 输出最大 2MHz / 11 输出最大 50MHz
 ******************************************************************************/
void GPIO_ConfigPin(GPIO_TypeDef *port, uint8_t pin, uint32_t cnf, uint32_t mode)
{
    volatile uint32_t *reg;             /* 指向 CRL 或 CRH */
    uint32_t shift;                     /* 该引脚配置位在寄存器中的起始位置 */
    uint32_t tmp;

    if (pin < 8U)                       /* Px0 ~ Px7 用 CRL */
    {
        reg   = &port->CRL;
        shift = (uint32_t)pin * 4U;
    }
    else                                /* Px8 ~ Px15 用 CRH */
    {
        reg   = &port->CRH;
        shift = ((uint32_t)pin - 8U) * 4U;
    }

    tmp  = *reg;                        /* 读出现值 */
    tmp &= ~(0xFU << shift);            /* 先清零该引脚的 4 位（不影响其它引脚） */
    tmp |= ((cnf << 2) | mode) << shift;/* 写入 CNF(高2位) + MODE(低2位) */
    *reg = tmp;                         /* 写回寄存器 */
}

/******************************************************************************
 * 函数名 : LED_On / LED_Off
 * 功能   : 点亮 / 熄灭 LED，直接写 BSRR、BRR 寄存器
 * 说明   : BSRR 低 16 位：写 1 对应引脚输出高电平（置位）
 *          BSRR 高 16 位：写 1 对应引脚输出低电平（复位）
 *          BRR  低 16 位：写 1 对应引脚输出低电平（复位）
 *          用 BSRR/BRR 而不是读改写 ODR，好处是硬件原子操作、不影响其它引脚
 ******************************************************************************/
void LED_On_HighActive(GPIO_TypeDef *port, uint8_t pin)   /* 高电平点亮 */
{
    port->BSRR = (1UL << pin);
}

void LED_Off_HighActive(GPIO_TypeDef *port, uint8_t pin)  /* 高电平熄灭 */
{
    port->BRR = (1UL << pin);
}

void LED_On_LowActive(void)      /* PC13 蓝灯：低电平点亮 */
{
    GPIOC->BRR = (1UL << LED_B_PIN);
}

void LED_Off_LowActive(void)     /* PC13 蓝灯：高电平熄灭 */
{
    GPIOC->BSRR = (1UL << LED_B_PIN);
}

/******************************************************************************
 * 函数名 : HCLK_GetHz
 * 功能   : 读 RCC 寄存器，算出当前 CPU 实际运行频率 HCLK(Hz)
 * 为什么 : Keil 工程里如果用了 CMSIS 的 system_stm32f103xb.c，它默认会把时钟
 *          从复位后的 HSI 8MHz 切换到 HSE+PLL 的 72MHz；如果没使能，就还是 8MHz。
 *          延时函数的计数值必须跟着变，所以这里用寄存器把频率"问"出来，
 *          无论工程配成 8MHz 还是 72MHz，延时都是准确的 1 秒。
 *
 * 计算步骤（参考手册 RCC 章节）：
 *   1) 看 CFGR 的 SWS(bit[3:2]) 判断当前系统时钟来自 HSI / HSE / PLL
 *   2) 若是 PLL：输入源由 PLLSRC(bit16) 选 HSI/2=4MHz 或 HSE(=8MHz 晶振)；
 *      倍频系数由 PLLMUL(bit[21:18]) 决定：0000=x2 ... 1110=x16，1111 也是 x16
 *   3) 得到的 SYSCLK 再除以 AHB 预分频 HPRE(bit[7:4])，就是 HCLK
 ******************************************************************************/
uint32_t HCLK_GetHz(void)
{
    /* HPRE 编码表：bit[7:4] 的值 -> 分频系数 */
    const uint16_t ahb_div[16] = { 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U,
                                   2U, 4U, 8U, 16U, 64U, 128U, 256U, 512U };
    uint32_t cfgr = RCC_CFGR;
    uint32_t sysclk;

    switch ((cfgr >> 2) & 0x3UL)                    /* SWS：当前时钟源 */
    {
        case 0x2UL:                                 /* 10 = PLL 输出 */
        {
            uint32_t mul_code = (cfgr >> 18) & 0xFUL;       /* PLLMUL */
            uint32_t mul      = (mul_code == 0xFUL) ? 16UL : (mul_code + 2UL);
            uint32_t src;

            if ((cfgr >> 16) & 0x1UL)               /* PLLSRC = 1：用 HSE */
            {
                src = 8000000UL >> ((cfgr >> 17) & 0x1UL);  /* PLLXTPRE 再分频 */
            }
            else                                    /* PLLSRC = 0：用 HSI/2 */
            {
                src = 4000000UL;
            }
            sysclk = src * mul;
            break;
        }
        case 0x1UL:  sysclk = 8000000UL; break;     /* 01 = HSE 直接作为系统时钟 */
        default:     sysclk = 8000000UL; break;     /* 00 = HSI 8MHz（复位默认） */
    }

    return sysclk / (uint32_t)ahb_div[(cfgr >> 4) & 0xFUL];   /* HCLK */
}

/******************************************************************************
 * 函数名 : Delay_ms
 * 功能   : 毫秒级延时，用内核 SysTick 计数实现（不占用任何 TIM 外设）
 * 原理   : 置 CTRL 的 bit2 CLKSOURCE = 1，SysTick 时钟源 = HCLK；
 *          LOAD 里装 HCLK/1000 个计数就是 1ms；CTRL 的 bit16 COUNTFLAG
 *          在计数减到 0 时由硬件置 1，程序轮询该位即可判断 1ms 是否到。
 ******************************************************************************/
void Delay_ms(uint32_t ms)
{
    uint32_t i;
    uint32_t ticks;                  /* 1ms 需要的计数值 */

    ticks = HCLK_GetHz() / 1000UL;   /* 例如 72MHz -> 72000；8MHz -> 8000 */

    for (i = 0U; i < ms; i++)
    {
        SysTick->CTRL = 0UL;              /* 先停表：LOAD/VAL 在未使能状态下写才可靠 */
        SysTick->LOAD = ticks - 1UL;      /* 倒数 ticks 个数 = 1ms */
        SysTick->VAL  = 0UL;              /* 清当前计数值 */
        SysTick->CTRL = 0x00000005UL;     /* bit2 CLKSOURCE=1(用 HCLK)，bit0 ENABLE=1 */

        while ((SysTick->CTRL & 0x00010000UL) == 0UL)
        {
            /* 等 COUNTFLAG(bit16) 置 1，说明 1ms 时间到 */
        }
    }

    SysTick->CTRL = 0UL;              /* 关闭 SysTick */
    SysTick->VAL  = 0UL;
}


/* ======================= 模块 5：主函数 ======================= */
int main(void)
{
    /* ---- 5.1 打开 3 个 GPIO 端口的时钟 -------------------------------
     * 复位后 APB2ENR = 0x00000000，GPIOA/B/C 的时钟是关闭的，
     * 必须先置位，否则后面写 CRL/CRH/BSRR 都不会生效。
     */
    RCC_APB2ENR |= (1UL << 2)      /* IOPAEN : 使能 GPIOA 时钟 */
                 | (1UL << 3)      /* IOPBEN : 使能 GPIOB 时钟 */
                 | (1UL << 4);     /* IOPCEN : 使能 GPIOC 时钟 */

    /* ---- 5.2 配置 LED 引脚为通用推挽输出 -----------------------------
     * PA0 / PB0 在 CRL 中：CNF=00(推挽)，MODE=11(50MHz)  => 参数 (0, 3)
     * PC13 在 CRH 中     ：CNF=00(推挽)，MODE=10(2MHz)   => 参数 (0, 2)
     * 为什么 PC13 只用 2MHz？PC13/PC14/PC15 由低功耗 3.3V 域供电，
     * 数据手册标称灌/拉电流能力只有 3mA，输出速度越快越容易超过极限。
     */
    GPIO_ConfigPin(LED_R_PORT, LED_R_PIN, 0U, 3U);   /* PA0  推挽输出 50MHz */
    GPIO_ConfigPin(LED_G_PORT, LED_G_PIN, 0U, 3U);   /* PB0  推挽输出 50MHz */
    GPIO_ConfigPin(LED_B_PORT, LED_B_PIN, 0U, 2U);   /* PC13 推挽输出 2MHz  */

    /* ---- 5.3 上电先把 3 个灯全部熄灭，避免随机亮 ------ */
    LED_Off_HighActive(LED_R_PORT, LED_R_PIN);
    LED_Off_HighActive(LED_G_PORT, LED_G_PIN);
    LED_Off_LowActive();

    /* ---- 5.4 主循环：3 只灯轮流点亮，各亮 1 秒 --------- */
    while (1)
    {
        /* 第 1 秒：红灯（PA0）亮 */
        LED_On_HighActive(LED_R_PORT, LED_R_PIN);
        Delay_ms(1000);
        LED_Off_HighActive(LED_R_PORT, LED_R_PIN);

        /* 第 2 秒：绿灯（PB0）亮 */
        LED_On_HighActive(LED_G_PORT, LED_G_PIN);
        Delay_ms(1000);
        LED_Off_HighActive(LED_G_PORT, LED_G_PIN);

        /* 第 3 秒：蓝灯（PC13）亮 */
        LED_On_LowActive();
        Delay_ms(1000);
        LED_Off_LowActive();

        /* 之后回到循环开头，如此往复形成轮流闪烁 */
    }
}
void SystemInit(void)
{
    /* 空函数，本代码内部HCLK_GetHz自动读取时钟，不在此处配置时钟 */
}
