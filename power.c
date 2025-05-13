/****************************************
  软件名称：电池管理器下位机软件
************************************************************************************************/

#include <reg52.h> /* 标准头文件引用 */
#include <absacc.h>
#include <intrins.h>
#include <math.h>

#define TRUE 0xAA  /* 定义真值 */
#define FALSE 0x55 /* 定义假值 */

#define CAN_ACR 0xC6 /* 接收码寄存器 */
#define CAN_AMR 0xC0 /* 接收屏蔽寄存器  */

#define SILENT_TIME 4 /* 最大总线沉寂时间(s) */

#define K1 9.0779 /* 充电电流变换系数 */
#define B1 -0.0552
#define K2 13.386 /* 放电电流变换系数 */
#define B2 -0.0832

#define K31 6.0155 /* 功率电流1 */
#define B31 0.1005
#define K32 6.0206 /* 功率电流2 */
#define B32 0.0462
#define K33 5.9919 /* 功率电流3 */
#define B33 -0.0253

#define K4 10.02 /* 负载电流 */
#define B4 0
#define K5 6.11 /* 蓄电池组电压 */
#define B5 0.072

#define FULLPOWER_H 0xC3 /* 满电量下传值高字节 */
#define FULLPOWER_L 0x50 /* 满电量下传值低字节 */

#define FULLPOWER 150 /* 满电量150Ah */

#define AD558_VSET 128        /* AD558上电输出默认值  */
#define GENERATRIX_VSET 0x92  /* 母线电压报警门限值(24V)           -146   Y = 8.1993X - 0.0025    */
#define ACCUMULATOR_VSET 0xBC /* 蓄电池组电压报警门限默认值(23.1V) -188   Y = 6.11X   + 0.072     */
#define ACCUMULATOR_VMIN 0x9D /* 蓄电池组电压报警门限默认值(19.2V) -157                           */
#define ACCUMULATOR_VMAX 0xCE /* 蓄电池组电压报警门限默认值(25.2V) -206                          */

#define TEMPE_22C 0x4C /* 主回路加热关闭门限默认值(22℃)   */
#define TEMPE_19C 0x53 /* 主回路加热开启门限默认值(19℃)   */
#define TEMPE_21C 0x4E /* 备回路加热关闭门限默认值(21℃)   */
#define TEMPE_18C 0x55 /* 备回路加热开启门限默认值(18℃)   */
#define TEMPE_10C 0x69 /* 控温门限上注最小值(10℃)         */
#define TEMPE_35C 0x35 /* 控温安全模式判读门限、控温门限上注最大值(35℃)  */

#define BUS_ADDR 0x5000          /* 总线地址         */
#define ONOFF_ADDR XBYTE[0x4800] /* ONOFF指令地址     */
#define AN_ADDR XBYTE[0x4400]    /* 模拟量地址        */
#define AD574H XBYTE[0x6000]     /* AD574转换高地址   */
#define AD574L XBYTE[0x6001]     /* AD574转换低地址   */
#define AD558 XBYTE[0x4C00]      /* AD558地址         */
#define EQU_ADDR XBYTE[0x5800]   /* 均衡控制输出地址   */

const unsigned char code rsTab8[57] = {
    /* 8bit模拟量采集码表                   */
    0x21, 0x0B,                                     /* N01 N02  */
    0x3D, 0x3B, 0x33,                               /* N09-11   */
    0x1B, 0x01, 0x10, 0x1C, 0x12, 0x1E, 0x0F, 0x1A, /* N15-38   */
    0x2A, 0x29, 0x1F, 0x20, 0x26, 0x23, 0x03, 0x0E,
    0x18, 0x14, 0x0D, 0x24, 0x1D, 0x0C, 0x19, 0x11,
    0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A,       /* N39-45  	*/
    0x35, 0x3E, 0x3F, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F, /* N46-53   */
    0x36, 0x3C, 0x39, 0x3A, 0x31, 0x32,             /* N54-59   */
    0x30,                                           /* N64      */
    0x27, 0x28, 0x38, 0x22, 0x37, 0x34              /* N73      */
};

const unsigned char code rsTab12[6] = {0x16, 0x25, 0x17, 0x02, 0x13, 0x15};    /* 参数采集码表 */
const unsigned char code onoffTab[16] = {0x04, 0x03, 0x0C, 0x0F, 0x06, 0x05, 0x0A, 0x0B, 0x08, 0x09, 0x0E, 0x01, 0x0D, 0x07, 0x02, 0x00}; /* 间接指令通码表  */

const unsigned char code heatTab[2][3] = {
    {0x41, TEMPE_22C, TEMPE_19C},
    {0x41, TEMPE_21C, TEMPE_18C}};

const unsigned char code EQU_CLOSE_Tab[7] = {0xFE, 0xFD, 0xFB, 0xF7, 0xEF, 0xDF, 0xBF}; /* 均衡控制关闭 */
const unsigned char code EQU_OPEN_Tab[7] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40};  /* 均衡控制开启 */

sbit onoffEn = P1 ^ (unsigned int)(0x06);      /* 指令输出使能   */
sbit fgRstType = P1 ^ (unsigned int)(0x04);    /* 热复位标识，0=冷复位 1=热复位  */
sbit dog = P1 ^ (unsigned int)(0x05);          /* 看门狗输出   */
sbit equControlEn = P1 ^ (unsigned int)(0x02) ; /* 均衡控制输出使能控制  */

struct _HEAT_PARA
{
  unsigned char Mode;  /* 温控模式(开环OR闭环)  */
  unsigned char Close; /* 温控关闭门限  */
  unsigned char Open;  /* 温控开启门限  */
};

struct _HEAT_PARA xdata heatPara[2];         /* 2路温控 A区 */
struct _HEAT_PARA xdata heatParaB[2];        /* 2路温控 B区 */
struct _HEAT_PARA xdata heatParaC[2];        /* 2路温控 C区 */

unsigned char xdata heatParaTH[7]; 				/* 控温7个热敏电阻     */
unsigned char clockCount;                    	/* 1s程序运行定时计数器	           */
unsigned char xdata currentCount;  				/* 电流30s持续时间计数  */
unsigned char       errorCount;              	/* 总线通讯不符合协议计数器           */
unsigned char xdata errorCountA;                /* 总线通讯不符合协议计数器A  */

unsigned char xdata resetCount;  /* 热复位计数 A */
unsigned char xdata resetCountB; /* 热复位计数 B */
unsigned char xdata resetCountC; /* 热复位计数 C */

unsigned char xdata rsCount;          /* 遥控指令执行计数(包括指令、上注数据) */
unsigned char xdata downHeatNumber;   /* 下传控温状态回路路序   */
unsigned char xdata downHealthNumber; /* 下传健康状态回路路序  */
unsigned char xdata fgSwitch;         /* 数据采集保存切区标志  */
unsigned char xdata fgDataRequest;    /* 待处理数据块要求标志  */

unsigned char xdata onoffNumber;  /* 执行指令号A     */
unsigned char xdata onoffNumberB; /* 执行指令号B     */
unsigned char xdata onoffNumberC; /* 执行指令号C     */

unsigned char xdata fgONOFFExecute;  /* 指令执行标志A   */
unsigned char xdata fgONOFFExecuteB; /* 指令执行标志B   */
unsigned char xdata fgONOFFExecuteC; /* 指令执行标志C   */

unsigned char xdata fgDownRs;     /* 热敏电阻下传切换*/
unsigned char xdata workState;    /* 下位机工作状态  */
unsigned char xdata heatState;    /* 下位机加热状态  */
unsigned char xdata warnState;    /* 状态字          */
unsigned char xdata warnCount[9]; /* 监视判断计数器  */

unsigned char xdata voltageWarnValue;  /* 蓄电池组电压预警值A */
unsigned char xdata voltageWarnValueB; /* 蓄电池组电压预警值B */
unsigned char xdata voltageWarnValueC; /* 蓄电池组电压预警值C */

unsigned char xdata chargeEndValue;  /* 蓄电池组电压充电终止电流A */
unsigned char xdata chargeEndValueB; /* 蓄电池组电压充电终止电流B */
unsigned char xdata chargeEndValueC; /* 蓄电池组电压充电终止电流C */

unsigned char xdata healthState;   /* 健康状态标志 */
unsigned char xdata healthData[5]; /* 健康状态数据 */

unsigned char xdata fgAhEn;  /* 蓄电池组安时计使能标志A */
unsigned char xdata fgAhEnB; /* 蓄电池组安时计使能标志B */
unsigned char xdata fgAhEnC; /* 蓄电池组安时计使能标志C */

unsigned char xdata fgChargeEn;  /* 蓄电池组过充保护使能标志A */
unsigned char xdata fgChargeEnB; /* 蓄电池组过充保护使能标志B */
unsigned char xdata fgChargeEnC; /* 蓄电池组过充保护使能标志C */

unsigned char xdata fgDischargeEn;  /* 蓄电池组过放保护使能标志A */
unsigned char xdata fgDischargeEnB; /* 蓄电池组过放保护使能标志B */
unsigned char xdata fgDischargeEnC; /* 蓄电池组过放保护使能标志C */

unsigned char xdata fgExceDischarge; /* 过放保护指令发送标志 */

unsigned char xdata fgAh;  /* 安时计标志A  */
unsigned char xdata fgAhB; /* 安时计标志B  */
unsigned char xdata fgAhC; /* 安时计标志C  */

unsigned char xdata fgTinyCurrent;  /* 涓流标志A */
unsigned char xdata fgTinyCurrentB; /* 涓流标志B */
unsigned char xdata fgTinyCurrentC; /* 涓流标志C */

unsigned char xdata proportion;  /* 蓄电池组充放比A */
unsigned char xdata proportionB; /* 蓄电池组充放比B */
unsigned char xdata proportionC; /* 蓄电池组充放比C */

unsigned char xdata fgChargeOnoffA; /* 大电流充电结束指令标志A */
unsigned char xdata fgChargeOnoffB; /* 大电流充电结束指令标志B */

float xdata currentPower; /* 当前电量值 */

float xdata chargePower;  /* 充电电量值A  */
float xdata chargePowerB; /* 充电电量值B  */
float xdata chargePowerC; /* 充电电量值C  */

float xdata dischargePower;  /* 放电电量值A */
float xdata dischargePowerB; /* 放电电量值B */
float xdata dischargePowerC; /* 放电电量值C */

unsigned char xdata powerSave[6];         /* 电量数据保存数组    */
unsigned char xdata anData8[57];          /* 原始模拟量数据采集区 8bit */
unsigned int xdata anData12[6];           /* 原始模拟量数据采集区 12bit */
unsigned char xdata buildFrameBuffer[80]; /* 缓变待组帧数据缓冲区(Length\Title\有效数据\Sum)  */
unsigned char xdata soonFrameA[49];       /* 速变遥测参数数组A  */
unsigned char xdata soonFrameB[49];       /* 速变遥测参数数组B  */
unsigned char xdata slowFrameA[104];      /* 缓变遥测参数数组A  */
unsigned char xdata slowFrameB[104];      /* 缓变遥测参数数组B  */

unsigned char xdata equEn;  /* 均衡控制允许使能A */
unsigned char xdata equEnB; /* 均衡控制允许使能B */
unsigned char xdata equEnC; /* 均衡控制允许使能C */

unsigned char xdata workEnState;        /* 工作使能状态          */
unsigned char xdata equOutputState;     /* 均衡控制输出状态字    */
unsigned char xdata equOutputWord;      /* 均衡控制输出控制字    */
unsigned char xdata equControlState[7]; /* 均衡控制输出状态      */
unsigned int xdata aq[7];               /* 单体电压值            */
unsigned int xdata aqpx[7];             /* 排序后单体电压值      */

unsigned char xdata canResetCount;     /* CAN热复位计数    */
unsigned int xdata wingTemperature[6]; /* 温度采集值       */

unsigned char fgSRAMchk;      /* SRAM自检标志          */
unsigned char chkAddr;        /* SRAM后4K自检位置标记，每秒自检256字节 */

unsigned int  AD558set;                      /* DA设置输出值A         */
unsigned int  AD558setB;                     /* DA设置输出值B         */
unsigned int  AD558setC;                     /* DA设置输出值C         */

unsigned char xdata fgAnCheck;        /* 模拟量自检标志        */
unsigned int xdata anCount[6];        /* 模拟量自检计数        */
unsigned char xdata anCheckEn;        /* 模拟量自检使能标志    */
unsigned char xdata anCheckEnB;       /* 模拟量自检使能标志B   */
unsigned char xdata anCheckEnC;       /* 模拟量自检使能标志C   */
unsigned int xdata chargeUnlockCount; /* 充电解锁计数          */
unsigned char xdata fg2S;             /* 2s定时运行标志        */
unsigned char xdata fg60s;            /* 60S时间周期定时标志   */
unsigned char xdata fgPreWarn;        /* 预警标志位   */
unsigned char xdata uploadData[8];    /* 上注数据块数据(不包括0x55标识字)  */

unsigned char xdata *pSend;             /* 总线发送指针   */
unsigned char sendIndex;                /* 总线发送帧索引   */
unsigned char sendFrame;                /* 总线发送数据包总帧数  */
unsigned char receiveFrame;             /* 总线接收数据帧计数  */
unsigned int receiveSum;                /* 总线接收数据包累加和  */
unsigned char receiveCount;             /* 总线接收数据计数  */
unsigned char xdata receiveBuffer[256]; /* 总线数据包接收缓存区(wLength+Title+Data+Sum)  */

void SystemInit(void);                           /* 系统初始化函数       */
void DataInit(void);                             /* 数据初始化函数 */
void CANInit(unsigned int address);              /* SJA1000初始化函数 */
void CPUInit(void);                              /* 80C32初始化函数 */
void PowerInit(void);                            /* 电量参数初始化函数 */
void ANCollect(void);                            /* 模拟量采集函数 */
unsigned char ADConvert8(unsigned char channel); /* AD 8bit转换函数 */
unsigned int ADConvert12(unsigned char channel); /* AD 12bit转换函数 */
void StateWordBuild(void);                       /* 组状态字函数 */
void DataSave(void);                             /* 遥测包组帧函数 */
void HeatControl(void);                          /* 温度控制赫纳数 */
void CanResume(void);                            /* CAN恢复函数 */
void PowerControl(void);                         /* 安时计函数 */
void ChargeProtect(void);                        /* 过充保护函数 */
void DischargeProtect(void);                     /* 过放保护函数 */
void ONOFFOutput(unsigned char index);           /* 指令输出函数 */
void SafeWarn(void);                             /* 安全预警函数 */
void ONOFFHook(void);                            /* 指令输出链接函数 */
void DataLoad(void);                             /* 数据块处理函数 */

void FrameBuild(unsigned char *pBuildFrame, unsigned char frameIndex, unsigned char endDLC); /* 组帧函数 */
unsigned char Get2_3_X(unsigned char *a, unsigned char *b, unsigned char *c);                /* 数据3取2判决函数 */
unsigned char Get2_3_F(float *a, float *b, float *c);                                        /* 浮点数3取2判决函数 */
unsigned char Get2_3_I(unsigned int *a, unsigned int *b, unsigned int *c);                   /* 整形数3取2判决函数 */
void Delay(unsigned int time);                                                               /* 固定延时函数 */
void DataProtect(void);                                                                      /* 数据参数保护函数 */
void SRAMcheck(void);                                                                        /* SRAM周期性自检 */
void DAcheck(void);                                                                          /* DA自检函数 */
void HeatCheck(void);                                                                        /* 控温自检函数 */
void Ancheck(void);                                                                          /* 模拟量自检函数 */
void EquControlCut(void);                                                                    /* 均衡控制断开函数    */
void EquControlLink(void);                                                                   /* 均衡控制接通函数    */
void ChargeUnlock(void);                                                                     /* 充电解锁功能 */

void CANRXD(void);      /* CAN中断接收处理函数 */
void CANTXD(void);      /* CAN中断发送处理函数 */
void RsManage(void);    /* CAN中断轮询处理函数 */
void ONOFFManage(void); /* CAN中断指令处理函数 */
void DataManage(void);  /* CAN中断数据块处理函数 */


/*******************************************************
  函数名称:  main
  功能描述:  
  修改记录:  
********************************************************/
void main(void)
{
  unsigned char clockCountTemp; /* 满足规则检查增加临时变量 */

  SystemInit(); /* 系统初始化 */

  while (1) /* 执行固定周期任务  1s */
  {
      dog = !dog;                      /* 1s 牵狗 */
      AD558 = (unsigned char)AD558set; /* 周期性设置DA输出 */

      if ((fgSRAMchk == TRUE) || (fgAnCheck == TRUE)) /* SRAM自检异常 */
      {
        XBYTE[BUS_ADDR] = 0x41; /* SJA1000进入复位模式  */
      }
      else /* SRAM自检正常时执行固定任务 */
      {
        errorCount++;
        fg60s++;            /* 60S周期定时 */
                            /* 1s固定执行函数 */
        DataProtect();      /* 数据保护函数 */
        ANCollect();        /* 模拟量采集 */
        StateWordBuild();   /* 构成状态字 */
        DataSave();         /* 组帧函数 */
        HeatControl();      /* 控温函数 */
        CanResume();        /* CAN总线异常处理函数 */
        ChargeProtect();    /* 过充保护 */
        DischargeProtect(); /* 过放保护 */
        EquControlCut();    /* 均衡控制断开功能 */

        if (fg2S == FALSE) /* 2s逻辑翻转 */
        {
          fg2S = TRUE;
          ChargeUnlock(); /* 充电解锁功能 */
          Ancheck();      /* 模拟量自检函数 */
        }
        else /* 2s固定执行函数 */
        {
          fg2S = FALSE;
          DAcheck();      /* DA自检函数 */
          PowerControl(); /* 安时计控制 */
          SafeWarn();     /* 能源安全模式 */
        }

        if (fg60s >= 60) /* 60S进行均衡控制 */
        {
          EquControlLink(); /* 均衡控制接通函数 */
          fg60s = 0;
        }

        ONOFFHook(); /* 间接指令处理钩子 */
        DataLoad();
        SRAMcheck(); /* 完成固定任务后进行SRAM周期性自检 */
      }

      dog = !dog; /* 1s 牵狗 */

      clockCountTemp = clockCount;
      while (clockCountTemp < 100) /* 延时等待1s */
      {
        clockCountTemp = clockCount;
      }
      clockCount = (unsigned char)(clockCount - 100); /* 时间补偿 */
    }
}

/*******************************************************
  函数名称:  SystemInit
  功能描述:  
  修改记录:  
********************************************************/
void SystemInit(void)
{
  dog = !dog;

  EA = 0; /* 关中断 */

  ONOFF_ADDR = 0xFF; /* 输出指令码FFH */
  onoffEn = 0;       /* 指令输出使能 */
  onoffEn = 1;       /* 指令输出禁能 */

  equControlEn = 0; /* 控制输出设置为高电平  全部断开 */
  EQU_ADDR = 0;

  AD558 = AD558_VSET; /* 设置DA输出为典型值 */

  AD558set = AD558_VSET;
  AD558setB = AD558_VSET;
  AD558setC = AD558_VSET;

  DataInit();  /* 默认数据初始化 */
  HeatCheck(); /* 温控自检函数 */
  PowerInit(); /* 默认电量参数初始化 */

  ANCollect();      /* 模拟量采集 */
  StateWordBuild(); /* 构成状态字 */
  DataSave();       /* 组帧函数 数据填充AB区 */
  DataSave();       /* 组帧函数 */

  CPUInit();          /* 80C32初始化 */
  CANInit(BUS_ADDR); /* CAN总线初始化 */

  EA = 1; /* 开中断  */
}

/*******************************************************
  函数名称:  DataInit
  功能描述:  
  修改记录:  
********************************************************/
void DataInit(void)
{
  unsigned int i;

  fgRstType = 1;      /* 设置P1口输入状态 */
  if (fgRstType == 1) /* 判断是冷复位或热复位 */
  {
    resetCount++;                                    /* 热启动计数自加 */
    resetCount = (unsigned char)(resetCount & 0x0F); /* 计数清零操作 */
    resetCountB = resetCount;                        /* B,C赋值 */
    resetCountC = resetCount;
  }
  else
  {
    resetCount = 0;  /* 冷启动复位计数清零  */
    resetCountB = 0; /* B,C赋值 */
    resetCountC = 0;
  }

  for (i = 0; i < 2; i++) /* 温控回路参数初始化   */
  {
    heatPara[i].Mode = heatTab[i][0]; /* A区默认参数 */
    heatPara[i].Close = heatTab[i][1];
    heatPara[i].Open = heatTab[i][2];

    heatParaB[i].Mode = heatTab[i][0]; /* B区默认参数 */
    heatParaB[i].Close = heatTab[i][1];
    heatParaB[i].Open = heatTab[i][2];

    heatParaC[i].Mode = heatTab[i][0]; /* C区默认参数 */
    heatParaC[i].Close = heatTab[i][1];
    heatParaC[i].Open = heatTab[i][2];
  }

  voltageWarnValue = ACCUMULATOR_VSET;  /* 电压预警值 A值 */
  voltageWarnValueB = ACCUMULATOR_VSET; /* 电压预警值 B值 */
  voltageWarnValueC = ACCUMULATOR_VSET; /* 电压预警值 C值 */

  chargeEndValue = 150;  /* 蓄电池组电压充电终止电流   X=(W*2)/100  默认值3 */
  chargeEndValueB = 150; /* 蓄电池组电压充电终止电流 B */
  chargeEndValueC = 150; /* 蓄电池组电压充电终止电流 C */

  chargeUnlockCount = 0; /* 充电解锁计数 */

  clockCount = 0;  /* 1s程序运行定时计数器 */
  errorCount = 0;  /* 总线通讯不符合协议计数器  */
  errorCountA = 0; /* 总线通讯不符合协议计数器  */

  onoffNumber = 0; /* 执行指令号 */
  onoffNumberB = 0;
  onoffNumberC = 0;

  rsCount = 0;          /* 遥控指令执行计数(包括指令、上注数据) */
  currentCount = 0;     /* 电流30s计数器 */
  downHeatNumber = 0;   /* 下传控温状态回路路序 */
  downHealthNumber = 0; /* 下传健康状态回路路序 */

  fg60s = 0;         /* 60S周期定时 */
  fg2S = FALSE;      /* 2S周期定时 */
  fgSwitch = FALSE;  /* 数据采集保存切区标志 */
  fgDownRs = FALSE;  /* 默认禁止切换 */
  fgSRAMchk = FALSE; /* 默认SRAM自检正常 */

  equEn = FALSE; /* 均衡控制初始状态为禁止 */
  equEnB = FALSE;
  equEnC = FALSE;

  fgDischargeEn = FALSE; /* 默认过放保护功能禁止 */
  fgDischargeEnB = FALSE;
  fgDischargeEnC = FALSE;

  fgChargeEn = TRUE; /* 默认过充保护允许 */
  fgChargeEnB = TRUE;
  fgChargeEnC = TRUE;

  fgChargeOnoffA = FALSE; /* 默认大电流充电结束指令标志 */
  fgChargeOnoffB = FALSE;

  fgDataRequest = FALSE; /* 待处理数据块要求标志 */

  fgONOFFExecute = FALSE; /* 指令执行标志 */
  fgONOFFExecuteB = FALSE;
  fgONOFFExecuteC = FALSE;

  fgExceDischarge = FALSE; /* 默认无过放状态 */
  
  fgPreWarn = FALSE; /* 预警标志位 无效  */

  workState = 0;   /* 工作状态字初始化清零       初始值待构成状态字 */
  workEnState = 0; /* 工作使能状态 D0=0 D1=0 D2=0 D3=1 D4=0   初始值待构成状态字 */

  healthState = 0; /* 报警状态字    */

  warnState = 0;      /* 预警状态字清零 */
  heatState = 0;      /* 健康状态字清零 */
  equOutputState = 0; /* 均衡控制输出状态字  7路断开(默认)  下传用  初始值待构成状态字 */

  equOutputWord = 0; /* 均衡控制输出控制字  7路断开(默认)  实际控制用 */
  chkAddr = 0;       /* SRAM后4K自检位置初始化为0，即从0x1000位置起 */

  healthData[0] = 0xFF; /* 健康状态数据1 默认0xFF */
  for (i = 1; i < 5; i++)
  {
    healthData[i] = 0; /* 健康状态数据2-5 默认0 */
  }

  for (i = 0; i < 9; i++)
  {
    warnCount[i] = 0; /* 监视判断计数器 */
  }

  for (i = 0; i < 7; i++) /* 均衡控制输出状态初始化，默认断开   FALSE 断开  控制用1通 0断  下传用：1断 0通 */
  {
    equControlState[i] = FALSE;
  }

  fgAnCheck = FALSE; /* 模拟量自检标志默认为正常 */

  anCheckEn = FALSE; /* 默认禁止模拟量自检功能  */
  anCheckEnB = FALSE;
  anCheckEnC = FALSE;

  for (i = 0; i < 6; i++) /* 模拟量自检计数 */
  {
    anCount[i] = 0; /* 模拟量计数清零 */
  }

  receiveSum = 0; /* 总线接收数据包参数清零   */
  receiveCount = 0;
  receiveFrame = 0;
   for (i = 0; i < 256; i++)
   {
     receiveBuffer[i] = 0;
   }

   for (i = 0; i < 8; i++) /* 上注数据块缓冲区清零    */
   {
     uploadData[i] = 0;
   }

  canResetCount = 0; /* CAN复位计数清零 */

   for (i = 0; i < 6; i++) /* 采集值清零 */
   {
     wingTemperature[i] = 0;
   }
}

/*******************************************************
* 函数介绍:  SRAM自检函数
* 备注: 
* 
*******************************************************/
void SRAMcheck(void)
{
  unsigned int i;
  unsigned char dataBuffer; /* SRAM自检暂存数据缓冲区 */

  if ((fgAh == TRUE) || (fgAh == FALSE)) /* 安时计标志为典型值 */
  {
    if ((fgTinyCurrent == TRUE) || (fgTinyCurrent == FALSE)) /* 涓流标志为典型值 */
    {
      fgSRAMchk = FALSE; /* 典型值认为SRAM自检正确 */
    }
    else /* 非典型值为SRAM异常 */
    {
      fgSRAMchk = TRUE;
    }
  }
  else /* 非典型值为SRAM异常 */
  {
    fgSRAMchk = TRUE;
  }

  if (fgSRAMchk == FALSE) /* SRAM自检正确时执行SRAM读写检测 */
  {
    if (chkAddr >= 16) /* 检测范围 16 * 256 BYTE  4K  读写测试前判读地址有效范围 */
    {
      chkAddr = 0; /* 循环清零	 */
    }

    for (i = 0; i < 256; i++) /* 256BYTE检测 */
    {
      dataBuffer = XBYTE[0x1000 + (unsigned int)(256 * chkAddr) + i]; /* 当前数据保存 */
      XBYTE[0x1000 + (unsigned int)(256 * chkAddr) + i] = 0xEB;       /* 写入0xEB测试写入值 */
      if (XBYTE[0x1000 + (unsigned int)(256 * chkAddr) + i] != 0xEB)  /* 读出比较 */
      {
        fgSRAMchk = TRUE; /* 数据不等时SRAM异常 */
        break;
      }
      else /* 数据相等恢复保存数据 */
      {
        XBYTE[0x1000 + (unsigned int)(256 * chkAddr) + i] = dataBuffer;
      }
    }
    chkAddr = (unsigned char)(chkAddr + 1); /* 检测地址循环自加 */
  }
}

/*******************************************************
* 函数介绍:  D/A输出循检函数
* 备注: 
* 
*******************************************************/
void DAcheck(void)
{
  int result;

  result = abs(((int)anData8[15] * 2) - (int)AD558set);
  if (result < 11) /* D/A采集值与输出值在10个分层值内 */
  {
    workState = (unsigned char)(workState & 0xFE); /* 设置自检成功标志	 */
  }
  else
  {
    workState = (unsigned char)(workState | 0x01); /* 设置自检失败标志 */
  }
}

/*******************************************************
  函数名称:  HeatCheck
  功能描述:  
  修改记录:  
********************************************************/
void HeatCheck(void)
{
  P1 = (unsigned char)(P1&0xFC);  /* 开启温控输出	 2路    */
  Delay(2000);                    /* 延时等待约30ms  */
  ANCollect();                    /* 控温输出状态采集 */
  StateWordBuild();               /* 控温输出状态字 */
  if ((heatState & 0x03) != 0x00) /* 状态字判断  （全部加热） */
  {
    workState = (unsigned char)(workState | 0x40); /* 设置自检失败标志 */
  }

  P1 = (unsigned char)(P1|0x03); /* 关闭温控输出	 2路 */
  Delay(2000);                     /* 延时等待约30ms  */
  ANCollect();                     /* 控温输出状态采集 */
  StateWordBuild();                /* 控温输出状态字 */
  if ((heatState & 0x03) != 0x03)  /* 状态字判断  （全部关闭） */
  {
    workState = (unsigned char)(workState | 0x40); /* 设置自检失败标志 */
  }
}

/*******************************************************
* 函数介绍: 模拟量自检函数
* 备注:
* 
*******************************************************/
void Ancheck(void)
{
  unsigned char i;
  unsigned char xdata fgChk[6];

  if (anCheckEn == TRUE)
  {
    for (i = 0; i < 6; i++) /* 6个独立判读标识      */
    {
      fgChk[i] = FALSE;
    }

    if (anData8[11] < 0x96) /* 12V主份电源电压  96 - 3V */
    {
      anCount[0]++;

      if (anCount[0] >= 15) /* 持续时间大于30s */
      {
        fgChk[0] = TRUE;
      }
    }
    else /* 条件不满足 */
    {
      anCount[0] = 0;
      fgChk[0] = FALSE;
    }

    if (anData8[11] > 0xFA) /* 12V主份电源电压  FA - 5V */
    {
      anCount[1]++;
      if (anCount[1] >= 15) /* 持续时间大于30s */
      {
        fgChk[1] = TRUE;
      }
    }
    else /* 条件不满足 */
    {
      anCount[1] = 0;
      fgChk[1] = FALSE;
    }

    if (anData8[12] < 0x96) /* 12V备份电源电压 96 - 3V */
    {
      anCount[2]++;
      if (anCount[2] >= 15) /* 持续时间大于30s */
      {
        fgChk[2] = TRUE;
      }
    }
    else /* 条件不满足 */
    {
      anCount[2] = 0;
      fgChk[2] = FALSE;
    }

    if (anData8[12] > 0xFA) /* 12V备份电源电压 FA - 5V */
    {
      anCount[3]++;
      if (anCount[3] >= 15) /* 持续时间大于30s */
      {
        fgChk[3] = TRUE;
      }
    }
    else /* 条件不满足 */
    {
      anCount[3] = 0;
      fgChk[3] = FALSE;
    }

    if (anData8[14] < 0x64) /* 5V基准电源电压 4V - 64  */
    {
      anCount[4]++;
      if (anCount[4] >= 15) /* 持续时间大于30s */
      {
        fgChk[4] = TRUE;
      }
    }
    else /* 条件不满足 */
    {
      anCount[4] = 0;
      fgChk[4] = FALSE;
    }

    if (anData8[14] > 0x96) /* 5V基准电源电压 6V - 96 */
    {
      anCount[5]++;
      if (anCount[5] >= 15) /* 持续时间大于30s */
      {
        fgChk[5] = TRUE;
      }
    }
    else /* 条件不满足 */
    {
      anCount[5] = 0;
      fgChk[5] = FALSE;
    }

    for (i = 0; i < 6; i++) /* 任意条件成立，设置模拟量自检异常标志 */
    {
      if (fgChk[i] == TRUE)
      {
        fgAnCheck = TRUE;
      }
    }
  }
}

/*******************************************************
  函数名称:  CANInit
  功能描述:  
  修改记录:  
********************************************************/
void CANInit(unsigned int address)
{
  unsigned char temp;

  XBYTE[address] = 0x41; /* 进入复位模式    */
  Delay(5);
  XBYTE[address + 1] = 0xEC;    /* 命令寄存器设置 */
  XBYTE[address + 4] = CAN_ACR; /* 接受代码寄存器设置       */
  XBYTE[address + 5] = CAN_AMR; /* 接受屏蔽寄存器设置       */
  XBYTE[address + 6] = 0x41;    /* 总线时序寄存器0设置      */
  XBYTE[address + 7] = 0xC5;    /* 总线时序寄存器1设置      */
  XBYTE[address + 8] = 0x4A;    /* 输出控制寄存器设置       */
  XBYTE[address + 31] = 0x04;   /* 时钟分频寄存器设置       */

  XBYTE[address] = 0x46; /* 中断的开启处理  */


  temp = XBYTE[address + 2]; /* 清状态寄存器  */
  temp = XBYTE[address + 3]; /* 清中断寄存器   */
}

/*******************************************************
  函数名称:  CPUInit
  功能描述:  
  修改记录:  
********************************************************/
void CPUInit(void)
{
  IP = 0x02; /* 设置优先级: T0为高优先级、INT0为低优先级 */
  IT0 = 1;   /* 电平中断触发 */

  TMOD = 0x01; /* 定时器0设定工作方式1 */
  TH0 = 0xDC;
  TL0 = 0x00; /* 定时器10ms定时  */
  TR0 = 1;    /* 启动T0计数 */

  ET0 = 1; /* 允许定时器0中断 */
  EX0 = 1; /* 允许外部中断0中断   */
}

/*******************************************************
  函数名称:  PowerInit
  功能描述:  
  修改记录:  
********************************************************/
void PowerInit(void)
{
  proportion = 130;  /* 默认电量控制初始参数  (130 + 900)/1000=1.03 */
  proportionB = 130; /* 默认充放比1.03  */
  proportionC = 130;

  powerSave[0] = FULLPOWER_H; /* 当前电量150Ah  （下传值） */
  powerSave[1] = FULLPOWER_L;
  powerSave[2] = 0; /* 充电电量0Ah */
  powerSave[3] = 0;
  powerSave[4] = 0; /* 放电电量0Ah */
  powerSave[5] = 0;

  currentPower = (float)FULLPOWER; /* 当前电量参数初始值 （程序计算使用值） */

  chargePower = (float)0; /* 充电电量参数初始值 */
  chargePowerB = (float)0;
  chargePowerC = (float)0;

  dischargePower = (float)0; /* 放电电量参数初始值 */
  dischargePowerB = (float)0;
  dischargePowerC = (float)0;

  fgAhEn = TRUE; /* 默认安时计允许状态  */
  fgAhEnB = TRUE;
  fgAhEnC = TRUE;

  fgAh = FALSE; /* 设置安时计控制标志(表示0) */
  fgAhB = FALSE;
  fgAhC = FALSE;

  fgTinyCurrent = FALSE; /* 设置涓流标志(表示0) */
  fgTinyCurrentB = FALSE;
  fgTinyCurrentC = FALSE;
}


/*******************************************************
  函数名称:  ANCollect
  功能描述:  
  修改记录:  
********************************************************/
void ANCollect(void)
{
  unsigned char i;
  unsigned char j;
  unsigned int xdata temp;
  unsigned long xdata sum; /* 特殊参数算术平均值 */

  for (i = 0; i < 57; i++) /* 采集全部遥测参数(8bit) */
  {
    anData8[i] = ADConvert8(rsTab8[i]);
  }

  for (i = 0; i < 6; i++) /* 温度采集 (12bit) */
  {
    wingTemperature[i] = ADConvert12(rsTab8[44 + i]);
  }

  for (i = 0; i < 6; i++) /* 电流20次采集 (12bit) */
  {
    sum = 0; /* 累加和清零 */
    for (j = 0; j < 20; j++)
    {
      sum = sum + ADConvert12(rsTab12[i]); /* 计算20次累加和 */
    }
    anData12[i] = (unsigned int)((float)sum / 20.0); 
  }

  for (i = 0; i < 7; i++) /
  {
    aq[i] = ADConvert12(rsTab8[29 + i]);
    aq[i] = aq[i] >> 2; /* D11 ~ D2	 */
    aqpx[i] = aq[i];
  }

  for (i = 0; i < 6; i++) /* 冒泡排序 */
  {
    for (j = 1; j < (unsigned char)(7 - i); j++)
    {
      if (aqpx[i] > aqpx[i + j]) /* 升序 */
      {
        temp = aqpx[i + j];
        aqpx[i + j] = aqpx[i];
        aqpx[i] = temp;
      }
    }
  }
}

/*******************************************************
  函数名称:  ADConvert8
  功能描述:  实现模拟量的数据转换采集功能
  修改记录:  
********************************************************/
unsigned char ADConvert8(unsigned char channel)
{
  unsigned char i;
  unsigned char j;
  unsigned char temp;
  unsigned char ad[3]; /* 中间值滤波存储 */

  AN_ADDR = channel; /* 打开模拟量通道 */
  Delay(25);         /* 延时等待通道打开 */
  for (i = 0; i < 3; i++)
  {
    AD574H = 0;     /* A/D转换启动 */
    Delay(3);       /* A/D转换延时 */
    ad[i] = AD574H; /* 取Bit11-Bit4  20mv */
  }

  for (i = 0; i < 2; i++) /* 冒泡法排序  */
  {
    for (j = 1; j < (unsigned char)(3 - i); j++)
    {
      if (ad[i] > ad[i + j])
      {
        temp = ad[i + j];
        ad[i + j] = ad[i];
        ad[i] = temp;
      }
    }
  }

  return (ad[1]); /* 返回中间值 */
}

/*******************************************************
  函数名称:  ADConvert12
  功能描述:  
  修改记录:  
********************************************************/
unsigned int ADConvert12(unsigned char channel)
{
  unsigned char i;
  unsigned char j;
  unsigned int temp;
  unsigned int xdata ad12[3]; /* 中间值滤波存储 */
  unsigned char HData;
  unsigned char LData;

  AN_ADDR = channel; /* 打开模拟量通道 */
  Delay(25);         /* 延时等待通道打开 */
  for (i = 0; i < 3; i++)
  {
    AD574H = 0; /* A/D转换启动 */
    Delay(3);   /* A/D转换延时 */
    HData = AD574H;
    LData = AD574L;

    ad12[i] = (unsigned int)((unsigned int)((unsigned int)((unsigned int)HData << 4) & 0x0FF0) | (unsigned int)((unsigned int)((unsigned int)LData >> 4) & 0x000F));
  }

  for (i = 0; i < 2; i++) /* 冒泡法排序  */
  {
    for (j = 1; j < (unsigned char)(3 - i); j++)
    {
      if (ad12[i] > ad12[i + j])
      {
        temp = ad12[i + j];
        ad12[i + j] = ad12[i];
        ad12[i] = temp;
      }
    }
  }

  return (ad12[1]); /* 返回中间值 */
}

/*******************************************************
  函数名称:  StateWordBuild
  功能描述: 
  修改记录:  
********************************************************/
void StateWordBuild(void)
{
  unsigned char i;
  unsigned char xdata *p;

  if (CBYTE[0x3FFF] == TRUE) /* 处理器使用状态 */
  {
    workState = (unsigned char)(workState & 0xFD); /* CPU使用状态: A机 */
  }
  else
  {
    workState = (unsigned char)(workState | 0x02); /* CPU使用状态: B机 */
  }

  if (fgAh == TRUE) /* 判断安时计标志(D4) */
  {
    workState = (unsigned char)(workState | 0x10);
  }
  else
  {
    workState = (unsigned char)(workState & 0xEF);
  }

  if (fgTinyCurrent == TRUE) /* 判断涓流标志(D5) */
  {
    workState = (unsigned char)(workState | 0x20);
  }
  else
  {
    workState = (unsigned char)(workState & 0xDF);
  }
  /* D6为控温回路自检位 */
  if (fgPreWarn == TRUE) /* 预警标志(D7) */
  {
    workState = (unsigned char)(workState | 0x80);
  }
  else
  {
    workState = (unsigned char)(workState & 0x7F);
  }

  heatState = 0;          /* 加热状态清零 */
  p = &anData8[51];       /* 电源下位机加热状态  */
  for (i = 0; i < 6; i++) /* 指向加热通道状态采集参数 */
  {
    if (*p > 125)
    {
      heatState = (unsigned char)(heatState | 0x40);
    }
    heatState = (unsigned char)(heatState >> 1);
    p++;
  }

  equOutputState = 0; /* 清零操作 */
  for (i = 0; i < 7; i++)
  {
    if (equControlState[i] == FALSE) /* 若为断开状态 */
    {
      equOutputState = (unsigned char)(equOutputState | 0x80); /* 设置D7 */
    }
    equOutputState = (unsigned char)(equOutputState >> 1); /* 循环右移 */
  }

  if (fgAhEn == FALSE) /* D0 安时计状态 */
  {
    workEnState = (unsigned char)(workEnState | 0x01); /* 安时计禁止 */
  }
  else
  {
    workEnState = (unsigned char)(workEnState & 0xFE); /* 安时计允许 */
  }

  if (equEn == TRUE) /* D1 均衡状态 */
  {
    workEnState = (unsigned char)(workEnState | 0x02); /* 均衡控制允许  */
  }
  else
  {
    workEnState = (unsigned char)(workEnState & 0xFD); /* 均衡控制禁止 */
  }
  if (fgDischargeEn == FALSE) /* D3 过放保护状态  */
  {
    workEnState = (unsigned char)(workEnState & 0xF7); /* 过放保护不允许 */
  }
  else
  {
    workEnState = (unsigned char)(workEnState | 0x08);
  }
  /* D4 过放标志 (过放保护模块中设置，无清除操作) */
  if (fgChargeEn == FALSE) /* D5 过充保护状态  */
  {
    workEnState = (unsigned char)(workEnState | 0x20); /* 过充保护禁止 */
  }
  else
  {
    workEnState = (unsigned char)(workEnState & 0xDF); /* 过充保护允许 */
  }

  if (anCheckEn == FALSE) /* D6 模拟量自检使能标志 */
  {
    workEnState = (unsigned char)(workEnState & 0xBF); /* 0 禁止 */
  }
  else
  {
    workEnState = (unsigned char)(workEnState | 0x40); /* 1 允许 */
  }
}

/*******************************************************
  函数名称:  DataSave
  功能描述:  
  修改记录:  
********************************************************/
void DataSave(void)
{
  unsigned char i;
  unsigned char xdata *pBuffer; /* 数据存储区指针定义 */
  unsigned char xdata *pSoon;
  unsigned char xdata *pSlow;

  unsigned char xdata rsSoonSum; /* 速变遥测参数SUM */
  unsigned char xdata rsSlowSum; /* 缓变遥测参数SUM */

  rsSoonSum = 0x23; /* 速变遥测参数Title */
  rsSlowSum = 0x43; /* 缓变遥测参数Title */

  if (fgSwitch == FALSE) /* 判断切换标志进行切区 */
  {
    pSoon = &soonFrameA[0]; /* A区赋值 */
    pSlow = &slowFrameA[0];
  }
  else /* B区赋值 */
  {
    pSoon = &soonFrameB[0];
    pSlow = &slowFrameB[0];
  }

  pBuffer = &buildFrameBuffer[0]; /* 速变参数构成 */
  *pBuffer = 0x1F;                /* 存储Length */
  pBuffer++;
  *pBuffer = 0x23; /* 存储Title  */
  pBuffer++;

  *pBuffer = anData8[0]; /* W0  */
  pBuffer++;
  *pBuffer = anData8[1]; /* W1  */
  pBuffer++;

  for (i = 0; i < 6; i++) /* W2-W13 */
  {
    *pBuffer = (unsigned char)(anData12[i] >> 10); 
    pBuffer++;
    *pBuffer = (unsigned char)(anData12[i] >> 2);
    pBuffer++;
  }

  for (i = 2; i < 5; i++) /* W14-W16 */
  {
    *pBuffer = anData8[i];
    pBuffer++;
  }

  for (i = 0; i < 7; i++) /* W17-W30 */
  {
    *pBuffer = (unsigned char)(aq[i] >> 8); 
    pBuffer++;
    *pBuffer = (unsigned char)aq[i];
    pBuffer++;
  }

  for (i = 2; i < 33; i++) /* 计算速变累加和 */
  {
    rsSoonSum = (unsigned char)(rsSoonSum + buildFrameBuffer[i]);
  }
  *pBuffer = rsSoonSum; /* 速变累加和付值 */

  FrameBuild(pSoon, 5, 7); /* 构成完整速变遥测参数(5帧) */

  if (fgDownRs == TRUE) /* 下传健康状态数据切换 */
  {
    downHeatNumber++; /* 轮流下传温控参数  */
    downHealthNumber++;

    if (downHeatNumber > 2) /* 控温参数循环切换 0,1 控温参数 2 蓄电池组充电终止电流 */
    {
      downHeatNumber = 0; /* 0-1 有效范围 */
    }
    if (downHealthNumber > 4) /* 健康状态数据切换 */
    {
      downHealthNumber = 0; /* 0-4 有效范围 */
    }
    fgDownRs = FALSE; /* 清除热控下传标志    */
  }

  pBuffer = buildFrameBuffer; /* 构成缓变遥测帧 */
  *pBuffer = 0x44;            /* 存储Length */
  pBuffer++;
  *pBuffer = 0x43; /* 存储Title  */
  pBuffer++;

  *pBuffer = healthState; /* W0 下位机健康标志 */
  pBuffer++;
  *pBuffer = healthData[downHealthNumber]; /* W1 下位机健康状态数据 */
  pBuffer++;
  *pBuffer = warnState; /* W2 下位机危险标志 */
  pBuffer++;

  for (i = 5; i < 29; i++) /* W3-W26 缓变参数赋值 */
  {
    *pBuffer = anData8[i];
    pBuffer++;
  }

  for (i = 3; i < 6; i++) /* W27-W32 温度1~3 */
  {
    *pBuffer = (unsigned char)(wingTemperature[i] >> 10); 
    pBuffer++;
    *pBuffer = (unsigned char)(wingTemperature[i] >> 2);
    pBuffer++;
  }

  *pBuffer = (unsigned char)canResetCount; /* W33 总线复位计数  */
  pBuffer++;

  for (i = 36; i < 44; i++) /* W34-W41 缓变参数赋值 */
  {
    *pBuffer = anData8[i];
    pBuffer++;
  }

  for (i = 0; i < 3; i++) /* W42-W47 太阳翼温度4~6 */
  {
    *pBuffer = (unsigned char)(wingTemperature[i] >> 10); 
    pBuffer++;
    *pBuffer = (unsigned char)(wingTemperature[i] >> 2);
    pBuffer++;
  }

  for (i = 0; i < 6; i++) /* W48-W53 蓄电池组当前电量 */
  {
    *pBuffer = powerSave[i];
    pBuffer++;
  }

  *pBuffer = anData8[50]; /* W55 充放比 */
   pBuffer++;
  *pBuffer = proportion; /* W55 充电保护状态 */
  pBuffer++;

  if (downHeatNumber < 2)
  {
    *pBuffer = (unsigned char)(downHeatNumber + 1); /* W56 控温路数 */
    pBuffer++;
    *pBuffer = heatPara[downHeatNumber].Mode; /* W57 控温模式 */
    pBuffer++;
    *pBuffer = heatPara[downHeatNumber].Close; /* W58 关闭门限 */
    pBuffer++;
    *pBuffer = heatPara[downHeatNumber].Open; /* W59 开启门限 */
    pBuffer++;
  }
  else
  {
    *pBuffer = chargeEndValue; /* W56 蓄电池组充电终止电流 */
    pBuffer++;
    *pBuffer = 0x00; /* W57 00H */
    pBuffer++;
    *pBuffer = 0x00; /* W58 00H */
    pBuffer++;
    *pBuffer = 0x00; /* W59 00H */
    pBuffer++;
  }

  *pBuffer = XBYTE[BUS_ADDR + 2]; /* W60 总线控制器状态字 */
  pBuffer++;
  *pBuffer = 0xAA; /* W61 预留 */
  pBuffer++;
  *pBuffer = (unsigned char)((unsigned char)(rsCount << 4) + resetCount); /* W62 下位机计数器 */
  pBuffer++;
  *pBuffer = workState; /* W63 下位机状态 */
  pBuffer++;
  *pBuffer = heatState; /* W64 加热状态 */
  pBuffer++;
  *pBuffer = equOutputState; /* W65 均衡状态 */
  pBuffer++;
  *pBuffer = workEnState; /* W66 软件使能状态 */
  pBuffer++;
  *pBuffer = voltageWarnValue; /* W67 蓄电池组电压报警门限 */
  pBuffer++;

  for (i = 2; i < 70; i++) /* 计算缓变参数累加和 */
  {
    rsSlowSum = (unsigned char)(rsSlowSum + buildFrameBuffer[i]);
  }
  *pBuffer = rsSlowSum;

  FrameBuild(pSlow, 11, 2); /* 缓变组帧（11帧） */

  if (fgSwitch == FALSE) /* 切区标志转换 */
  {
    fgSwitch = TRUE;
  }
  else
  {
    fgSwitch = FALSE;
  }
}

/*******************************************************
  函数名称:  FrameBuild
  功能描述:  
  修改记录:  
********************************************************/
void FrameBuild(unsigned char *pBuildFrame, unsigned char frameIndex, unsigned char endDLC)
{
  unsigned char i;
  unsigned char j;
  unsigned char xdata *p;

  p = &buildFrameBuffer[0];

  for (i = 0; i < (unsigned char)(frameIndex - 1); i++) /* 起始帧,中间帧构成 */
  {
    *pBuildFrame = 0x86; /* 首字节 */
    pBuildFrame++;
    *pBuildFrame = 0x68; /* 次字节 */
    pBuildFrame++;
    *pBuildFrame = i; /* 帧计数 */
    pBuildFrame++;
    for (j = 0; j < 7; j++) /* 有效数据 */
    {
      *pBuildFrame = *p;
      pBuildFrame++;
      p++;
    }
  }

  *pBuildFrame = 0x86; /* 构成结束帧 */
  pBuildFrame++;
  *pBuildFrame = (unsigned char)(0x60 + endDLC); /* 结束帧数据场长度 */
  pBuildFrame++;
  *pBuildFrame = (unsigned char)(frameIndex - 1); /* 帧技术 */
  pBuildFrame++;
  for (i = 0; i < (unsigned char)(endDLC - 1); i++) /* 结束帧有效数据 */
  {
    *pBuildFrame = *p;
    pBuildFrame++;
    p++;
  }
}

/*******************************************************
  函数名称:  HeatControl
  功能描述:  
  修改记录:  
********************************************************/
void HeatControl(void)
{
  unsigned char i;
  unsigned char j;
  unsigned char temp;                 /* 排序冒泡法用临时变量 */
  float temperature;                  /* 实测温度值  */
  unsigned char xdata heatParaTHT[7]; /* 7个热敏电阻排序用数组 */

  for (i = 0; i < 7; i++) /* 热敏电阻付初值 */
  {
    heatParaTH[i] = anData8[37 + i];
    heatParaTHT[i] = heatParaTH[i];
  }

  for (i = 0; i < 6; i++) /* 从小到大进行排序 */
  {
    for (j = 1; j < (unsigned char)(7 - i); j++)
    {
      if (heatParaTHT[i] > heatParaTHT[i + j])
      {
        temp = heatParaTHT[i + j];
        heatParaTHT[i + j] = heatParaTHT[i];
        heatParaTHT[i] = temp;
      }
    }
  }

  for (i = 0; i < 2; i++) /* 两路控温 */
  {
    if ((heatPara[i].Mode & 0xC0) == 0x40) /* 判断温控模式为闭环     */
    {
      if ((heatPara[i].Mode & 0x10) == 0x10) /* 判断按照D3D2D1D0位选取控温电阻 */
      {
        temperature = (float)heatParaTH[(heatPara[i].Mode & 0x07) - 1];
      }
        temperature = (float)((float)((unsigned int)heatParaTHT[2] + (unsigned int)heatParaTHT[3] + (unsigned int)heatParaTHT[4]) / 3.0); /* 选用中间三个平均值 */

      if (temperature < (float)heatPara[i].Close)
      {
        P1 = (unsigned char)(P1|(_crol_(0x01,i))); /* 向左循环移位 相或  关闭加热 */
      }
      else if (temperature > (float)heatPara[i].Open)
      {
        P1 = (unsigned char)(P1&(_crol_(0xFE,i))); /* 向左循环移位 相与  开启加热 */
      }
      else
      {
        _nop_();
      }
    }
    else if ((heatPara[i].Mode & 0xC0) == 0x80) /* 判断温控模式为开环 */
    {
      if ((heatPara[i].Mode & 0x20) == 0x20) /* 判断为开环开启加热 */
      {
        P1 = (unsigned char)(P1&(_crol_(0xFE,i))); /* 向左循环移位 相与  开启加热    */
      }
      else /* 判断为开环停止加热   */
      {
        P1 = (unsigned char)(P1|(_crol_(0x01,i)));  /* 向左循环移位 相或  关闭加热 */
      }
    }
    else
    {
      _nop_();
    }
  }
}

/*******************************************************
  函数名称:  DataProtect
  功能描述:  
  修改记录:  
********************************************************/
void DataProtect(void)
{
  unsigned char i;
  unsigned char result1;
  unsigned char result2;
  unsigned char result3;

  result1 = Get2_3_X(&proportion, &proportionB, &proportionC); /* 充放比、充电、放电参数3取2判决 */
  result2 = Get2_3_F(&chargePower, &chargePowerB, &chargePowerC);
  result3 = Get2_3_F(&dischargePower, &dischargePowerB, &dischargePowerC);
  if ((result1 == FALSE) || (result2 == FALSE) || (result3 == FALSE))
  {
    PowerInit(); /* 恢复全部参数 包含充放比参数 */
  }

  result1 = Get2_3_X(&voltageWarnValue, &voltageWarnValueB, &voltageWarnValueC); /* 蓄电池组电压3取2判决 */
  if (result1 == FALSE)
  {
    voltageWarnValue = ACCUMULATOR_VSET; /* 初始化默认参数 */
    voltageWarnValueB = ACCUMULATOR_VSET;
    voltageWarnValueC = ACCUMULATOR_VSET;
  }

  result1 = Get2_3_X(&resetCount, &resetCountB, &resetCountC); /* 热复位计数3取2判决 */
  if (result1 == FALSE)
  {
    resetCount = 0; /* 初始化默认参数  清零 */
    resetCountB = 0;
    resetCountC = 0;
  }

  result1 = Get2_3_X(&chargeEndValue, &chargeEndValueB, &chargeEndValueC); /* 蓄电池组充电终止电流值3取2判决 */
  if (result1 == FALSE)
  {
    chargeEndValue = 150; /* 初始化默认参数  3 (150) */
    chargeEndValueB = 150;
    chargeEndValueC = 150;
  }

  for (i = 0; i < 2; i++) /* 2路控温参数纠错 */
  {
    result1 = Get2_3_X(&heatPara[i].Mode, &heatParaB[i].Mode, &heatParaC[i].Mode); /* 控温参数3取2判决 */
    result2 = Get2_3_X(&heatPara[i].Close, &heatParaB[i].Close, &heatParaC[i].Close);
    result3 = Get2_3_X(&heatPara[i].Open, &heatParaB[i].Open, &heatParaC[i].Open);
    if ((result1 == FALSE) || (result2 == FALSE) || (result3 == FALSE))
    {
      heatPara[i].Mode = heatTab[i][0]; /* 使用默认控温参数 */
      heatPara[i].Close = heatTab[i][1];
      heatPara[i].Open = heatTab[i][2];

      heatParaB[i].Mode = heatTab[i][0]; /* 同时设置参数B、C */
      heatParaB[i].Close = heatTab[i][1];
      heatParaB[i].Open = heatTab[i][2];

      heatParaC[i].Mode = heatTab[i][0];
      heatParaC[i].Close = heatTab[i][1];
      heatParaC[i].Open = heatTab[i][2];
    }
  }

  result1 = Get2_3_X(&fgAhEn, &fgAhEnB, &fgAhEnC); /* 安时计使能标志判决处理  */
  if (result1 == FALSE)
  {
    fgAhEn = TRUE;
    fgAhEnB = TRUE;
    fgAhEnC = TRUE;
  }

  result1 = Get2_3_X(&fgChargeEn, &fgChargeEnB, &fgChargeEnC); /* 充电保护使能标志判决处理   */
  if (result1 == FALSE)
  {
    fgChargeEn = TRUE;
    fgChargeEnB = TRUE;
    fgChargeEnC = TRUE;
  }

  result1 = Get2_3_X(&fgDischargeEn, &fgDischargeEnB, &fgDischargeEnC); /* 过放保护使能标志判决处理 */
  if (result1 == FALSE)
  {
    fgDischargeEn = FALSE;
    fgDischargeEnB = FALSE;
    fgDischargeEnC = FALSE;
  }

  result1 = Get2_3_X(&fgAh, &fgAhB, &fgAhC); /* 安时计标志判决处理   */
  if (result1 == FALSE)
  {
    fgAh = FALSE;
    fgAhB = FALSE;
    fgAhC = FALSE;
  }

  result1 = Get2_3_X(&fgTinyCurrent, &fgTinyCurrentB, &fgTinyCurrentC); /* 涓流标志判决处理 */
  if (result1 == FALSE)
  {
    fgTinyCurrent = FALSE;
    fgTinyCurrentB = FALSE;
    fgTinyCurrentC = FALSE;
  }

  result1 = Get2_3_X(&equEn, &equEnB, &equEnC); /* 均衡使能标志判决处理  */
  if (result1 == FALSE)
  {
    equEn = FALSE;
    equEnB = FALSE;
    equEnC = FALSE;
  }

  result1 = Get2_3_I(&AD558set, &AD558setB, &AD558setC); /* DA输出判决处理 */
  if (result1 == FALSE)
  {
    AD558set = AD558_VSET;
    AD558setB = AD558_VSET;
    AD558setC = AD558_VSET;
  }

  result1 = Get2_3_X(&anCheckEn, &anCheckEnB, &anCheckEnC); /* 模拟量自检使能标志判决处理 */
  if (result1 == FALSE)
  {
    anCheckEn = FALSE;
    anCheckEnB = FALSE;
    anCheckEnC = FALSE;
  }

  result1 = Get2_3_X(&onoffNumber, &onoffNumberB, &onoffNumberC);          /* 执行指令号判决处理   */
  result2 = Get2_3_X(&fgONOFFExecute, &fgONOFFExecuteB, &fgONOFFExecuteC); /* 指令执行标志判决处理 */

  if ((result1 == FALSE) || (result2 == FALSE))
  {
    onoffNumber = 0;
    onoffNumberB = 0;
    onoffNumberC = 0;

    fgONOFFExecute = FALSE;
    fgONOFFExecuteB = FALSE;
    fgONOFFExecuteC = FALSE;
  }
}

/*******************************************************
  函数名称:  CanResume
  功能描述:  
  修改记录:  
********************************************************/
void CanResume(void)
{
  unsigned char aSR;
  unsigned char aCR;

  aCR = XBYTE[BUS_ADDR];     /* 读总线SJA1000控制字 */
  aSR = XBYTE[BUS_ADDR + 2]; /* 读总线SJA1000状态字 */

  if ((aSR & 0x02) == 0x02) /* 判断总线控制器数据溢出 */
  {
    EX0 = 0;            	/* 增加关中断操作   */
    CANInit(BUS_ADDR);		/* 对控制器重新初始化 */

    errorCount = 0;
    receiveSum = 0; 		/* 总线接收数据包参数清零    */
    receiveCount = 0;
    receiveFrame = 0;

    canResetCount++;        /* CAN复位计数++    */
    EX0 = 1; 				/* 增加开中断操作   */
  }

  if (errorCount > 110) 	/* 不符合协议时间超限  110 +- 10 s */
  {
    EX0 = 0;            	/* 增加关中断操作   */
    CANInit(BUS_ADDR); 		/* 对总线控制器重新初始化 */
    errorCount = 0;

    receiveSum = 0; 		/* 总线接收数据包参数清零     */
    receiveCount = 0;
    receiveFrame = 0;

    canResetCount++;        /* CAN复位计数++    */
    EX0 = 1; 				/* 增加开中断操作   */
  }

  if (((aSR & 0x80) == 0x80) || ((aCR & 0x02) == 0x00)) /* 判断总线关闭 或 控制器接收无效 */
  {
    errorCountA++; 			/* 总线计数+1 */
    if (errorCountA > 15) 	/* 持续16秒 */
    {
      EX0 = 0; 				/* 增加关中断操作   */

      CANInit(BUS_ADDR); 	/* 对总线控制器重新初始化 */

      errorCountA = 0;
      receiveSum = 0; 		/* 总线接收数据包参数清零    */
      receiveCount = 0;
      receiveFrame = 0;

      canResetCount++;      /* CAN复位计数++    */
      EX0 = 1;
    }
  }
}

/*******************************************************
  函数名称:  PowerControl
  功能描述:  
  修改记录:  
********************************************************/
void PowerControl(void)
{
  float xdata chargeCurrent; /* 电流临时变量	 */
  float xdata dischargeCurrent;
  float xdata currentPhalanx;
  float xdata currentPhalanx1;
  float xdata currentPhalanx2;
  float xdata currentPhalanx3;
  float xdata currentLoad;

  if (fgAhEn == TRUE)
  {
    chargeCurrent = (float)((K1 * (float)anData12[4] * 0.00125) + B1);     /* 充电电流物理值换算  */
    dischargeCurrent = (float)((K2 * (float)anData12[5] * 0.00125) + B2);  /* 放电电流物理值换算	 */
    currentLoad = (float)((K4 * (float)anData12[0] * 0.00125) + B4);       /* 负载电流 */
    currentPhalanx1 = (float)((K31 * (float)anData12[1] * 0.00125) + B31); /* 电流1 */
    currentPhalanx2 = (float)((K32 * (float)anData12[2] * 0.00125) + B32); /* 电流2 */
    currentPhalanx3 = (float)((K33 * (float)anData12[3] * 0.00125) + B33); /* 电流3 */
    currentPhalanx = currentPhalanx1 + currentPhalanx2 + currentPhalanx3;  /* 电流之和 */

    if (((currentPhalanx - currentLoad) >= (float)((chargeEndValue / 50.0) + 0.5)) || (chargeCurrent <= (float)(chargeEndValue / 50.0))) 
    {
      currentCount++;

      if (currentCount >= 15) /* 持续时间大于等于30s */
      {
        currentCount = 0;

        fgAh = TRUE;
        fgAhB = TRUE;
        fgAhC = TRUE;

        fgTinyCurrent = TRUE;
        fgTinyCurrentB = TRUE;
        fgTinyCurrentC = TRUE;

        currentPower = (float)150; /* 当前电量是150Ah */

        chargePower = (float)0;    /* 充电、放电电量是0Ah */
        chargePowerB = (float)0;
        chargePowerC = (float)0;

        dischargePower = (float)0; /* 同时设置参数A、B、C */
        dischargePowerB = (float)0;
        dischargePowerC = (float)0;

        if (fgChargeOnoffA == FALSE) /* 满足发送条件 */
        {
          ONOFFOutput(6);        /* 发送充电结束指令  */
          fgChargeOnoffA = TRUE; /* 反转标志 */
        }
      }
    }
    else
    {
      
      currentCount = 0;
     fgChargeOnoffA = FALSE; /* 设置未满足条件时标志 */
    }

    if ((fgAh == TRUE) && (dischargeCurrent > 1))
    {
      fgTinyCurrent = FALSE;
      fgTinyCurrentB = FALSE;
      fgTinyCurrentC = FALSE;
    }

    if ((fgAh == TRUE) && (fgTinyCurrent == FALSE)) /* 安时计标志为1，涓流标志为0 */
    {
      if (chargeCurrent > 0.1) /* 如果充电电流大于0.1A */
      {
        chargePower = chargePower + ((float)(chargeCurrent * 10) / (float)(18 * (proportion + 900)));   /* 充电电量计算A */
        chargePowerB = chargePowerB + ((float)(chargeCurrent * 10) / (float)(18 * (proportion + 900))); /* 充电电量计算B */
        chargePowerC = chargePowerC + ((float)(chargeCurrent * 10) / (float)(18 * (proportion + 900))); /* 充电电量计算C */
      }
      if (dischargeCurrent > 0.3) /* 如果放电电流大于0.3A */
      {
        dischargePower = dischargePower + (dischargeCurrent / 1800);   /* 放电电流计算A */
        dischargePowerB = dischargePowerB + (dischargeCurrent / 1800); /* 放电电流计算B */
        dischargePowerC = dischargePowerC + (dischargeCurrent / 1800); /* 放电电流计算C */
      }
      currentPower = 150 + chargePower - dischargePower; /* 当前电量计算 */

      if (chargePower >= dischargePower) /* 充电电量大于等于放电电量蓄电池充满 */
      {
        fgTinyCurrent = TRUE;
        fgTinyCurrentB = TRUE;
        fgTinyCurrentC = TRUE;

        currentPower = (float)150; /* 设置满电量	 */

        chargePower = (float)0;
        chargePowerB = (float)0;
        chargePowerC = (float)0;

        dischargePower = (float)0;
        dischargePowerB = (float)0;
        dischargePowerC = (float)0;

        if (fgChargeOnoffB == FALSE) /* 满足发送条件 */
        {
          ONOFFOutput(6);        /* 发送充电结束指令  */
          fgChargeOnoffB = TRUE; /* 反转标志 */
        }
      }
      else
      {
        fgChargeOnoffB = FALSE; /* 设置不满足条件标志 */
      }
    }

    if (currentPower <= 0) /* 如果当前电量值出现负值 */
    {
      currentPower = (float)0; /* 当前电量维持0Ah */
    }

    if (dischargePower >= 196) /* 如果放电电量超过196Ah */
    {
      dischargePower = dischargePower - chargePower; /* 放电电量取放电电量与充电电量差值 */
      chargePower = (float)0;                        /* 充电电量清零 */

      chargePowerB = chargePower;
      chargePowerC = chargePower;
      dischargePowerB = dischargePower;
      dischargePowerC = dischargePower;
    }

    powerSave[0] = (unsigned char)((unsigned int)((currentPower * 1000) / 3.0) >> 8);     /* 取当前电量的高字节 */
    powerSave[1] = (unsigned char)((unsigned int)((currentPower * 1000) / 3.0) & 0xFF);   /* 取当前电量的低字节 */
    powerSave[2] = (unsigned char)((unsigned int)((chargePower * 1000) / 3.0) >> 8);      /* 取充电电量的高字节 */
    powerSave[3] = (unsigned char)((unsigned int)((chargePower * 1000) / 3.0) & 0xFF);    /* 取充电电量的低字节 */
    powerSave[4] = (unsigned char)((unsigned int)((dischargePower * 1000) / 3.0) >> 8);   /* 取放电电量的高字节 */
    powerSave[5] = (unsigned char)((unsigned int)((dischargePower * 1000) / 3.0) & 0xFF); /* 取放电电量的低字节 */
  }
}

/*******************************************************
  函数名称:  chargeProtect
  功能描述:  过充保护 
  修改记录: 
********************************************************/
void ChargeProtect(void)
{
  unsigned char xdata flag;
  float xdata batteryVoltage;

  if (fgChargeEn == TRUE)
  {
    flag = FALSE; /* 设置初始状态 默认无过充 */

    batteryVoltage = (float)((K5 * (float)anData8[1] * 0.02) + B5); /* 蓄电池组电压物理值换算 */

    if (batteryVoltage >= 30.1) /* 蓄电池组电压>=30.1  */
    {
      warnCount[4]++;         /* 计数1自加 */
      if (warnCount[4] >= 30) /* 持续时间大于等于30s */
      {
        flag = TRUE; /* 设置过充标志 */
      }
    }
    else
    {
      warnCount[4] = 0; /* 计数1清零 */
    }

    if (aqpx[6] >= 850) /* 任一单体电压大于等于4.25V */
    {
      warnCount[5]++;         /* 计数2自加 */
      if (warnCount[5] >= 30) /* 持续时间大于等于30s */
      {
        flag = TRUE; /* 设置过充标志  */
      }
    }
    else
    {
      warnCount[5] = 0; /* 计数2清零 */
    }

    if (flag == TRUE)
    {
      workEnState = (unsigned char)(workEnState | 0x04); /* 过充标志为置1 */
      ONOFFOutput(6);                                    /* 发送充电结束指令  */
      flag = FALSE;
      warnCount[4] = 0;
      warnCount[5] = 0;
    }
  }
}

/*******************************************************
  函数名称:  dischargeProtect
  功能描述:  过放保护 
  修改记录: 
********************************************************/
void DischargeProtect(void)
{
  float xdata batteryVoltage;

  if (fgDischargeEn == TRUE) /* 过放允许 */
  {
    batteryVoltage = (float)((K5 * (float)anData8[1] * 0.02) + B5); /* 蓄电池组电压物理值换算 */

    if ((batteryVoltage <= 21) && (aqpx[3] <= 600)) /* 蓄电池电压小于等于21且任意三节以上电压小于等于3V (3节<=3V) */
    {
      warnCount[6]++;         /* 过放保护计数累加 */
      if (warnCount[6] >= 30) /* 持续时间>=30秒 */
      {
        workEnState = (unsigned char)(workEnState | 0x10); /* 过放标志为置1 */
        fgExceDischarge = TRUE;                            /* 设置已过放标志 */
      }
    }
    else
    {
      warnCount[6] = 0; /* 计数清零 */
    }

    if (fgExceDischarge == TRUE) /* 已过放 */
    {
      warnCount[7]++;        /* 已过放时间计数加1 */
      if (warnCount[7] > 32) /* 过放超过32S */
      {
        fgExceDischarge = FALSE;
        warnCount[6] = 0; /* 30s 和 16s 计时同时清零 */
        warnCount[7] = 0;
        ONOFFOutput(5); /* 发蓄电池组断开指令 */
      }
    }
  }
}

/*******************************************************
  函数名称:  ChargeUnlock
  功能描述:  
  修改记录:  

********************************************************/
void ChargeUnlock(void)
{
  unsigned char i;
  unsigned char flag;

  flag = FALSE;
  for (i = 0; i < 7; i++)
  {
    if ((aqpx[i] >= 600) && (aqpx[i] < 740))
    {
      flag = TRUE;
    }
  }

  if ((anData8[9] <= 60) && (anData8[10] == 0) && (flag == TRUE)) /* 判断成立 <=3.6V  =0  MEA=3X  标定处  20180403 */
  {
    chargeUnlockCount++;
    if (chargeUnlockCount >= 30)
    {
      ONOFFOutput(14);       /* 发送充电终止断开指令    */
      chargeUnlockCount = 0; /* 计数清零 */
    }
  }
  else
  {
    chargeUnlockCount = 0;
  }
}

/*******************************************************
  函数名称:  EquControlCut
  功能描述:  
  修改记录:  
********************************************************/
void EquControlCut(void)
{
  unsigned char i;
  float xdata chargeCurrent;
  float xdata dischargeCurrent;

  if (equEn == TRUE) /* 均衡控制允许 */
  {
    chargeCurrent = (float)((K1 * (float)anData12[4] * 0.00125) + B1);    /* 充电电流物理值换算  */
    dischargeCurrent = (float)((K2 * (float)anData12[5] * 0.00125) + B2); /* 放电电流物理值换算	 */

    if ((chargeCurrent <= 0.1) && (dischargeCurrent >= 1)) /* 充电电流小于等于0.1A且放电电流大于等于1A */
    {
      equOutputWord = 0x00; /* 断开7个旁路全断 */
      equControlEn = 0;
      EQU_ADDR = equOutputWord; /* 设置均衡控制输出 */
      for (i = 0; i < 7; i++)
      {
        equControlState[i] = FALSE; /* 旁路控制电路状态为关闭 */
      }
    }
  }
}

/*******************************************************
  函数名称:  EquControlLink
  功能描述:  
  修改记录:  
********************************************************/
void EquControlLink(void)
{
  unsigned char i;
  unsigned char index;
  float xdata chargeCurrent;
  float xdata dischargeCurrent;

  if (equEn == TRUE) /* 均衡控制允许 */
  {
    chargeCurrent = (float)((K1 * (float)anData12[4] * 0.00125) + B1);    /* 充电电流物理值换算  */
    dischargeCurrent = (float)((K2 * (float)anData12[5] * 0.00125) + B2); /* 放电电流物理值换算	 */

    if (dischargeCurrent <= 0.3) /* 放电电流小于等于0.3A */
    {
      index = 0;
      for (i = 0; i < 7; i++)
      {
        if (aq[i] > 600) /* 单体电压大于3V */
        {
          index++; /* 电压大于3V的单体计数 */
        }
      }

      if (index >= 1) /*  参与均衡控制的单体大于等于1 */
      {
        for (i = 0; i < 7; i++)
        {
          if ((aq[i] > 600) && (aq[i] < 860)) /* 单体电压大于3V 且 小于4.3V	 */
          {
            if (equControlState[i] == FALSE) /* 状态为关闭  FALSE 断开 */
            {
              if ((aq[i] - aqpx[7 - index]) >= 4) /* 差值大于40mv */
              {
                equOutputWord = (unsigned char)(equOutputWord | EQU_OPEN_Tab[i]); /* 指令接通   置1 */
                equControlState[i] = TRUE;                                        /* 置单体旁路控制电路状态为开启 */
              }
            }
            else /* 状态为接通  */
            {
              if ((aq[i] - aqpx[7 - index]) <= 2) /* 差值小于20mv   置0 */
              {
                equOutputWord = (unsigned char)(equOutputWord & EQU_CLOSE_Tab[i]); /* 指令断开 */
                equControlState[i] = FALSE;                                        /* 置单体旁路控制电路状态为关闭 */
              }
            }
          }
          else
          {
            equOutputWord = (unsigned char)(equOutputWord & EQU_CLOSE_Tab[i]); /* 指令断开 */
            equControlState[i] = FALSE;
          }
        }
      }
      equControlEn = 0; /* 设置输出状态 */
      EQU_ADDR = equOutputWord;
    }
  }
}

/*******************************************************
  函数名称:  SafeWarn
  功能描述:  
  修改记录:  
********************************************************/
void SafeWarn(void)
{
  unsigned char i;
  unsigned char j;
  unsigned char temp;
  unsigned char flag;            /* 标识临时变量 */
  unsigned char xdata heatTN[7]; /* 热敏电阻 */

  flag = FALSE;

  if (anData8[0] <= GENERATRIX_VSET) /* 母线电压 */
  {
    warnCount[0]++;
    if (warnCount[0] >= 15)
    {
      warnState = (unsigned char)(warnState | 0x88);     /* D7、D3 = 1 */
      healthState = (unsigned char)(healthState | 0x08); /* D3 = 1 */

      healthData[1] = anData8[0]; /* 异常时母线电压数据 */
      flag = TRUE;
    }
  }
  else
  {
    warnCount[0] = 0;
    healthState = (unsigned char)(healthState & 0xF7); /* D3 = 0 */
  }

  if (anData8[1] <= voltageWarnValue) /* 蓄电池组电压监视 */
  {
    warnCount[1]++;
    if (warnCount[1] >= 15)
    {
      warnState = (unsigned char)(warnState | 0x44);     /* D6、D2 = 1 */
      healthState = (unsigned char)(healthState | 0x04); /* D2 = 1 */

      healthData[2] = anData8[1]; /* 异常时蓄电池组电压数据 */
      flag = TRUE;
    }
  }
  else
  {
    warnCount[1] = 0;
    healthState = (unsigned char)(healthState & 0xFB); /* D2 = 0 */
  }

  if (anData8[1] <= (unsigned char)(voltageWarnValue + 6)) /* 蓄电池组电压监视 预报警 */
  {
    warnCount[8]++;
    if (warnCount[8] >= 15)
    {
      fgPreWarn = TRUE; /* 设置预警标志 */
    }
  }
  else
  {
    warnCount[8] = 0;
  }

  if (currentPower <= 100) /* 当前电量监视(<=100Ah)  */
  {
    warnCount[2]++;
    if (warnCount[2] >= 15)
    {
      warnState = (unsigned char)(warnState | 0x22);     /* D5、D1 = 1 */
      healthState = (unsigned char)(healthState | 0x02); /* D1 = 1 */

      healthData[3] = powerSave[0]; /* 异常时当前电量数据 */
      flag = TRUE;
    }
  }
  else
  {
    warnCount[2] = 0;
    healthState = (unsigned char)(healthState & 0xFD); /* D1 = 0 */
  }

  for (i = 0; i < 7; i++)
  {
    heatTN[i] = anData8[37 + i]; /* 读取蓄电池组7个温度值   */
  }

  for (i = 0; i < 6; i++) /* 从小到大进行排序 */
  {
    for (j = 1; j < (unsigned char)(7 - i); j++)
    {
      if (heatTN[i] > heatTN[i + j])
      {
        temp = heatTN[i + j];
        heatTN[i + j] = heatTN[i];
        heatTN[i] = temp;
      }
    }
  }

  if (((float)((unsigned int)heatTN[2] + (unsigned int)heatTN[3] + (unsigned int)heatTN[4]) / 3.0) <= TEMPE_35C) /* 中间值取平均 35C */
  {
    warnCount[3]++;
    if (warnCount[3] >= 15) /* 当前温度监视(>=35℃)  */
    {
      warnState = (unsigned char)(warnState | 0x11);     /* D4、D0 = 1 */
      healthState = (unsigned char)(healthState | 0x01); /* D0 = 1 */

      healthData[4] = (unsigned char)((float)((unsigned int)heatTN[2] + (unsigned int)heatTN[3] + (unsigned int)heatTN[4]) / 3.0); /* 异常时温度平均数据 */
      flag = TRUE;
    }
  }
  else
  {
    warnCount[3] = 0;
    healthState = (unsigned char)(healthState & 0xFE); /* D0 = 0 */
  }

  if (flag == TRUE) /* 分系统危险标志 */
  {
    healthState = (unsigned char)(healthState | 0x80); /* D7 = 1 */
  }
  else
  {
    healthState = (unsigned char)(healthState & 0x7F); /* D7 = 0 */
  }
}

/*******************************************************
  函数名称:  ONOFFHook
  功能描述:  
  修改记录:  
********************************************************/
void ONOFFHook(void)
{
  unsigned char i;

  if ((onoffNumber > 0) && (onoffNumber < 37) && (fgONOFFExecute == TRUE)) /* 1 - 36 指令范围 、 指令请求处*/
  {
    if (onoffNumber < 17) /* OC指令  1-16 */
    {
	  if(onoffNumber == 0x05)	/* 蓄电池组放电开关断开指令 */
	  {
	    if(fgDischargeEn == TRUE)  	/* 过放保护允许时 */
		{
	       ONOFFOutput(onoffNumber); /* 执行OC指令输出 */	
		}
           rsCount++;        /* 遥测计数+1 */

	  }	
      else
	  {
         ONOFFOutput(onoffNumber); /* OC指令输出 */		  
  		 rsCount++;          /* 遥测计数+1 */
	  }
    }
    else
    {
      if (onoffNumber < 24) /* 旁路指令范围  17-23 */
      {
        equOutputWord = (unsigned char)(equOutputWord | (_crol_((unsigned char)0x01, (unsigned char)(onoffNumber - 17)))); /* 旁路输出控制 */
        equControlEn = 0;                                                                                                  /* 旁路设置使能有效 */
        EQU_ADDR = equOutputWord;                                                                                          /* 设置旁路输出 */

        equControlState[onoffNumber - 17] = TRUE; /* 设置旁路控制电路开关状态 */

        rsCount++;        /* 遥测计数+1 */
      }
      else /* 数字指令范围 */
      {
        switch (onoffNumber)
        {
        case 24: /* 均衡控制允许 */
          equEn = TRUE;
          equEnB = TRUE;
          equEnC = TRUE;

          rsCount++;       /* 遥测计数+1 */
          break;

        case 25: /* 均衡控制禁止 */
          equEn = FALSE;
          equEnB = FALSE;
          equEnC = FALSE;

          equOutputWord = 0x00; /* 断开7个旁路输出 */
          equControlEn = 0;
          EQU_ADDR = equOutputWord; /* 设置均衡控制输出 */
          for (i = 0; i < 7; i++)
          {
            equControlState[i] = FALSE; /* 设置单体旁路控制电路开关状态 */
          }

          rsCount++;        /* 遥测计数+1 */
          break;

        case 26: /* 安时计控制允许 */
          fgAhEn = TRUE;
          fgAhEnB = TRUE;
          fgAhEnC = TRUE;

          rsCount++;       /* 遥测计数+1 */
          break;

        case 27: /* 安时计控制禁止 */
          fgAhEn = FALSE;
          fgAhEnB = FALSE;
          fgAhEnC = FALSE;

          fgAh = FALSE; /* 设置安时计控制标志(表示0) */
          fgAhB = FALSE;
          fgAhC = FALSE;

          fgTinyCurrent = FALSE; /* 设置涓流标志(表示0) */
          fgTinyCurrentB = FALSE;
          fgTinyCurrentC = FALSE;

          powerSave[0] = FULLPOWER_H; /* 当前电量150Ah  （下传值） */
          powerSave[1] = FULLPOWER_L;
          powerSave[2] = 0; /* 充电电量0Ah */
          powerSave[3] = 0;
          powerSave[4] = 0; /* 放电电量0Ah */
          powerSave[5] = 0;

          currentPower = (float)FULLPOWER; /* 当前电量参数初始值 （程序计算使用值） */

          chargePower = (float)0; /* 充电电量参数初始值	 */
          chargePowerB = (float)0;
          chargePowerC = (float)0;

          dischargePower = (float)0; /* 放电电量参数初始值 */
          dischargePowerB = (float)0;
          dischargePowerC = (float)0;

          rsCount++;         /* 遥测计数+1 */
          break;

        case 28: /* 过放保护控制允许 */
          fgDischargeEn = TRUE;
          fgDischargeEnB = TRUE;
          fgDischargeEnC = TRUE;

          rsCount++;       /* 遥测计数+1 */
          break;

        case 29: /* 过放保护控制禁止 */
          fgDischargeEn = FALSE;
          fgDischargeEnB = FALSE;
          fgDischargeEnC = FALSE;

          rsCount++;       /* 遥测计数+1 */
          break;

        case 30: /* 清除标志 */
          workEnState = (unsigned char)(workEnState & 0xFB);
          warnCount[4] = 0;
          warnCount[5] = 0;

          rsCount++;       /* 遥测计数+1 */
          break;

        case 31: /* 过充保护允许 */
          fgChargeEn = TRUE;
          fgChargeEnB = TRUE;
          fgChargeEnC = TRUE;

          rsCount++;     /* 遥测计数+1 */
          break;

        case 32: /* 过充保护禁止 */
          fgChargeEn = FALSE;
          fgChargeEnB = FALSE;
          fgChargeEnC = FALSE;

          rsCount++;      /* 遥测计数+1 */
          break;

        case 33: /* 健康状态清零指令 */
          for (i = 0; i < 4; i++)
          {
            warnCount[i] = 0;      /* 能源安全状态异常计数清零 */
            healthData[i + 1] = 0; /* 健康状态数据清零 */
          }
          warnState = 0;   /* 能源安全状态字清零 */
          healthState = 0; /* 健康状态标志清零 */

          rsCount++;     /* 遥测计数+1 */
          break;

        case 34:
          anCheckEn = TRUE; /* 启动模拟量自检功能允许 */
          anCheckEnB = TRUE;
          anCheckEnC = TRUE;

          rsCount++;       /* 遥测计数+1 */
          break;

        case 35:              /* 总线初始化指令 */
          EX0 = 0;            /* 增加关中断操作   */
          CANInit(BUS_ADDR); /* 禔总线控制器重新初始化 */
          errorCount = 0;
          receiveSum = 0; /* 总线接收数据包参数清零    */
          receiveCount = 0;
          receiveFrame = 0;

          canResetCount++;                         /* CAN复位计数++    */
          EX0 = 1;

          rsCount++;      /* 遥测计数+1 */
          break;

        case 36:             /* 预警值清零指令 */
          fgPreWarn = FALSE; /* 清除标志 */
          warnCount[8] = 0;

          rsCount++;      /* 遥测计数+1 */
		  break;

        default:
          break;
        }
      }
    }

    onoffNumber = 0; /* 指令号清零 */
    onoffNumberB = 0;
    onoffNumberC = 0;

    fgONOFFExecute = FALSE; /* 设置指令请求无效 */
    fgONOFFExecuteB = FALSE;
    fgONOFFExecuteC = FALSE;

    rsCount = (unsigned char)(rsCount & 0x0F); /* 有效范围 0- F    */
  }
}

/*******************************************************
  函数名称:  ONOFFOutput
  功能描述:  
  修改记录:  
********************************************************/
void ONOFFOutput(unsigned char index)
{
  ONOFF_ADDR = onoffTab[index - 1]; /* 设置指令输出通道 */
  onoffEn = 0;                      /* 指令输出使能有效 */
  Delay(11340);                     /* 延时约160ms */
  ONOFF_ADDR = 0xFF;                /* 设置无效指令输出通道 */
  onoffEn = 1;                      /* 指令输出使能无效 */
}

/*******************************************************
  函数名称:  DataLoad
  功能描述:  
  修改记录:  
********************************************************/
void DataLoad(void)
{
  unsigned char i;

  if (fgDataRequest == TRUE) /* 数据块上注查询处理 */
  {
    if (uploadData[0] == 0x01) /* 01H为充放电参数数据块 */
    {
      if ((uploadData[2] >= 50) && (uploadData[2] <= 250)) /* 蓄电池组充电终止电流 W=X*100/2 范围1-5 */
      {
        chargeEndValue = uploadData[2]; /*  */
        chargeEndValueB = uploadData[2];
        chargeEndValueC = uploadData[2];
      }

      if (uploadData[3] <= 250) /* 充放比为0.9-1.15 X = W*1000 - 900 */
      {
        proportion = uploadData[3]; /* 设置充放比 */
        proportionB = uploadData[3];
        proportionC = uploadData[3];
      }

      if ((uploadData[4] >= 100) && (uploadData[4] <= 150)) /* 上注值 100 - 150 */
      {
        fgAh = TRUE; /* 强制启动安时计 */
        fgAhB = TRUE;
        fgAhC = TRUE;

        fgTinyCurrent = FALSE;
        fgTinyCurrentB = FALSE;
        fgTinyCurrentC = FALSE;

        dischargePower = (float)(FULLPOWER + chargePower - (float)(uploadData[4])); 
        dischargePowerB = dischargePower;
        dischargePowerC = dischargePower;
      }

      AD558set = (unsigned int)uploadData[5]; /* 充电电流设置电平   无条件接收执行 */
      AD558setB = (unsigned int)uploadData[5];
      AD558setC = (unsigned int)uploadData[5];
    }

    if (uploadData[0] == 0x02) /* 02H为温控参数数据块 */
    {
        if ((uploadData[3] & 0xC0) == 0x40) /* 判断为闭环模式 */
        {
          if (((uploadData[3] & 0x0F) < 8) && ((uploadData[3] & 0x0F) > 0)) /* 判断为有效选择的热敏电阻 */
          {
            heatPara[uploadData[2] - 1].Mode = uploadData[3];
            heatParaB[uploadData[2] - 1].Mode = uploadData[3];
            heatParaC[uploadData[2] - 1].Mode = uploadData[3];
          }
        }

        if ((uploadData[3] & 0xC0) == 0x80) /* 判断开环 */
        {
          heatPara[uploadData[2] - 1].Mode = uploadData[3];
          heatParaB[uploadData[2] - 1].Mode = uploadData[3];
          heatParaC[uploadData[2] - 1].Mode = uploadData[3];
        }

        if ((uploadData[4] >= TEMPE_35C) && (uploadData[5] <= TEMPE_10C)) /* 判断关闭门限小于开启门限 */
        {
          heatPara[uploadData[2] - 1].Close = uploadData[4]; /* 温控关闭、开启门限  */
          heatPara[uploadData[2] - 1].Open = uploadData[5];

          heatParaB[uploadData[2] - 1].Close = uploadData[4]; /* 温控关闭、开启门限 备用B */
          heatParaB[uploadData[2] - 1].Open = uploadData[5];

          heatParaC[uploadData[2] - 1].Close = uploadData[4]; /* 温控关闭、开启门限 备用C */
          heatParaC[uploadData[2] - 1].Open = uploadData[5];
        }
    

      if ((uploadData[1] >= ACCUMULATOR_VMIN) && (uploadData[1] <= ACCUMULATOR_VMAX)) /* 蓄电池组电压 */
      {
        voltageWarnValue = uploadData[1];
        voltageWarnValueB = uploadData[1];
        voltageWarnValueC = uploadData[1];
      }
    }

    fgDataRequest = FALSE;

    rsCount++;
    rsCount = (unsigned char)(rsCount & 0x0F);

    for (i = 0; i < 8; i++) /* 上注数据块清零 */
    {
      uploadData[i] = 0;
    }
  }
}

/*******************************************************
  函数名称:  Get2_3_X
  功能描述:  
  修改记录:  
********************************************************/
unsigned char Get2_3_X(unsigned char *a, unsigned char *b, unsigned char *c)
{
  if ((*a == *b) && (*b == *c)) /* 三区对比一致 */
  {
    return (TRUE); /* 3取2判读正确 */
  }
  else
  {
    if (*a == *b) /* 前两个数相等	 */
    {
      EA = 0;  /* 关中断 防止中断读写冲突    */
      *c = *a; /* 赋值操作 */
      EA = 1;
      return (TRUE); /* 3取2判读正确 */
    }
    else
    {
      if (*a == *c) /* 另两个数相等 */
      {
        EA = 0;  /* 关中断 防止中断读写冲突      */
        *b = *a; /* 赋值操作 */
        EA = 1;
        return (TRUE); /* 3取2判读正确 */
      }
      else
      {
        if (*b == *c) /* 剩余两个数相等 */
        {
          EA = 0;  /* 关中断 防止中断读写冲突    */
          *a = *b; /* 赋值操作 */
          EA = 1;
          return (TRUE); /* 3取2判读正确 */
        }
        else
        {
          return (FALSE); /* 3取2判读异常 */
        }
      }
    }
  }
}

/*******************************************************
  函数名称:  Get2_3_I
  功能描述:  
  修改记录:  
********************************************************/
unsigned char Get2_3_I(unsigned int *a, unsigned int *b, unsigned int *c)
{
  if ((*a == *b) && (*b == *c)) /* 三区对比一致 */
  {
    return (TRUE); /* 3取2判读正确 */
  }
  else
  {
    if (*a == *b) /* 前两个数相等	 */
    {
      *c = *a;       /* 赋值操作 */
      return (TRUE); /* 3取2判读正确 */
    }
    else
    {
      if (*a == *c) /* 另两个数相等 */
      {
        *b = *a;       /* 赋值操作 */
        return (TRUE); /* 3取2判读正确 */
      }
      else
      {
        if (*b == *c) /* 剩余两个数相等 */
        {
          *a = *b;       /* 赋值操作 */
          return (TRUE); /* 3取2判读正确 */
        }
        else
        {
          return (FALSE); /* 3取2判读异常 */
        }
      }
    }
  }
}

/*******************************************************
  函数名称:   Get2_3_F
  功能描述:  
  修改记录:  
********************************************************/
unsigned char Get2_3_F(float *a, float *b, float *c)
{
  float temp;

  temp = *a - *b; /* 取两个值的绝对值     */
  if (temp < 0.0001)    /* 判读两个绝对值之差小于很小的数即为相等（符合规范要求）  */
  {
    *c = *a;       /* 赋值操作 */
    return (TRUE); /* 3取2判读正确 */
  }
  else
  {
    temp = *a - *c; /* 另两个值之差绝对值 */
    if (temp < 0.0001)    /* 满足很小的数值 */
    {
      *b = *a;       /* 赋值操作 */
      return (TRUE); /* 3取2判读正确 */
    }
    else
    {
      temp = *b - *c; /* 剩余其他两个 */
      if (temp < 0.0001)    /* 满足很小的数值 */
      {
        *a = *b;       /* 赋值操作 */
        return (TRUE); /* 3取2判读正确 */
      }
      else /* 3个值均不相等 */
      {
        return (FALSE); /* 3取2判读异常 */
      }
    }
  }
}

/*******************************************************
  函数名称:  Delay
  功能描述:  
  修改记录:  
********************************************************/
void Delay(unsigned int time)
{
  unsigned int k;
  for (k = 0; k < time; k++) /* 循环延时 */
  {
    _nop_(); /* 执行空语句 */
  }
}

/*******************************************************
  函数名称:  CAN_ISR
  功能描述:  
  修改记录:  
********************************************************/
void CAN_ISR(void) interrupt 0 using 1
{
  unsigned char IR;

  IR = XBYTE[BUS_ADDR + 3]; /* 读总线中断寄存器 */
  if ((IR & 0x01) == 0x01)   /* 中断寄存器接收状态字判读 */
  {
    CANRXD(); /* 总线接收 */
  }
  else if ((IR & 0x02) == 0x02) /* 中断寄存器发送状态字判读 */
  {
    CANTXD(); /* 总线发送 */
  }
  else
  {
    _nop_(); /* 异常中断处理 */
  }
}

/*******************************************************
  函数名称:  CANRXD
  功能描述:  
  修改记录:  
********************************************************/
void CANRXD(void) using 1
{
  unsigned char i; /* 定义临时变量 */
  unsigned char DLC;
  unsigned char length;
  unsigned char fgEndFrame;
  unsigned char xdata *p;

  if ((XBYTE[0x5014] == 0x86) || (XBYTE[0x5014] == 0x46)) /* 判断为电源下位机识别码 */
  {
    fgEndFrame = FALSE;                 /* 置数据包接收结束标志 */
    DLC = (XBYTE[0x5015] & 0x0F);       /* 帧有效数据长度 */
    if ((XBYTE[0x5015] & 0x20) == 0x00) /* 判断单帧/多帧结束帧/多帧中断帧 */
    {
      length = DLC;                                    /* 置接收数据长度 */
      p = 0x5016;                                      /* 置接收数据指针 */
      receiveSum = 0;                                 /* 接收数据累加和清零 */
      receiveBuffer[0] = (unsigned char)(length - 2); /* 存储长度字 */
      receiveCount = 1;
    }
    else
    {
      if (DLC < 8) /* 数据帧为多帧结束帧 */
      {
        length = (unsigned char)(DLC - 1);             /* 置接收数据长度 */
        receiveSum = receiveSum - receiveBuffer[0]; /* 接收数据累加和减去多帧数据的length字节 */
      }
      else /* 数据帧为多帧中继帧 */
      {
        length = 7;        /* 置接收数据长度 */
        fgEndFrame = TRUE; /* 置数据包正在接收标志 */
      }
      p = 0x5017; /* 置接收数据指针(不包括帧index) */

      if ((XBYTE[0x5014] != 0x46) || (XBYTE[0x5016] == 0) || (XBYTE[0x5016] != receiveFrame))
      {
        receiveSum = 0; /* 累加和、计数清零 */
        receiveCount = 0;
        receiveFrame = 0;
      }
      receiveFrame++; /* 多帧计数增加 */
    }

    for (i = 0; i < length; i++) /* 接收数据(TITLE+DATA+SUM)并计算累加和 */
    {
      receiveBuffer[receiveCount] = *p;
      receiveSum = receiveSum + *p; /* 累加和计算(Title + Data + Sum) */
      p++;
      receiveCount++; /* 存储DATA计数 */
    }

    if (fgEndFrame == FALSE)
    {
      receiveSum = ((receiveSum - receiveBuffer[receiveCount - 1]) & 0x00FF); /* 计算累加和 */

      if ((receiveSum == receiveBuffer[receiveCount - 1]) /* 判断数据包的累加和与长度都正确 */
          && (receiveCount == (unsigned char)(receiveBuffer[0] + 3)))
      {
        errorCount = 0; /* 通讯错误计数清零 */
        switch (receiveBuffer[1])
        {
        case 0x00: /* 轮询控制处理 */
          if ((XBYTE[0x5014] == 0x86) && (receiveBuffer[2] == 0x10))
          {
            RsManage();
          }
          break;

        case 0x20: /* 间接指令处理 */
          if (XBYTE[0x5014] == 0x46)
          {
            ONOFFManage();
          }
          break;

        case 0x40: /* 上行数据块(控温参数、充电参数)程序块处理   */
          if (XBYTE[0x5014] == 0x46)
          {
            DataManage();
          }
          break;

        default:
          break;
        }
      }
      receiveSum = 0; /* 累加和清零 */
      receiveCount = 0;
      receiveFrame = 0;
    }
  }
  XBYTE[0x5001] = 0x0C; /* 释放接收缓存器 */
}

/*******************************************************
  函数名称:  CANTXD
  功能描述:  
  修改记录:  
********************************************************/
void CANTXD(void) using 1
{
  unsigned char i;

  if (sendIndex != sendFrame) /* 判断应答帧是否发完 */
  {
    for (i = 0; i < 10; i++) /* 构成发送帧 */
    {
      XBYTE[0x500A + i] = *pSend;
      pSend++;
    }
    XBYTE[0x5001] = 0x01;
    sendIndex++; /* 发送帧索引号+1 */
  }
  else
  {
    sendIndex = 1; /* 设置发送结束 */
    sendFrame = 1;
  }
}

/*******************************************************
  函数名称:  RsManage
  功能描述:  
  修改记录:  
********************************************************/
void RsManage(void) using 1
{
  unsigned char i;
  unsigned char xdata *p;
  p = 0x500A; /* 指向CAN总线发送缓冲区 */

  switch (receiveBuffer[3]) /* 判断控制轮询数据类型 */
  {
  case 0x01:               /* 状态遥测参数 */
    if (fgSwitch == FALSE) /* 切换B缓冲区发送 */
    {
      pSend = &soonFrameB[0];
    }
    else
    {
      pSend = &soonFrameA[0]; /* 切换A缓冲区发送 */
    }
    for (i = 0; i < 10; i++) /* 写入发送缓冲区 */
    {
      *p = *pSend;
      p++;
      pSend++;
    }
    XBYTE[0x5001] = 0x01; /* 启动发送帧 */
    sendIndex = 1;
    sendFrame = 5; /* 5帧速变参数 */
    break;

  case 0x02:               /* 电源缓变遥测参数 */
    if (fgSwitch == FALSE) /* 切换B区发送 */
    {
      pSend = &slowFrameB[0];
    }
    else
    {
      pSend = &slowFrameA[0]; /* 切换A区发送 */
    }

    for (i = 0; i < 10; i++) /* 写入发送缓冲区 */
    {
      *p = *pSend;
      p++;
      pSend++;
    }
    XBYTE[0x5001] = 0x01; /* 启动发送帧 */
    sendIndex = 1;
    sendFrame = 11; /* 11帧缓变参数 */

    fgDownRs = TRUE; /* 默认禁止切换 */
    break;

  default:
    break;
  }
}

/*******************************************************
  函数名称:  ONOFFManage
  功能描述:  
  修改记录:  
********************************************************/
void ONOFFManage(void) using 1
{
  unsigned char xdata *p;

  if ((receiveBuffer[2] == 0xA3) && (receiveBuffer[3] > 0) && (receiveBuffer[3] < 37 
      && (receiveBuffer[4] == 0xAA) && (receiveBuffer[5] == 0xAA))                     /* 标识、范围、填充字 */
  {
    p = 0x500A; /* 指向CAN总线发送缓冲区 */
    *p = 0x46;  /* 应答码    */
    p++;
    *p = 0x44;
    p++;
    *p = 0x03;
    p++;
    *p = 0xFF;
    p++;
    *p = 0x20;
    p++;
    *p = 0x22;

    XBYTE[0x5001] = 0x01; /* 启动发送帧 */
    sendIndex = 1;       /* 构成发送信息 */
    sendFrame = 1;

    if ((onoffNumber == 0) && (fgONOFFExecute == FALSE)) /* 判断当前无指令执行时 */
    {
      onoffNumber = receiveBuffer[3]; /* 读取指令号 */
      onoffNumberB = receiveBuffer[3];
      onoffNumberC = receiveBuffer[3];

      fgONOFFExecute = TRUE; /* 设置指令执行标志 */
      fgONOFFExecuteB = TRUE;
      fgONOFFExecuteC = TRUE;
    }
  }
}

/*******************************************************
  函数名称:  DataManage
  功能描述:  
  修改记录:   
********************************************************/
void DataManage(void) using 1
{
  unsigned char i;
  unsigned char xdata *p;

  p = 0x500A; /* 指向CAN总线发送缓冲区 */
  *p = 0x46;  /* 应答码 */
  p++;
  *p = 0x44;
  p++;
  *p = 0x03;
  p++;
  *p = 0xFF;
  p++;
  *p = 0x40;
  p++;
  *p = 0x42;

  XBYTE[0x5001] = 0x01; /* 启动发送帧 */
  sendIndex = 1;       /* 构成发送信息 */
  sendFrame = 1;

  if (receiveBuffer[2] == 0x55) /* 判断是上注数据块 */
  {
    for (i = 0; i < 8; i++) /* 读取上注参数有效内容 */
    {
      uploadData[i] = receiveBuffer[i + 3];
    }

    fgDataRequest = TRUE; /* 设置数据块请求标志 */
  }
}

/*******************************************************
  函数名称:  TIMER_ISR
  功能描述:  
  修改记录:  
********************************************************/
void TIMER_ISR(void) interrupt 1 using 2
{
  TR0 = 0;    /* 定时器重新计数 */
  TH0 = 0xDC; /* 装载新值 */
  TL0 = 0x00;
  TR0 = 1;
  clockCount++; /* 时钟计数 */
}


