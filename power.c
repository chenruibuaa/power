/****************************************
  ������ƣ���ع�������λ�����
************************************************************************************************/

#include <reg52.h> /* ��׼ͷ�ļ����� */
#include <absacc.h>
#include <intrins.h>
#include <math.h>

#define TRUE 0xAA  /* ������ֵ */
#define FALSE 0x55 /* �����ֵ */

#define CAN_ACR 0xC6 /* ������Ĵ��� */
#define CAN_AMR 0xC0 /* �������μĴ���  */

#define SILENT_TIME 4 /* ������߳���ʱ��(s) */

#define K1 9.0779 /* �������任ϵ�� */
#define B1 -0.0552
#define K2 13.386 /* �ŵ�����任ϵ�� */
#define B2 -0.0832

#define K31 6.0155 /* ���ʵ���1 */
#define B31 0.1005
#define K32 6.0206 /* ���ʵ���2 */
#define B32 0.0462
#define K33 5.9919 /* ���ʵ���3 */
#define B33 -0.0253

#define K4 10.02 /* ���ص��� */
#define B4 0
#define K5 6.11 /* �������ѹ */
#define B5 0.072

#define FULLPOWER_H 0xC3 /* �������´�ֵ���ֽ� */
#define FULLPOWER_L 0x50 /* �������´�ֵ���ֽ� */

#define FULLPOWER 150 /* ������150Ah */

#define AD558_VSET 128        /* AD558�ϵ����Ĭ��ֵ  */
#define GENERATRIX_VSET 0x92  /* ĸ�ߵ�ѹ��������ֵ(24V)           -146   Y = 8.1993X - 0.0025    */
#define ACCUMULATOR_VSET 0xBC /* �������ѹ��������Ĭ��ֵ(23.1V) -188   Y = 6.11X   + 0.072     */
#define ACCUMULATOR_VMIN 0x9D /* �������ѹ��������Ĭ��ֵ(19.2V) -157                           */
#define ACCUMULATOR_VMAX 0xCE /* �������ѹ��������Ĭ��ֵ(25.2V) -206                          */

#define TEMPE_22C 0x4C /* ����·���ȹر�����Ĭ��ֵ(22��)   */
#define TEMPE_19C 0x53 /* ����·���ȿ�������Ĭ��ֵ(19��)   */
#define TEMPE_21C 0x4E /* ����·���ȹر�����Ĭ��ֵ(21��)   */
#define TEMPE_18C 0x55 /* ����·���ȿ�������Ĭ��ֵ(18��)   */
#define TEMPE_10C 0x69 /* ����������ע��Сֵ(10��)         */
#define TEMPE_35C 0x35 /* ���°�ȫģʽ�ж����ޡ�����������ע���ֵ(35��)  */

#define BUS_ADDR 0x5000          /* ���ߵ�ַ         */
#define ONOFF_ADDR XBYTE[0x4800] /* ONOFFָ���ַ     */
#define AN_ADDR XBYTE[0x4400]    /* ģ������ַ        */
#define AD574H XBYTE[0x6000]     /* AD574ת���ߵ�ַ   */
#define AD574L XBYTE[0x6001]     /* AD574ת���͵�ַ   */
#define AD558 XBYTE[0x4C00]      /* AD558��ַ         */
#define EQU_ADDR XBYTE[0x5800]   /* ������������ַ   */

const unsigned char code rsTab8[57] = {
    /* 8bitģ�����ɼ����                   */
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

const unsigned char code rsTab12[6] = {0x16, 0x25, 0x17, 0x02, 0x13, 0x15};    /* �����ɼ���� */
const unsigned char code onoffTab[16] = {0x04, 0x03, 0x0C, 0x0F, 0x06, 0x05, 0x0A, 0x0B, 0x08, 0x09, 0x0E, 0x01, 0x0D, 0x07, 0x02, 0x00}; /* ���ָ��ͨ���  */

const unsigned char code heatTab[2][3] = {
    {0x41, TEMPE_22C, TEMPE_19C},
    {0x41, TEMPE_21C, TEMPE_18C}};

const unsigned char code EQU_CLOSE_Tab[7] = {0xFE, 0xFD, 0xFB, 0xF7, 0xEF, 0xDF, 0xBF}; /* ������ƹر� */
const unsigned char code EQU_OPEN_Tab[7] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40};  /* ������ƿ��� */

sbit onoffEn = P1 ^ (unsigned int)(0x06);      /* ָ�����ʹ��   */
sbit fgRstType = P1 ^ (unsigned int)(0x04);    /* �ȸ�λ��ʶ��0=�临λ 1=�ȸ�λ  */
sbit dog = P1 ^ (unsigned int)(0x05);          /* ���Ź����   */
sbit equControlEn = P1 ^ (unsigned int)(0x02) ; /* ����������ʹ�ܿ���  */

struct _HEAT_PARA
{
  unsigned char Mode;  /* �¿�ģʽ(����OR�ջ�)  */
  unsigned char Close; /* �¿عر�����  */
  unsigned char Open;  /* �¿ؿ�������  */
};

struct _HEAT_PARA xdata heatPara[2];         /* 2·�¿� A�� */
struct _HEAT_PARA xdata heatParaB[2];        /* 2·�¿� B�� */
struct _HEAT_PARA xdata heatParaC[2];        /* 2·�¿� C�� */

unsigned char xdata heatParaTH[7]; 				/* ����7����������     */
unsigned char clockCount;                    	/* 1s�������ж�ʱ������	           */
unsigned char xdata currentCount;  				/* ����30s����ʱ�����  */
unsigned char       errorCount;              	/* ����ͨѶ������Э�������           */
unsigned char xdata errorCountA;                /* ����ͨѶ������Э�������A  */

unsigned char xdata resetCount;  /* �ȸ�λ���� A */
unsigned char xdata resetCountB; /* �ȸ�λ���� B */
unsigned char xdata resetCountC; /* �ȸ�λ���� C */

unsigned char xdata rsCount;          /* ң��ָ��ִ�м���(����ָ���ע����) */
unsigned char xdata downHeatNumber;   /* �´�����״̬��··��   */
unsigned char xdata downHealthNumber; /* �´�����״̬��··��  */
unsigned char xdata fgSwitch;         /* ���ݲɼ�����������־  */
unsigned char xdata fgDataRequest;    /* ���������ݿ�Ҫ���־  */

unsigned char xdata onoffNumber;  /* ִ��ָ���A     */
unsigned char xdata onoffNumberB; /* ִ��ָ���B     */
unsigned char xdata onoffNumberC; /* ִ��ָ���C     */

unsigned char xdata fgONOFFExecute;  /* ָ��ִ�б�־A   */
unsigned char xdata fgONOFFExecuteB; /* ָ��ִ�б�־B   */
unsigned char xdata fgONOFFExecuteC; /* ָ��ִ�б�־C   */

unsigned char xdata fgDownRs;     /* ���������´��л�*/
unsigned char xdata workState;    /* ��λ������״̬  */
unsigned char xdata heatState;    /* ��λ������״̬  */
unsigned char xdata warnState;    /* ״̬��          */
unsigned char xdata warnCount[9]; /* �����жϼ�����  */

unsigned char xdata voltageWarnValue;  /* �������ѹԤ��ֵA */
unsigned char xdata voltageWarnValueB; /* �������ѹԤ��ֵB */
unsigned char xdata voltageWarnValueC; /* �������ѹԤ��ֵC */

unsigned char xdata chargeEndValue;  /* �������ѹ�����ֹ����A */
unsigned char xdata chargeEndValueB; /* �������ѹ�����ֹ����B */
unsigned char xdata chargeEndValueC; /* �������ѹ�����ֹ����C */

unsigned char xdata healthState;   /* ����״̬��־ */
unsigned char xdata healthData[5]; /* ����״̬���� */

unsigned char xdata fgAhEn;  /* �����鰲ʱ��ʹ�ܱ�־A */
unsigned char xdata fgAhEnB; /* �����鰲ʱ��ʹ�ܱ�־B */
unsigned char xdata fgAhEnC; /* �����鰲ʱ��ʹ�ܱ�־C */

unsigned char xdata fgChargeEn;  /* ��������䱣��ʹ�ܱ�־A */
unsigned char xdata fgChargeEnB; /* ��������䱣��ʹ�ܱ�־B */
unsigned char xdata fgChargeEnC; /* ��������䱣��ʹ�ܱ�־C */

unsigned char xdata fgDischargeEn;  /* ��������ű���ʹ�ܱ�־A */
unsigned char xdata fgDischargeEnB; /* ��������ű���ʹ�ܱ�־B */
unsigned char xdata fgDischargeEnC; /* ��������ű���ʹ�ܱ�־C */

unsigned char xdata fgExceDischarge; /* ���ű���ָ��ͱ�־ */

unsigned char xdata fgAh;  /* ��ʱ�Ʊ�־A  */
unsigned char xdata fgAhB; /* ��ʱ�Ʊ�־B  */
unsigned char xdata fgAhC; /* ��ʱ�Ʊ�־C  */

unsigned char xdata fgTinyCurrent;  /* �����־A */
unsigned char xdata fgTinyCurrentB; /* �����־B */
unsigned char xdata fgTinyCurrentC; /* �����־C */

unsigned char xdata proportion;  /* �������ű�A */
unsigned char xdata proportionB; /* �������ű�B */
unsigned char xdata proportionC; /* �������ű�C */

unsigned char xdata fgChargeOnoffA; /* �����������ָ���־A */
unsigned char xdata fgChargeOnoffB; /* �����������ָ���־B */

float xdata currentPower; /* ��ǰ����ֵ */

float xdata chargePower;  /* ������ֵA  */
float xdata chargePowerB; /* ������ֵB  */
float xdata chargePowerC; /* ������ֵC  */

float xdata dischargePower;  /* �ŵ����ֵA */
float xdata dischargePowerB; /* �ŵ����ֵB */
float xdata dischargePowerC; /* �ŵ����ֵC */

unsigned char xdata powerSave[6];         /* �������ݱ�������    */
unsigned char xdata anData8[57];          /* ԭʼģ�������ݲɼ��� 8bit */
unsigned int xdata anData12[6];           /* ԭʼģ�������ݲɼ��� 12bit */
unsigned char xdata buildFrameBuffer[80]; /* �������֡���ݻ�����(Length\Title\��Ч����\Sum)  */
unsigned char xdata soonFrameA[49];       /* �ٱ�ң���������A  */
unsigned char xdata soonFrameB[49];       /* �ٱ�ң���������B  */
unsigned char xdata slowFrameA[104];      /* ����ң���������A  */
unsigned char xdata slowFrameB[104];      /* ����ң���������B  */

unsigned char xdata equEn;  /* �����������ʹ��A */
unsigned char xdata equEnB; /* �����������ʹ��B */
unsigned char xdata equEnC; /* �����������ʹ��C */

unsigned char xdata workEnState;        /* ����ʹ��״̬          */
unsigned char xdata equOutputState;     /* ����������״̬��    */
unsigned char xdata equOutputWord;      /* ����������������    */
unsigned char xdata equControlState[7]; /* ����������״̬      */
unsigned int xdata aq[7];               /* �����ѹֵ            */
unsigned int xdata aqpx[7];             /* ��������ѹֵ      */

unsigned char xdata canResetCount;     /* CAN�ȸ�λ����    */
unsigned int xdata wingTemperature[6]; /* �¶Ȳɼ�ֵ       */

unsigned char fgSRAMchk;      /* SRAM�Լ��־          */
unsigned char chkAddr;        /* SRAM��4K�Լ�λ�ñ�ǣ�ÿ���Լ�256�ֽ� */

unsigned int  AD558set;                      /* DA�������ֵA         */
unsigned int  AD558setB;                     /* DA�������ֵB         */
unsigned int  AD558setC;                     /* DA�������ֵC         */

unsigned char xdata fgAnCheck;        /* ģ�����Լ��־        */
unsigned int xdata anCount[6];        /* ģ�����Լ����        */
unsigned char xdata anCheckEn;        /* ģ�����Լ�ʹ�ܱ�־    */
unsigned char xdata anCheckEnB;       /* ģ�����Լ�ʹ�ܱ�־B   */
unsigned char xdata anCheckEnC;       /* ģ�����Լ�ʹ�ܱ�־C   */
unsigned int xdata chargeUnlockCount; /* ����������          */
unsigned char xdata fg2S;             /* 2s��ʱ���б�־        */
unsigned char xdata fg60s;            /* 60Sʱ�����ڶ�ʱ��־   */
unsigned char xdata fgPreWarn;        /* Ԥ����־λ   */
unsigned char xdata uploadData[8];    /* ��ע���ݿ�����(������0x55��ʶ��)  */

unsigned char xdata *pSend;             /* ���߷���ָ��   */
unsigned char sendIndex;                /* ���߷���֡����   */
unsigned char sendFrame;                /* ���߷������ݰ���֡��  */
unsigned char receiveFrame;             /* ���߽�������֡����  */
unsigned int receiveSum;                /* ���߽������ݰ��ۼӺ�  */
unsigned char receiveCount;             /* ���߽������ݼ���  */
unsigned char xdata receiveBuffer[256]; /* �������ݰ����ջ�����(wLength+Title+Data+Sum)  */

void SystemInit(void);                           /* ϵͳ��ʼ������       */
void DataInit(void);                             /* ���ݳ�ʼ������ */
void CANInit(unsigned int address);              /* SJA1000��ʼ������ */
void CPUInit(void);                              /* 80C32��ʼ������ */
void PowerInit(void);                            /* ����������ʼ������ */
void ANCollect(void);                            /* ģ�����ɼ����� */
unsigned char ADConvert8(unsigned char channel); /* AD 8bitת������ */
unsigned int ADConvert12(unsigned char channel); /* AD 12bitת������ */
void StateWordBuild(void);                       /* ��״̬�ֺ��� */
void DataSave(void);                             /* ң�����֡���� */
void HeatControl(void);                          /* �¶ȿ��ƺ����� */
void CanResume(void);                            /* CAN�ָ����� */
void PowerControl(void);                         /* ��ʱ�ƺ��� */
void ChargeProtect(void);                        /* ���䱣������ */
void DischargeProtect(void);                     /* ���ű������� */
void ONOFFOutput(unsigned char index);           /* ָ��������� */
void SafeWarn(void);                             /* ��ȫԤ������ */
void ONOFFHook(void);                            /* ָ��������Ӻ��� */
void DataLoad(void);                             /* ���ݿ鴦���� */

void FrameBuild(unsigned char *pBuildFrame, unsigned char frameIndex, unsigned char endDLC); /* ��֡���� */
unsigned char Get2_3_X(unsigned char *a, unsigned char *b, unsigned char *c);                /* ����3ȡ2�о����� */
unsigned char Get2_3_F(float *a, float *b, float *c);                                        /* ������3ȡ2�о����� */
unsigned char Get2_3_I(unsigned int *a, unsigned int *b, unsigned int *c);                   /* ������3ȡ2�о����� */
void Delay(unsigned int time);                                                               /* �̶���ʱ���� */
void DataProtect(void);                                                                      /* ���ݲ����������� */
void SRAMcheck(void);                                                                        /* SRAM�������Լ� */
void DAcheck(void);                                                                          /* DA�Լ캯�� */
void HeatCheck(void);                                                                        /* �����Լ캯�� */
void Ancheck(void);                                                                          /* ģ�����Լ캯�� */
void EquControlCut(void);                                                                    /* ������ƶϿ�����    */
void EquControlLink(void);                                                                   /* ������ƽ�ͨ����    */
void ChargeUnlock(void);                                                                     /* ���������� */

void CANRXD(void);      /* CAN�жϽ��մ����� */
void CANTXD(void);      /* CAN�жϷ��ʹ����� */
void RsManage(void);    /* CAN�ж���ѯ������ */
void ONOFFManage(void); /* CAN�ж�ָ����� */
void DataManage(void);  /* CAN�ж����ݿ鴦���� */


/*******************************************************
  ��������:  main
  ��������:  
  �޸ļ�¼:  
********************************************************/
void main(void)
{
  unsigned char clockCountTemp; /* ���������������ʱ���� */

  SystemInit(); /* ϵͳ��ʼ�� */

  while (1) /* ִ�й̶���������  1s */
  {
      dog = !dog;                      /* 1s ǣ�� */
      AD558 = (unsigned char)AD558set; /* ����������DA��� */

      if ((fgSRAMchk == TRUE) || (fgAnCheck == TRUE)) /* SRAM�Լ��쳣 */
      {
        XBYTE[BUS_ADDR] = 0x41; /* SJA1000���븴λģʽ  */
      }
      else /* SRAM�Լ�����ʱִ�й̶����� */
      {
        errorCount++;
        fg60s++;            /* 60S���ڶ�ʱ */
                            /* 1s�̶�ִ�к��� */
        DataProtect();      /* ���ݱ������� */
        ANCollect();        /* ģ�����ɼ� */
        StateWordBuild();   /* ����״̬�� */
        DataSave();         /* ��֡���� */
        HeatControl();      /* ���º��� */
        CanResume();        /* CAN�����쳣������ */
        ChargeProtect();    /* ���䱣�� */
        DischargeProtect(); /* ���ű��� */
        EquControlCut();    /* ������ƶϿ����� */

        if (fg2S == FALSE) /* 2s�߼���ת */
        {
          fg2S = TRUE;
          ChargeUnlock(); /* ���������� */
          Ancheck();      /* ģ�����Լ캯�� */
        }
        else /* 2s�̶�ִ�к��� */
        {
          fg2S = FALSE;
          DAcheck();      /* DA�Լ캯�� */
          PowerControl(); /* ��ʱ�ƿ��� */
          SafeWarn();     /* ��Դ��ȫģʽ */
        }

        if (fg60s >= 60) /* 60S���о������ */
        {
          EquControlLink(); /* ������ƽ�ͨ���� */
          fg60s = 0;
        }

        ONOFFHook(); /* ���ָ����� */
        DataLoad();
        SRAMcheck(); /* ��ɹ̶���������SRAM�������Լ� */
      }

      dog = !dog; /* 1s ǣ�� */

      clockCountTemp = clockCount;
      while (clockCountTemp < 100) /* ��ʱ�ȴ�1s */
      {
        clockCountTemp = clockCount;
      }
      clockCount = (unsigned char)(clockCount - 100); /* ʱ�䲹�� */
    }
}

/*******************************************************
  ��������:  SystemInit
  ��������:  
  �޸ļ�¼:  
********************************************************/
void SystemInit(void)
{
  dog = !dog;

  EA = 0; /* ���ж� */

  ONOFF_ADDR = 0xFF; /* ���ָ����FFH */
  onoffEn = 0;       /* ָ�����ʹ�� */
  onoffEn = 1;       /* ָ��������� */

  equControlEn = 0; /* �����������Ϊ�ߵ�ƽ  ȫ���Ͽ� */
  EQU_ADDR = 0;

  AD558 = AD558_VSET; /* ����DA���Ϊ����ֵ */

  AD558set = AD558_VSET;
  AD558setB = AD558_VSET;
  AD558setC = AD558_VSET;

  DataInit();  /* Ĭ�����ݳ�ʼ�� */
  HeatCheck(); /* �¿��Լ캯�� */
  PowerInit(); /* Ĭ�ϵ���������ʼ�� */

  ANCollect();      /* ģ�����ɼ� */
  StateWordBuild(); /* ����״̬�� */
  DataSave();       /* ��֡���� �������AB�� */
  DataSave();       /* ��֡���� */

  CPUInit();          /* 80C32��ʼ�� */
  CANInit(BUS_ADDR); /* CAN���߳�ʼ�� */

  EA = 1; /* ���ж�  */
}

/*******************************************************
  ��������:  DataInit
  ��������:  
  �޸ļ�¼:  
********************************************************/
void DataInit(void)
{
  unsigned int i;

  fgRstType = 1;      /* ����P1������״̬ */
  if (fgRstType == 1) /* �ж����临λ���ȸ�λ */
  {
    resetCount++;                                    /* �����������Լ� */
    resetCount = (unsigned char)(resetCount & 0x0F); /* ����������� */
    resetCountB = resetCount;                        /* B,C��ֵ */
    resetCountC = resetCount;
  }
  else
  {
    resetCount = 0;  /* ��������λ��������  */
    resetCountB = 0; /* B,C��ֵ */
    resetCountC = 0;
  }

  for (i = 0; i < 2; i++) /* �¿ػ�·������ʼ��   */
  {
    heatPara[i].Mode = heatTab[i][0]; /* A��Ĭ�ϲ��� */
    heatPara[i].Close = heatTab[i][1];
    heatPara[i].Open = heatTab[i][2];

    heatParaB[i].Mode = heatTab[i][0]; /* B��Ĭ�ϲ��� */
    heatParaB[i].Close = heatTab[i][1];
    heatParaB[i].Open = heatTab[i][2];

    heatParaC[i].Mode = heatTab[i][0]; /* C��Ĭ�ϲ��� */
    heatParaC[i].Close = heatTab[i][1];
    heatParaC[i].Open = heatTab[i][2];
  }

  voltageWarnValue = ACCUMULATOR_VSET;  /* ��ѹԤ��ֵ Aֵ */
  voltageWarnValueB = ACCUMULATOR_VSET; /* ��ѹԤ��ֵ Bֵ */
  voltageWarnValueC = ACCUMULATOR_VSET; /* ��ѹԤ��ֵ Cֵ */

  chargeEndValue = 150;  /* �������ѹ�����ֹ����   X=(W*2)/100  Ĭ��ֵ3 */
  chargeEndValueB = 150; /* �������ѹ�����ֹ���� B */
  chargeEndValueC = 150; /* �������ѹ�����ֹ���� C */

  chargeUnlockCount = 0; /* ���������� */

  clockCount = 0;  /* 1s�������ж�ʱ������ */
  errorCount = 0;  /* ����ͨѶ������Э�������  */
  errorCountA = 0; /* ����ͨѶ������Э�������  */

  onoffNumber = 0; /* ִ��ָ��� */
  onoffNumberB = 0;
  onoffNumberC = 0;

  rsCount = 0;          /* ң��ָ��ִ�м���(����ָ���ע����) */
  currentCount = 0;     /* ����30s������ */
  downHeatNumber = 0;   /* �´�����״̬��··�� */
  downHealthNumber = 0; /* �´�����״̬��··�� */

  fg60s = 0;         /* 60S���ڶ�ʱ */
  fg2S = FALSE;      /* 2S���ڶ�ʱ */
  fgSwitch = FALSE;  /* ���ݲɼ�����������־ */
  fgDownRs = FALSE;  /* Ĭ�Ͻ�ֹ�л� */
  fgSRAMchk = FALSE; /* Ĭ��SRAM�Լ����� */

  equEn = FALSE; /* ������Ƴ�ʼ״̬Ϊ��ֹ */
  equEnB = FALSE;
  equEnC = FALSE;

  fgDischargeEn = FALSE; /* Ĭ�Ϲ��ű������ܽ�ֹ */
  fgDischargeEnB = FALSE;
  fgDischargeEnC = FALSE;

  fgChargeEn = TRUE; /* Ĭ�Ϲ��䱣������ */
  fgChargeEnB = TRUE;
  fgChargeEnC = TRUE;

  fgChargeOnoffA = FALSE; /* Ĭ�ϴ����������ָ���־ */
  fgChargeOnoffB = FALSE;

  fgDataRequest = FALSE; /* ���������ݿ�Ҫ���־ */

  fgONOFFExecute = FALSE; /* ָ��ִ�б�־ */
  fgONOFFExecuteB = FALSE;
  fgONOFFExecuteC = FALSE;

  fgExceDischarge = FALSE; /* Ĭ���޹���״̬ */
  
  fgPreWarn = FALSE; /* Ԥ����־λ ��Ч  */

  workState = 0;   /* ����״̬�ֳ�ʼ������       ��ʼֵ������״̬�� */
  workEnState = 0; /* ����ʹ��״̬ D0=0 D1=0 D2=0 D3=1 D4=0   ��ʼֵ������״̬�� */

  healthState = 0; /* ����״̬��    */

  warnState = 0;      /* Ԥ��״̬������ */
  heatState = 0;      /* ����״̬������ */
  equOutputState = 0; /* ����������״̬��  7·�Ͽ�(Ĭ��)  �´���  ��ʼֵ������״̬�� */

  equOutputWord = 0; /* ����������������  7·�Ͽ�(Ĭ��)  ʵ�ʿ����� */
  chkAddr = 0;       /* SRAM��4K�Լ�λ�ó�ʼ��Ϊ0������0x1000λ���� */

  healthData[0] = 0xFF; /* ����״̬����1 Ĭ��0xFF */
  for (i = 1; i < 5; i++)
  {
    healthData[i] = 0; /* ����״̬����2-5 Ĭ��0 */
  }

  for (i = 0; i < 9; i++)
  {
    warnCount[i] = 0; /* �����жϼ����� */
  }

  for (i = 0; i < 7; i++) /* ����������״̬��ʼ����Ĭ�϶Ͽ�   FALSE �Ͽ�  ������1ͨ 0��  �´��ã�1�� 0ͨ */
  {
    equControlState[i] = FALSE;
  }

  fgAnCheck = FALSE; /* ģ�����Լ��־Ĭ��Ϊ���� */

  anCheckEn = FALSE; /* Ĭ�Ͻ�ֹģ�����Լ칦��  */
  anCheckEnB = FALSE;
  anCheckEnC = FALSE;

  for (i = 0; i < 6; i++) /* ģ�����Լ���� */
  {
    anCount[i] = 0; /* ģ������������ */
  }

  receiveSum = 0; /* ���߽������ݰ���������   */
  receiveCount = 0;
  receiveFrame = 0;
   for (i = 0; i < 256; i++)
   {
     receiveBuffer[i] = 0;
   }

   for (i = 0; i < 8; i++) /* ��ע���ݿ黺��������    */
   {
     uploadData[i] = 0;
   }

  canResetCount = 0; /* CAN��λ�������� */

   for (i = 0; i < 6; i++) /* �ɼ�ֵ���� */
   {
     wingTemperature[i] = 0;
   }
}

/*******************************************************
* ��������:  SRAM�Լ캯��
* ��ע: 
* 
*******************************************************/
void SRAMcheck(void)
{
  unsigned int i;
  unsigned char dataBuffer; /* SRAM�Լ��ݴ����ݻ����� */

  if ((fgAh == TRUE) || (fgAh == FALSE)) /* ��ʱ�Ʊ�־Ϊ����ֵ */
  {
    if ((fgTinyCurrent == TRUE) || (fgTinyCurrent == FALSE)) /* �����־Ϊ����ֵ */
    {
      fgSRAMchk = FALSE; /* ����ֵ��ΪSRAM�Լ���ȷ */
    }
    else /* �ǵ���ֵΪSRAM�쳣 */
    {
      fgSRAMchk = TRUE;
    }
  }
  else /* �ǵ���ֵΪSRAM�쳣 */
  {
    fgSRAMchk = TRUE;
  }

  if (fgSRAMchk == FALSE) /* SRAM�Լ���ȷʱִ��SRAM��д��� */
  {
    if (chkAddr >= 16) /* ��ⷶΧ 16 * 256 BYTE  4K  ��д����ǰ�ж���ַ��Ч��Χ */
    {
      chkAddr = 0; /* ѭ������	 */
    }

    for (i = 0; i < 256; i++) /* 256BYTE��� */
    {
      dataBuffer = XBYTE[0x1000 + (unsigned int)(256 * chkAddr) + i]; /* ��ǰ���ݱ��� */
      XBYTE[0x1000 + (unsigned int)(256 * chkAddr) + i] = 0xEB;       /* д��0xEB����д��ֵ */
      if (XBYTE[0x1000 + (unsigned int)(256 * chkAddr) + i] != 0xEB)  /* �����Ƚ� */
      {
        fgSRAMchk = TRUE; /* ���ݲ���ʱSRAM�쳣 */
        break;
      }
      else /* ������Ȼָ��������� */
      {
        XBYTE[0x1000 + (unsigned int)(256 * chkAddr) + i] = dataBuffer;
      }
    }
    chkAddr = (unsigned char)(chkAddr + 1); /* ����ַѭ���Լ� */
  }
}

/*******************************************************
* ��������:  D/A���ѭ�캯��
* ��ע: 
* 
*******************************************************/
void DAcheck(void)
{
  int result;

  result = abs(((int)anData8[15] * 2) - (int)AD558set);
  if (result < 11) /* D/A�ɼ�ֵ�����ֵ��10���ֲ�ֵ�� */
  {
    workState = (unsigned char)(workState & 0xFE); /* �����Լ�ɹ���־	 */
  }
  else
  {
    workState = (unsigned char)(workState | 0x01); /* �����Լ�ʧ�ܱ�־ */
  }
}

/*******************************************************
  ��������:  HeatCheck
  ��������:  
  �޸ļ�¼:  
********************************************************/
void HeatCheck(void)
{
  P1 = (unsigned char)(P1&0xFC);  /* �����¿����	 2·    */
  Delay(2000);                    /* ��ʱ�ȴ�Լ30ms  */
  ANCollect();                    /* �������״̬�ɼ� */
  StateWordBuild();               /* �������״̬�� */
  if ((heatState & 0x03) != 0x00) /* ״̬���ж�  ��ȫ�����ȣ� */
  {
    workState = (unsigned char)(workState | 0x40); /* �����Լ�ʧ�ܱ�־ */
  }

  P1 = (unsigned char)(P1|0x03); /* �ر��¿����	 2· */
  Delay(2000);                     /* ��ʱ�ȴ�Լ30ms  */
  ANCollect();                     /* �������״̬�ɼ� */
  StateWordBuild();                /* �������״̬�� */
  if ((heatState & 0x03) != 0x03)  /* ״̬���ж�  ��ȫ���رգ� */
  {
    workState = (unsigned char)(workState | 0x40); /* �����Լ�ʧ�ܱ�־ */
  }
}

/*******************************************************
* ��������: ģ�����Լ캯��
* ��ע:
* 
*******************************************************/
void Ancheck(void)
{
  unsigned char i;
  unsigned char xdata fgChk[6];

  if (anCheckEn == TRUE)
  {
    for (i = 0; i < 6; i++) /* 6�������ж���ʶ      */
    {
      fgChk[i] = FALSE;
    }

    if (anData8[11] < 0x96) /* 12V���ݵ�Դ��ѹ  96 - 3V */
    {
      anCount[0]++;

      if (anCount[0] >= 15) /* ����ʱ�����30s */
      {
        fgChk[0] = TRUE;
      }
    }
    else /* ���������� */
    {
      anCount[0] = 0;
      fgChk[0] = FALSE;
    }

    if (anData8[11] > 0xFA) /* 12V���ݵ�Դ��ѹ  FA - 5V */
    {
      anCount[1]++;
      if (anCount[1] >= 15) /* ����ʱ�����30s */
      {
        fgChk[1] = TRUE;
      }
    }
    else /* ���������� */
    {
      anCount[1] = 0;
      fgChk[1] = FALSE;
    }

    if (anData8[12] < 0x96) /* 12V���ݵ�Դ��ѹ 96 - 3V */
    {
      anCount[2]++;
      if (anCount[2] >= 15) /* ����ʱ�����30s */
      {
        fgChk[2] = TRUE;
      }
    }
    else /* ���������� */
    {
      anCount[2] = 0;
      fgChk[2] = FALSE;
    }

    if (anData8[12] > 0xFA) /* 12V���ݵ�Դ��ѹ FA - 5V */
    {
      anCount[3]++;
      if (anCount[3] >= 15) /* ����ʱ�����30s */
      {
        fgChk[3] = TRUE;
      }
    }
    else /* ���������� */
    {
      anCount[3] = 0;
      fgChk[3] = FALSE;
    }

    if (anData8[14] < 0x64) /* 5V��׼��Դ��ѹ 4V - 64  */
    {
      anCount[4]++;
      if (anCount[4] >= 15) /* ����ʱ�����30s */
      {
        fgChk[4] = TRUE;
      }
    }
    else /* ���������� */
    {
      anCount[4] = 0;
      fgChk[4] = FALSE;
    }

    if (anData8[14] > 0x96) /* 5V��׼��Դ��ѹ 6V - 96 */
    {
      anCount[5]++;
      if (anCount[5] >= 15) /* ����ʱ�����30s */
      {
        fgChk[5] = TRUE;
      }
    }
    else /* ���������� */
    {
      anCount[5] = 0;
      fgChk[5] = FALSE;
    }

    for (i = 0; i < 6; i++) /* ������������������ģ�����Լ��쳣��־ */
    {
      if (fgChk[i] == TRUE)
      {
        fgAnCheck = TRUE;
      }
    }
  }
}

/*******************************************************
  ��������:  CANInit
  ��������:  
  �޸ļ�¼:  
********************************************************/
void CANInit(unsigned int address)
{
  unsigned char temp;

  XBYTE[address] = 0x41; /* ���븴λģʽ    */
  Delay(5);
  XBYTE[address + 1] = 0xEC;    /* ����Ĵ������� */
  XBYTE[address + 4] = CAN_ACR; /* ���ܴ���Ĵ�������       */
  XBYTE[address + 5] = CAN_AMR; /* �������μĴ�������       */
  XBYTE[address + 6] = 0x41;    /* ����ʱ��Ĵ���0����      */
  XBYTE[address + 7] = 0xC5;    /* ����ʱ��Ĵ���1����      */
  XBYTE[address + 8] = 0x4A;    /* ������ƼĴ�������       */
  XBYTE[address + 31] = 0x04;   /* ʱ�ӷ�Ƶ�Ĵ�������       */

  XBYTE[address] = 0x46; /* �жϵĿ�������  */


  temp = XBYTE[address + 2]; /* ��״̬�Ĵ���  */
  temp = XBYTE[address + 3]; /* ���жϼĴ���   */
}

/*******************************************************
  ��������:  CPUInit
  ��������:  
  �޸ļ�¼:  
********************************************************/
void CPUInit(void)
{
  IP = 0x02; /* �������ȼ�: T0Ϊ�����ȼ���INT0Ϊ�����ȼ� */
  IT0 = 1;   /* ��ƽ�жϴ��� */

  TMOD = 0x01; /* ��ʱ��0�趨������ʽ1 */
  TH0 = 0xDC;
  TL0 = 0x00; /* ��ʱ��10ms��ʱ  */
  TR0 = 1;    /* ����T0���� */

  ET0 = 1; /* ����ʱ��0�ж� */
  EX0 = 1; /* �����ⲿ�ж�0�ж�   */
}

/*******************************************************
  ��������:  PowerInit
  ��������:  
  �޸ļ�¼:  
********************************************************/
void PowerInit(void)
{
  proportion = 130;  /* Ĭ�ϵ������Ƴ�ʼ����  (130 + 900)/1000=1.03 */
  proportionB = 130; /* Ĭ�ϳ�ű�1.03  */
  proportionC = 130;

  powerSave[0] = FULLPOWER_H; /* ��ǰ����150Ah  ���´�ֵ�� */
  powerSave[1] = FULLPOWER_L;
  powerSave[2] = 0; /* ������0Ah */
  powerSave[3] = 0;
  powerSave[4] = 0; /* �ŵ����0Ah */
  powerSave[5] = 0;

  currentPower = (float)FULLPOWER; /* ��ǰ����������ʼֵ ���������ʹ��ֵ�� */

  chargePower = (float)0; /* ������������ʼֵ */
  chargePowerB = (float)0;
  chargePowerC = (float)0;

  dischargePower = (float)0; /* �ŵ����������ʼֵ */
  dischargePowerB = (float)0;
  dischargePowerC = (float)0;

  fgAhEn = TRUE; /* Ĭ�ϰ�ʱ������״̬  */
  fgAhEnB = TRUE;
  fgAhEnC = TRUE;

  fgAh = FALSE; /* ���ð�ʱ�ƿ��Ʊ�־(��ʾ0) */
  fgAhB = FALSE;
  fgAhC = FALSE;

  fgTinyCurrent = FALSE; /* ���������־(��ʾ0) */
  fgTinyCurrentB = FALSE;
  fgTinyCurrentC = FALSE;
}


/*******************************************************
  ��������:  ANCollect
  ��������:  
  �޸ļ�¼:  
********************************************************/
void ANCollect(void)
{
  unsigned char i;
  unsigned char j;
  unsigned int xdata temp;
  unsigned long xdata sum; /* �����������ƽ��ֵ */

  for (i = 0; i < 57; i++) /* �ɼ�ȫ��ң�����(8bit) */
  {
    anData8[i] = ADConvert8(rsTab8[i]);
  }

  for (i = 0; i < 6; i++) /* �¶Ȳɼ� (12bit) */
  {
    wingTemperature[i] = ADConvert12(rsTab8[44 + i]);
  }

  for (i = 0; i < 6; i++) /* ����20�βɼ� (12bit) */
  {
    sum = 0; /* �ۼӺ����� */
    for (j = 0; j < 20; j++)
    {
      sum = sum + ADConvert12(rsTab12[i]); /* ����20���ۼӺ� */
    }
    anData12[i] = (unsigned int)((float)sum / 20.0); 
  }

  for (i = 0; i < 7; i++) /
  {
    aq[i] = ADConvert12(rsTab8[29 + i]);
    aq[i] = aq[i] >> 2; /* D11 ~ D2	 */
    aqpx[i] = aq[i];
  }

  for (i = 0; i < 6; i++) /* ð������ */
  {
    for (j = 1; j < (unsigned char)(7 - i); j++)
    {
      if (aqpx[i] > aqpx[i + j]) /* ���� */
      {
        temp = aqpx[i + j];
        aqpx[i + j] = aqpx[i];
        aqpx[i] = temp;
      }
    }
  }
}

/*******************************************************
  ��������:  ADConvert8
  ��������:  ʵ��ģ����������ת���ɼ�����
  �޸ļ�¼:  
********************************************************/
unsigned char ADConvert8(unsigned char channel)
{
  unsigned char i;
  unsigned char j;
  unsigned char temp;
  unsigned char ad[3]; /* �м�ֵ�˲��洢 */

  AN_ADDR = channel; /* ��ģ����ͨ�� */
  Delay(25);         /* ��ʱ�ȴ�ͨ���� */
  for (i = 0; i < 3; i++)
  {
    AD574H = 0;     /* A/Dת������ */
    Delay(3);       /* A/Dת����ʱ */
    ad[i] = AD574H; /* ȡBit11-Bit4  20mv */
  }

  for (i = 0; i < 2; i++) /* ð�ݷ�����  */
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

  return (ad[1]); /* �����м�ֵ */
}

/*******************************************************
  ��������:  ADConvert12
  ��������:  
  �޸ļ�¼:  
********************************************************/
unsigned int ADConvert12(unsigned char channel)
{
  unsigned char i;
  unsigned char j;
  unsigned int temp;
  unsigned int xdata ad12[3]; /* �м�ֵ�˲��洢 */
  unsigned char HData;
  unsigned char LData;

  AN_ADDR = channel; /* ��ģ����ͨ�� */
  Delay(25);         /* ��ʱ�ȴ�ͨ���� */
  for (i = 0; i < 3; i++)
  {
    AD574H = 0; /* A/Dת������ */
    Delay(3);   /* A/Dת����ʱ */
    HData = AD574H;
    LData = AD574L;

    ad12[i] = (unsigned int)((unsigned int)((unsigned int)((unsigned int)HData << 4) & 0x0FF0) | (unsigned int)((unsigned int)((unsigned int)LData >> 4) & 0x000F));
  }

  for (i = 0; i < 2; i++) /* ð�ݷ�����  */
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

  return (ad12[1]); /* �����м�ֵ */
}

/*******************************************************
  ��������:  StateWordBuild
  ��������: 
  �޸ļ�¼:  
********************************************************/
void StateWordBuild(void)
{
  unsigned char i;
  unsigned char xdata *p;

  if (CBYTE[0x3FFF] == TRUE) /* ������ʹ��״̬ */
  {
    workState = (unsigned char)(workState & 0xFD); /* CPUʹ��״̬: A�� */
  }
  else
  {
    workState = (unsigned char)(workState | 0x02); /* CPUʹ��״̬: B�� */
  }

  if (fgAh == TRUE) /* �жϰ�ʱ�Ʊ�־(D4) */
  {
    workState = (unsigned char)(workState | 0x10);
  }
  else
  {
    workState = (unsigned char)(workState & 0xEF);
  }

  if (fgTinyCurrent == TRUE) /* �ж������־(D5) */
  {
    workState = (unsigned char)(workState | 0x20);
  }
  else
  {
    workState = (unsigned char)(workState & 0xDF);
  }
  /* D6Ϊ���»�·�Լ�λ */
  if (fgPreWarn == TRUE) /* Ԥ����־(D7) */
  {
    workState = (unsigned char)(workState | 0x80);
  }
  else
  {
    workState = (unsigned char)(workState & 0x7F);
  }

  heatState = 0;          /* ����״̬���� */
  p = &anData8[51];       /* ��Դ��λ������״̬  */
  for (i = 0; i < 6; i++) /* ָ�����ͨ��״̬�ɼ����� */
  {
    if (*p > 125)
    {
      heatState = (unsigned char)(heatState | 0x40);
    }
    heatState = (unsigned char)(heatState >> 1);
    p++;
  }

  equOutputState = 0; /* ������� */
  for (i = 0; i < 7; i++)
  {
    if (equControlState[i] == FALSE) /* ��Ϊ�Ͽ�״̬ */
    {
      equOutputState = (unsigned char)(equOutputState | 0x80); /* ����D7 */
    }
    equOutputState = (unsigned char)(equOutputState >> 1); /* ѭ������ */
  }

  if (fgAhEn == FALSE) /* D0 ��ʱ��״̬ */
  {
    workEnState = (unsigned char)(workEnState | 0x01); /* ��ʱ�ƽ�ֹ */
  }
  else
  {
    workEnState = (unsigned char)(workEnState & 0xFE); /* ��ʱ������ */
  }

  if (equEn == TRUE) /* D1 ����״̬ */
  {
    workEnState = (unsigned char)(workEnState | 0x02); /* �����������  */
  }
  else
  {
    workEnState = (unsigned char)(workEnState & 0xFD); /* ������ƽ�ֹ */
  }
  if (fgDischargeEn == FALSE) /* D3 ���ű���״̬  */
  {
    workEnState = (unsigned char)(workEnState & 0xF7); /* ���ű��������� */
  }
  else
  {
    workEnState = (unsigned char)(workEnState | 0x08);
  }
  /* D4 ���ű�־ (���ű���ģ�������ã����������) */
  if (fgChargeEn == FALSE) /* D5 ���䱣��״̬  */
  {
    workEnState = (unsigned char)(workEnState | 0x20); /* ���䱣����ֹ */
  }
  else
  {
    workEnState = (unsigned char)(workEnState & 0xDF); /* ���䱣������ */
  }

  if (anCheckEn == FALSE) /* D6 ģ�����Լ�ʹ�ܱ�־ */
  {
    workEnState = (unsigned char)(workEnState & 0xBF); /* 0 ��ֹ */
  }
  else
  {
    workEnState = (unsigned char)(workEnState | 0x40); /* 1 ���� */
  }
}

/*******************************************************
  ��������:  DataSave
  ��������:  
  �޸ļ�¼:  
********************************************************/
void DataSave(void)
{
  unsigned char i;
  unsigned char xdata *pBuffer; /* ���ݴ洢��ָ�붨�� */
  unsigned char xdata *pSoon;
  unsigned char xdata *pSlow;

  unsigned char xdata rsSoonSum; /* �ٱ�ң�����SUM */
  unsigned char xdata rsSlowSum; /* ����ң�����SUM */

  rsSoonSum = 0x23; /* �ٱ�ң�����Title */
  rsSlowSum = 0x43; /* ����ң�����Title */

  if (fgSwitch == FALSE) /* �ж��л���־�������� */
  {
    pSoon = &soonFrameA[0]; /* A����ֵ */
    pSlow = &slowFrameA[0];
  }
  else /* B����ֵ */
  {
    pSoon = &soonFrameB[0];
    pSlow = &slowFrameB[0];
  }

  pBuffer = &buildFrameBuffer[0]; /* �ٱ�������� */
  *pBuffer = 0x1F;                /* �洢Length */
  pBuffer++;
  *pBuffer = 0x23; /* �洢Title  */
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

  for (i = 2; i < 33; i++) /* �����ٱ��ۼӺ� */
  {
    rsSoonSum = (unsigned char)(rsSoonSum + buildFrameBuffer[i]);
  }
  *pBuffer = rsSoonSum; /* �ٱ��ۼӺ͸�ֵ */

  FrameBuild(pSoon, 5, 7); /* ���������ٱ�ң�����(5֡) */

  if (fgDownRs == TRUE) /* �´�����״̬�����л� */
  {
    downHeatNumber++; /* �����´��¿ز���  */
    downHealthNumber++;

    if (downHeatNumber > 2) /* ���²���ѭ���л� 0,1 ���²��� 2 ����������ֹ���� */
    {
      downHeatNumber = 0; /* 0-1 ��Ч��Χ */
    }
    if (downHealthNumber > 4) /* ����״̬�����л� */
    {
      downHealthNumber = 0; /* 0-4 ��Ч��Χ */
    }
    fgDownRs = FALSE; /* ����ȿ��´���־    */
  }

  pBuffer = buildFrameBuffer; /* ���ɻ���ң��֡ */
  *pBuffer = 0x44;            /* �洢Length */
  pBuffer++;
  *pBuffer = 0x43; /* �洢Title  */
  pBuffer++;

  *pBuffer = healthState; /* W0 ��λ��������־ */
  pBuffer++;
  *pBuffer = healthData[downHealthNumber]; /* W1 ��λ������״̬���� */
  pBuffer++;
  *pBuffer = warnState; /* W2 ��λ��Σ�ձ�־ */
  pBuffer++;

  for (i = 5; i < 29; i++) /* W3-W26 ���������ֵ */
  {
    *pBuffer = anData8[i];
    pBuffer++;
  }

  for (i = 3; i < 6; i++) /* W27-W32 �¶�1~3 */
  {
    *pBuffer = (unsigned char)(wingTemperature[i] >> 10); 
    pBuffer++;
    *pBuffer = (unsigned char)(wingTemperature[i] >> 2);
    pBuffer++;
  }

  *pBuffer = (unsigned char)canResetCount; /* W33 ���߸�λ����  */
  pBuffer++;

  for (i = 36; i < 44; i++) /* W34-W41 ���������ֵ */
  {
    *pBuffer = anData8[i];
    pBuffer++;
  }

  for (i = 0; i < 3; i++) /* W42-W47 ̫�����¶�4~6 */
  {
    *pBuffer = (unsigned char)(wingTemperature[i] >> 10); 
    pBuffer++;
    *pBuffer = (unsigned char)(wingTemperature[i] >> 2);
    pBuffer++;
  }

  for (i = 0; i < 6; i++) /* W48-W53 �����鵱ǰ���� */
  {
    *pBuffer = powerSave[i];
    pBuffer++;
  }

  *pBuffer = anData8[50]; /* W55 ��ű� */
   pBuffer++;
  *pBuffer = proportion; /* W55 ��籣��״̬ */
  pBuffer++;

  if (downHeatNumber < 2)
  {
    *pBuffer = (unsigned char)(downHeatNumber + 1); /* W56 ����·�� */
    pBuffer++;
    *pBuffer = heatPara[downHeatNumber].Mode; /* W57 ����ģʽ */
    pBuffer++;
    *pBuffer = heatPara[downHeatNumber].Close; /* W58 �ر����� */
    pBuffer++;
    *pBuffer = heatPara[downHeatNumber].Open; /* W59 �������� */
    pBuffer++;
  }
  else
  {
    *pBuffer = chargeEndValue; /* W56 ����������ֹ���� */
    pBuffer++;
    *pBuffer = 0x00; /* W57 00H */
    pBuffer++;
    *pBuffer = 0x00; /* W58 00H */
    pBuffer++;
    *pBuffer = 0x00; /* W59 00H */
    pBuffer++;
  }

  *pBuffer = XBYTE[BUS_ADDR + 2]; /* W60 ���߿�����״̬�� */
  pBuffer++;
  *pBuffer = 0xAA; /* W61 Ԥ�� */
  pBuffer++;
  *pBuffer = (unsigned char)((unsigned char)(rsCount << 4) + resetCount); /* W62 ��λ�������� */
  pBuffer++;
  *pBuffer = workState; /* W63 ��λ��״̬ */
  pBuffer++;
  *pBuffer = heatState; /* W64 ����״̬ */
  pBuffer++;
  *pBuffer = equOutputState; /* W65 ����״̬ */
  pBuffer++;
  *pBuffer = workEnState; /* W66 ���ʹ��״̬ */
  pBuffer++;
  *pBuffer = voltageWarnValue; /* W67 �������ѹ�������� */
  pBuffer++;

  for (i = 2; i < 70; i++) /* ���㻺������ۼӺ� */
  {
    rsSlowSum = (unsigned char)(rsSlowSum + buildFrameBuffer[i]);
  }
  *pBuffer = rsSlowSum;

  FrameBuild(pSlow, 11, 2); /* ������֡��11֡�� */

  if (fgSwitch == FALSE) /* ������־ת�� */
  {
    fgSwitch = TRUE;
  }
  else
  {
    fgSwitch = FALSE;
  }
}

/*******************************************************
  ��������:  FrameBuild
  ��������:  
  �޸ļ�¼:  
********************************************************/
void FrameBuild(unsigned char *pBuildFrame, unsigned char frameIndex, unsigned char endDLC)
{
  unsigned char i;
  unsigned char j;
  unsigned char xdata *p;

  p = &buildFrameBuffer[0];

  for (i = 0; i < (unsigned char)(frameIndex - 1); i++) /* ��ʼ֡,�м�֡���� */
  {
    *pBuildFrame = 0x86; /* ���ֽ� */
    pBuildFrame++;
    *pBuildFrame = 0x68; /* ���ֽ� */
    pBuildFrame++;
    *pBuildFrame = i; /* ֡���� */
    pBuildFrame++;
    for (j = 0; j < 7; j++) /* ��Ч���� */
    {
      *pBuildFrame = *p;
      pBuildFrame++;
      p++;
    }
  }

  *pBuildFrame = 0x86; /* ���ɽ���֡ */
  pBuildFrame++;
  *pBuildFrame = (unsigned char)(0x60 + endDLC); /* ����֡���ݳ����� */
  pBuildFrame++;
  *pBuildFrame = (unsigned char)(frameIndex - 1); /* ֡���� */
  pBuildFrame++;
  for (i = 0; i < (unsigned char)(endDLC - 1); i++) /* ����֡��Ч���� */
  {
    *pBuildFrame = *p;
    pBuildFrame++;
    p++;
  }
}

/*******************************************************
  ��������:  HeatControl
  ��������:  
  �޸ļ�¼:  
********************************************************/
void HeatControl(void)
{
  unsigned char i;
  unsigned char j;
  unsigned char temp;                 /* ����ð�ݷ�����ʱ���� */
  float temperature;                  /* ʵ���¶�ֵ  */
  unsigned char xdata heatParaTHT[7]; /* 7�������������������� */

  for (i = 0; i < 7; i++) /* �������踶��ֵ */
  {
    heatParaTH[i] = anData8[37 + i];
    heatParaTHT[i] = heatParaTH[i];
  }

  for (i = 0; i < 6; i++) /* ��С����������� */
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

  for (i = 0; i < 2; i++) /* ��·���� */
  {
    if ((heatPara[i].Mode & 0xC0) == 0x40) /* �ж��¿�ģʽΪ�ջ�     */
    {
      if ((heatPara[i].Mode & 0x10) == 0x10) /* �жϰ���D3D2D1D0λѡȡ���µ��� */
      {
        temperature = (float)heatParaTH[(heatPara[i].Mode & 0x07) - 1];
      }
        temperature = (float)((float)((unsigned int)heatParaTHT[2] + (unsigned int)heatParaTHT[3] + (unsigned int)heatParaTHT[4]) / 3.0); /* ѡ���м�����ƽ��ֵ */

      if (temperature < (float)heatPara[i].Close)
      {
        P1 = (unsigned char)(P1|(_crol_(0x01,i))); /* ����ѭ����λ ���  �رռ��� */
      }
      else if (temperature > (float)heatPara[i].Open)
      {
        P1 = (unsigned char)(P1&(_crol_(0xFE,i))); /* ����ѭ����λ ����  �������� */
      }
      else
      {
        _nop_();
      }
    }
    else if ((heatPara[i].Mode & 0xC0) == 0x80) /* �ж��¿�ģʽΪ���� */
    {
      if ((heatPara[i].Mode & 0x20) == 0x20) /* �ж�Ϊ������������ */
      {
        P1 = (unsigned char)(P1&(_crol_(0xFE,i))); /* ����ѭ����λ ����  ��������    */
      }
      else /* �ж�Ϊ����ֹͣ����   */
      {
        P1 = (unsigned char)(P1|(_crol_(0x01,i)));  /* ����ѭ����λ ���  �رռ��� */
      }
    }
    else
    {
      _nop_();
    }
  }
}

/*******************************************************
  ��������:  DataProtect
  ��������:  
  �޸ļ�¼:  
********************************************************/
void DataProtect(void)
{
  unsigned char i;
  unsigned char result1;
  unsigned char result2;
  unsigned char result3;

  result1 = Get2_3_X(&proportion, &proportionB, &proportionC); /* ��űȡ���硢�ŵ����3ȡ2�о� */
  result2 = Get2_3_F(&chargePower, &chargePowerB, &chargePowerC);
  result3 = Get2_3_F(&dischargePower, &dischargePowerB, &dischargePowerC);
  if ((result1 == FALSE) || (result2 == FALSE) || (result3 == FALSE))
  {
    PowerInit(); /* �ָ�ȫ������ ������űȲ��� */
  }

  result1 = Get2_3_X(&voltageWarnValue, &voltageWarnValueB, &voltageWarnValueC); /* �������ѹ3ȡ2�о� */
  if (result1 == FALSE)
  {
    voltageWarnValue = ACCUMULATOR_VSET; /* ��ʼ��Ĭ�ϲ��� */
    voltageWarnValueB = ACCUMULATOR_VSET;
    voltageWarnValueC = ACCUMULATOR_VSET;
  }

  result1 = Get2_3_X(&resetCount, &resetCountB, &resetCountC); /* �ȸ�λ����3ȡ2�о� */
  if (result1 == FALSE)
  {
    resetCount = 0; /* ��ʼ��Ĭ�ϲ���  ���� */
    resetCountB = 0;
    resetCountC = 0;
  }

  result1 = Get2_3_X(&chargeEndValue, &chargeEndValueB, &chargeEndValueC); /* ����������ֹ����ֵ3ȡ2�о� */
  if (result1 == FALSE)
  {
    chargeEndValue = 150; /* ��ʼ��Ĭ�ϲ���  3 (150) */
    chargeEndValueB = 150;
    chargeEndValueC = 150;
  }

  for (i = 0; i < 2; i++) /* 2·���²������� */
  {
    result1 = Get2_3_X(&heatPara[i].Mode, &heatParaB[i].Mode, &heatParaC[i].Mode); /* ���²���3ȡ2�о� */
    result2 = Get2_3_X(&heatPara[i].Close, &heatParaB[i].Close, &heatParaC[i].Close);
    result3 = Get2_3_X(&heatPara[i].Open, &heatParaB[i].Open, &heatParaC[i].Open);
    if ((result1 == FALSE) || (result2 == FALSE) || (result3 == FALSE))
    {
      heatPara[i].Mode = heatTab[i][0]; /* ʹ��Ĭ�Ͽ��²��� */
      heatPara[i].Close = heatTab[i][1];
      heatPara[i].Open = heatTab[i][2];

      heatParaB[i].Mode = heatTab[i][0]; /* ͬʱ���ò���B��C */
      heatParaB[i].Close = heatTab[i][1];
      heatParaB[i].Open = heatTab[i][2];

      heatParaC[i].Mode = heatTab[i][0];
      heatParaC[i].Close = heatTab[i][1];
      heatParaC[i].Open = heatTab[i][2];
    }
  }

  result1 = Get2_3_X(&fgAhEn, &fgAhEnB, &fgAhEnC); /* ��ʱ��ʹ�ܱ�־�о�����  */
  if (result1 == FALSE)
  {
    fgAhEn = TRUE;
    fgAhEnB = TRUE;
    fgAhEnC = TRUE;
  }

  result1 = Get2_3_X(&fgChargeEn, &fgChargeEnB, &fgChargeEnC); /* ��籣��ʹ�ܱ�־�о�����   */
  if (result1 == FALSE)
  {
    fgChargeEn = TRUE;
    fgChargeEnB = TRUE;
    fgChargeEnC = TRUE;
  }

  result1 = Get2_3_X(&fgDischargeEn, &fgDischargeEnB, &fgDischargeEnC); /* ���ű���ʹ�ܱ�־�о����� */
  if (result1 == FALSE)
  {
    fgDischargeEn = FALSE;
    fgDischargeEnB = FALSE;
    fgDischargeEnC = FALSE;
  }

  result1 = Get2_3_X(&fgAh, &fgAhB, &fgAhC); /* ��ʱ�Ʊ�־�о�����   */
  if (result1 == FALSE)
  {
    fgAh = FALSE;
    fgAhB = FALSE;
    fgAhC = FALSE;
  }

  result1 = Get2_3_X(&fgTinyCurrent, &fgTinyCurrentB, &fgTinyCurrentC); /* �����־�о����� */
  if (result1 == FALSE)
  {
    fgTinyCurrent = FALSE;
    fgTinyCurrentB = FALSE;
    fgTinyCurrentC = FALSE;
  }

  result1 = Get2_3_X(&equEn, &equEnB, &equEnC); /* ����ʹ�ܱ�־�о�����  */
  if (result1 == FALSE)
  {
    equEn = FALSE;
    equEnB = FALSE;
    equEnC = FALSE;
  }

  result1 = Get2_3_I(&AD558set, &AD558setB, &AD558setC); /* DA����о����� */
  if (result1 == FALSE)
  {
    AD558set = AD558_VSET;
    AD558setB = AD558_VSET;
    AD558setC = AD558_VSET;
  }

  result1 = Get2_3_X(&anCheckEn, &anCheckEnB, &anCheckEnC); /* ģ�����Լ�ʹ�ܱ�־�о����� */
  if (result1 == FALSE)
  {
    anCheckEn = FALSE;
    anCheckEnB = FALSE;
    anCheckEnC = FALSE;
  }

  result1 = Get2_3_X(&onoffNumber, &onoffNumberB, &onoffNumberC);          /* ִ��ָ����о�����   */
  result2 = Get2_3_X(&fgONOFFExecute, &fgONOFFExecuteB, &fgONOFFExecuteC); /* ָ��ִ�б�־�о����� */

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
  ��������:  CanResume
  ��������:  
  �޸ļ�¼:  
********************************************************/
void CanResume(void)
{
  unsigned char aSR;
  unsigned char aCR;

  aCR = XBYTE[BUS_ADDR];     /* ������SJA1000������ */
  aSR = XBYTE[BUS_ADDR + 2]; /* ������SJA1000״̬�� */

  if ((aSR & 0x02) == 0x02) /* �ж����߿������������ */
  {
    EX0 = 0;            	/* ���ӹ��жϲ���   */
    CANInit(BUS_ADDR);		/* �Կ��������³�ʼ�� */

    errorCount = 0;
    receiveSum = 0; 		/* ���߽������ݰ���������    */
    receiveCount = 0;
    receiveFrame = 0;

    canResetCount++;        /* CAN��λ����++    */
    EX0 = 1; 				/* ���ӿ��жϲ���   */
  }

  if (errorCount > 110) 	/* ������Э��ʱ�䳬��  110 +- 10 s */
  {
    EX0 = 0;            	/* ���ӹ��жϲ���   */
    CANInit(BUS_ADDR); 		/* �����߿��������³�ʼ�� */
    errorCount = 0;

    receiveSum = 0; 		/* ���߽������ݰ���������     */
    receiveCount = 0;
    receiveFrame = 0;

    canResetCount++;        /* CAN��λ����++    */
    EX0 = 1; 				/* ���ӿ��жϲ���   */
  }

  if (((aSR & 0x80) == 0x80) || ((aCR & 0x02) == 0x00)) /* �ж����߹ر� �� ������������Ч */
  {
    errorCountA++; 			/* ���߼���+1 */
    if (errorCountA > 15) 	/* ����16�� */
    {
      EX0 = 0; 				/* ���ӹ��жϲ���   */

      CANInit(BUS_ADDR); 	/* �����߿��������³�ʼ�� */

      errorCountA = 0;
      receiveSum = 0; 		/* ���߽������ݰ���������    */
      receiveCount = 0;
      receiveFrame = 0;

      canResetCount++;      /* CAN��λ����++    */
      EX0 = 1;
    }
  }
}

/*******************************************************
  ��������:  PowerControl
  ��������:  
  �޸ļ�¼:  
********************************************************/
void PowerControl(void)
{
  float xdata chargeCurrent; /* ������ʱ����	 */
  float xdata dischargeCurrent;
  float xdata currentPhalanx;
  float xdata currentPhalanx1;
  float xdata currentPhalanx2;
  float xdata currentPhalanx3;
  float xdata currentLoad;

  if (fgAhEn == TRUE)
  {
    chargeCurrent = (float)((K1 * (float)anData12[4] * 0.00125) + B1);     /* ����������ֵ����  */
    dischargeCurrent = (float)((K2 * (float)anData12[5] * 0.00125) + B2);  /* �ŵ��������ֵ����	 */
    currentLoad = (float)((K4 * (float)anData12[0] * 0.00125) + B4);       /* ���ص��� */
    currentPhalanx1 = (float)((K31 * (float)anData12[1] * 0.00125) + B31); /* ����1 */
    currentPhalanx2 = (float)((K32 * (float)anData12[2] * 0.00125) + B32); /* ����2 */
    currentPhalanx3 = (float)((K33 * (float)anData12[3] * 0.00125) + B33); /* ����3 */
    currentPhalanx = currentPhalanx1 + currentPhalanx2 + currentPhalanx3;  /* ����֮�� */

    if (((currentPhalanx - currentLoad) >= (float)((chargeEndValue / 50.0) + 0.5)) || (chargeCurrent <= (float)(chargeEndValue / 50.0))) 
    {
      currentCount++;

      if (currentCount >= 15) /* ����ʱ����ڵ���30s */
      {
        currentCount = 0;

        fgAh = TRUE;
        fgAhB = TRUE;
        fgAhC = TRUE;

        fgTinyCurrent = TRUE;
        fgTinyCurrentB = TRUE;
        fgTinyCurrentC = TRUE;

        currentPower = (float)150; /* ��ǰ������150Ah */

        chargePower = (float)0;    /* ��硢�ŵ������0Ah */
        chargePowerB = (float)0;
        chargePowerC = (float)0;

        dischargePower = (float)0; /* ͬʱ���ò���A��B��C */
        dischargePowerB = (float)0;
        dischargePowerC = (float)0;

        if (fgChargeOnoffA == FALSE) /* ���㷢������ */
        {
          ONOFFOutput(6);        /* ���ͳ�����ָ��  */
          fgChargeOnoffA = TRUE; /* ��ת��־ */
        }
      }
    }
    else
    {
      
      currentCount = 0;
     fgChargeOnoffA = FALSE; /* ����δ��������ʱ��־ */
    }

    if ((fgAh == TRUE) && (dischargeCurrent > 1))
    {
      fgTinyCurrent = FALSE;
      fgTinyCurrentB = FALSE;
      fgTinyCurrentC = FALSE;
    }

    if ((fgAh == TRUE) && (fgTinyCurrent == FALSE)) /* ��ʱ�Ʊ�־Ϊ1�������־Ϊ0 */
    {
      if (chargeCurrent > 0.1) /* �������������0.1A */
      {
        chargePower = chargePower + ((float)(chargeCurrent * 10) / (float)(18 * (proportion + 900)));   /* ����������A */
        chargePowerB = chargePowerB + ((float)(chargeCurrent * 10) / (float)(18 * (proportion + 900))); /* ����������B */
        chargePowerC = chargePowerC + ((float)(chargeCurrent * 10) / (float)(18 * (proportion + 900))); /* ����������C */
      }
      if (dischargeCurrent > 0.3) /* ����ŵ��������0.3A */
      {
        dischargePower = dischargePower + (dischargeCurrent / 1800);   /* �ŵ��������A */
        dischargePowerB = dischargePowerB + (dischargeCurrent / 1800); /* �ŵ��������B */
        dischargePowerC = dischargePowerC + (dischargeCurrent / 1800); /* �ŵ��������C */
      }
      currentPower = 150 + chargePower - dischargePower; /* ��ǰ�������� */

      if (chargePower >= dischargePower) /* ���������ڵ��ڷŵ�������س��� */
      {
        fgTinyCurrent = TRUE;
        fgTinyCurrentB = TRUE;
        fgTinyCurrentC = TRUE;

        currentPower = (float)150; /* ����������	 */

        chargePower = (float)0;
        chargePowerB = (float)0;
        chargePowerC = (float)0;

        dischargePower = (float)0;
        dischargePowerB = (float)0;
        dischargePowerC = (float)0;

        if (fgChargeOnoffB == FALSE) /* ���㷢������ */
        {
          ONOFFOutput(6);        /* ���ͳ�����ָ��  */
          fgChargeOnoffB = TRUE; /* ��ת��־ */
        }
      }
      else
      {
        fgChargeOnoffB = FALSE; /* ���ò�����������־ */
      }
    }

    if (currentPower <= 0) /* �����ǰ����ֵ���ָ�ֵ */
    {
      currentPower = (float)0; /* ��ǰ����ά��0Ah */
    }

    if (dischargePower >= 196) /* ����ŵ��������196Ah */
    {
      dischargePower = dischargePower - chargePower; /* �ŵ����ȡ�ŵ�������������ֵ */
      chargePower = (float)0;                        /* ���������� */

      chargePowerB = chargePower;
      chargePowerC = chargePower;
      dischargePowerB = dischargePower;
      dischargePowerC = dischargePower;
    }

    powerSave[0] = (unsigned char)((unsigned int)((currentPower * 1000) / 3.0) >> 8);     /* ȡ��ǰ�����ĸ��ֽ� */
    powerSave[1] = (unsigned char)((unsigned int)((currentPower * 1000) / 3.0) & 0xFF);   /* ȡ��ǰ�����ĵ��ֽ� */
    powerSave[2] = (unsigned char)((unsigned int)((chargePower * 1000) / 3.0) >> 8);      /* ȡ�������ĸ��ֽ� */
    powerSave[3] = (unsigned char)((unsigned int)((chargePower * 1000) / 3.0) & 0xFF);    /* ȡ�������ĵ��ֽ� */
    powerSave[4] = (unsigned char)((unsigned int)((dischargePower * 1000) / 3.0) >> 8);   /* ȡ�ŵ�����ĸ��ֽ� */
    powerSave[5] = (unsigned char)((unsigned int)((dischargePower * 1000) / 3.0) & 0xFF); /* ȡ�ŵ�����ĵ��ֽ� */
  }
}

/*******************************************************
  ��������:  chargeProtect
  ��������:  ���䱣�� 
  �޸ļ�¼: 
********************************************************/
void ChargeProtect(void)
{
  unsigned char xdata flag;
  float xdata batteryVoltage;

  if (fgChargeEn == TRUE)
  {
    flag = FALSE; /* ���ó�ʼ״̬ Ĭ���޹��� */

    batteryVoltage = (float)((K5 * (float)anData8[1] * 0.02) + B5); /* �������ѹ����ֵ���� */

    if (batteryVoltage >= 30.1) /* �������ѹ>=30.1  */
    {
      warnCount[4]++;         /* ����1�Լ� */
      if (warnCount[4] >= 30) /* ����ʱ����ڵ���30s */
      {
        flag = TRUE; /* ���ù����־ */
      }
    }
    else
    {
      warnCount[4] = 0; /* ����1���� */
    }

    if (aqpx[6] >= 850) /* ��һ�����ѹ���ڵ���4.25V */
    {
      warnCount[5]++;         /* ����2�Լ� */
      if (warnCount[5] >= 30) /* ����ʱ����ڵ���30s */
      {
        flag = TRUE; /* ���ù����־  */
      }
    }
    else
    {
      warnCount[5] = 0; /* ����2���� */
    }

    if (flag == TRUE)
    {
      workEnState = (unsigned char)(workEnState | 0x04); /* �����־Ϊ��1 */
      ONOFFOutput(6);                                    /* ���ͳ�����ָ��  */
      flag = FALSE;
      warnCount[4] = 0;
      warnCount[5] = 0;
    }
  }
}

/*******************************************************
  ��������:  dischargeProtect
  ��������:  ���ű��� 
  �޸ļ�¼: 
********************************************************/
void DischargeProtect(void)
{
  float xdata batteryVoltage;

  if (fgDischargeEn == TRUE) /* �������� */
  {
    batteryVoltage = (float)((K5 * (float)anData8[1] * 0.02) + B5); /* �������ѹ����ֵ���� */

    if ((batteryVoltage <= 21) && (aqpx[3] <= 600)) /* ���ص�ѹС�ڵ���21�������������ϵ�ѹС�ڵ���3V (3��<=3V) */
    {
      warnCount[6]++;         /* ���ű��������ۼ� */
      if (warnCount[6] >= 30) /* ����ʱ��>=30�� */
      {
        workEnState = (unsigned char)(workEnState | 0x10); /* ���ű�־Ϊ��1 */
        fgExceDischarge = TRUE;                            /* �����ѹ��ű�־ */
      }
    }
    else
    {
      warnCount[6] = 0; /* �������� */
    }

    if (fgExceDischarge == TRUE) /* �ѹ��� */
    {
      warnCount[7]++;        /* �ѹ���ʱ�������1 */
      if (warnCount[7] > 32) /* ���ų���32S */
      {
        fgExceDischarge = FALSE;
        warnCount[6] = 0; /* 30s �� 16s ��ʱͬʱ���� */
        warnCount[7] = 0;
        ONOFFOutput(5); /* ��������Ͽ�ָ�� */
      }
    }
  }
}

/*******************************************************
  ��������:  ChargeUnlock
  ��������:  
  �޸ļ�¼:  

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

  if ((anData8[9] <= 60) && (anData8[10] == 0) && (flag == TRUE)) /* �жϳ��� <=3.6V  =0  MEA=3X  �궨��  20180403 */
  {
    chargeUnlockCount++;
    if (chargeUnlockCount >= 30)
    {
      ONOFFOutput(14);       /* ���ͳ����ֹ�Ͽ�ָ��    */
      chargeUnlockCount = 0; /* �������� */
    }
  }
  else
  {
    chargeUnlockCount = 0;
  }
}

/*******************************************************
  ��������:  EquControlCut
  ��������:  
  �޸ļ�¼:  
********************************************************/
void EquControlCut(void)
{
  unsigned char i;
  float xdata chargeCurrent;
  float xdata dischargeCurrent;

  if (equEn == TRUE) /* ����������� */
  {
    chargeCurrent = (float)((K1 * (float)anData12[4] * 0.00125) + B1);    /* ����������ֵ����  */
    dischargeCurrent = (float)((K2 * (float)anData12[5] * 0.00125) + B2); /* �ŵ��������ֵ����	 */

    if ((chargeCurrent <= 0.1) && (dischargeCurrent >= 1)) /* ������С�ڵ���0.1A�ҷŵ�������ڵ���1A */
    {
      equOutputWord = 0x00; /* �Ͽ�7����·ȫ�� */
      equControlEn = 0;
      EQU_ADDR = equOutputWord; /* ���þ��������� */
      for (i = 0; i < 7; i++)
      {
        equControlState[i] = FALSE; /* ��·���Ƶ�·״̬Ϊ�ر� */
      }
    }
  }
}

/*******************************************************
  ��������:  EquControlLink
  ��������:  
  �޸ļ�¼:  
********************************************************/
void EquControlLink(void)
{
  unsigned char i;
  unsigned char index;
  float xdata chargeCurrent;
  float xdata dischargeCurrent;

  if (equEn == TRUE) /* ����������� */
  {
    chargeCurrent = (float)((K1 * (float)anData12[4] * 0.00125) + B1);    /* ����������ֵ����  */
    dischargeCurrent = (float)((K2 * (float)anData12[5] * 0.00125) + B2); /* �ŵ��������ֵ����	 */

    if (dischargeCurrent <= 0.3) /* �ŵ����С�ڵ���0.3A */
    {
      index = 0;
      for (i = 0; i < 7; i++)
      {
        if (aq[i] > 600) /* �����ѹ����3V */
        {
          index++; /* ��ѹ����3V�ĵ������ */
        }
      }

      if (index >= 1) /*  ���������Ƶĵ�����ڵ���1 */
      {
        for (i = 0; i < 7; i++)
        {
          if ((aq[i] > 600) && (aq[i] < 860)) /* �����ѹ����3V �� С��4.3V	 */
          {
            if (equControlState[i] == FALSE) /* ״̬Ϊ�ر�  FALSE �Ͽ� */
            {
              if ((aq[i] - aqpx[7 - index]) >= 4) /* ��ֵ����40mv */
              {
                equOutputWord = (unsigned char)(equOutputWord | EQU_OPEN_Tab[i]); /* ָ���ͨ   ��1 */
                equControlState[i] = TRUE;                                        /* �õ�����·���Ƶ�·״̬Ϊ���� */
              }
            }
            else /* ״̬Ϊ��ͨ  */
            {
              if ((aq[i] - aqpx[7 - index]) <= 2) /* ��ֵС��20mv   ��0 */
              {
                equOutputWord = (unsigned char)(equOutputWord & EQU_CLOSE_Tab[i]); /* ָ��Ͽ� */
                equControlState[i] = FALSE;                                        /* �õ�����·���Ƶ�·״̬Ϊ�ر� */
              }
            }
          }
          else
          {
            equOutputWord = (unsigned char)(equOutputWord & EQU_CLOSE_Tab[i]); /* ָ��Ͽ� */
            equControlState[i] = FALSE;
          }
        }
      }
      equControlEn = 0; /* �������״̬ */
      EQU_ADDR = equOutputWord;
    }
  }
}

/*******************************************************
  ��������:  SafeWarn
  ��������:  
  �޸ļ�¼:  
********************************************************/
void SafeWarn(void)
{
  unsigned char i;
  unsigned char j;
  unsigned char temp;
  unsigned char flag;            /* ��ʶ��ʱ���� */
  unsigned char xdata heatTN[7]; /* �������� */

  flag = FALSE;

  if (anData8[0] <= GENERATRIX_VSET) /* ĸ�ߵ�ѹ */
  {
    warnCount[0]++;
    if (warnCount[0] >= 15)
    {
      warnState = (unsigned char)(warnState | 0x88);     /* D7��D3 = 1 */
      healthState = (unsigned char)(healthState | 0x08); /* D3 = 1 */

      healthData[1] = anData8[0]; /* �쳣ʱĸ�ߵ�ѹ���� */
      flag = TRUE;
    }
  }
  else
  {
    warnCount[0] = 0;
    healthState = (unsigned char)(healthState & 0xF7); /* D3 = 0 */
  }

  if (anData8[1] <= voltageWarnValue) /* �������ѹ���� */
  {
    warnCount[1]++;
    if (warnCount[1] >= 15)
    {
      warnState = (unsigned char)(warnState | 0x44);     /* D6��D2 = 1 */
      healthState = (unsigned char)(healthState | 0x04); /* D2 = 1 */

      healthData[2] = anData8[1]; /* �쳣ʱ�������ѹ���� */
      flag = TRUE;
    }
  }
  else
  {
    warnCount[1] = 0;
    healthState = (unsigned char)(healthState & 0xFB); /* D2 = 0 */
  }

  if (anData8[1] <= (unsigned char)(voltageWarnValue + 6)) /* �������ѹ���� Ԥ���� */
  {
    warnCount[8]++;
    if (warnCount[8] >= 15)
    {
      fgPreWarn = TRUE; /* ����Ԥ����־ */
    }
  }
  else
  {
    warnCount[8] = 0;
  }

  if (currentPower <= 100) /* ��ǰ��������(<=100Ah)  */
  {
    warnCount[2]++;
    if (warnCount[2] >= 15)
    {
      warnState = (unsigned char)(warnState | 0x22);     /* D5��D1 = 1 */
      healthState = (unsigned char)(healthState | 0x02); /* D1 = 1 */

      healthData[3] = powerSave[0]; /* �쳣ʱ��ǰ�������� */
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
    heatTN[i] = anData8[37 + i]; /* ��ȡ������7���¶�ֵ   */
  }

  for (i = 0; i < 6; i++) /* ��С����������� */
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

  if (((float)((unsigned int)heatTN[2] + (unsigned int)heatTN[3] + (unsigned int)heatTN[4]) / 3.0) <= TEMPE_35C) /* �м�ֵȡƽ�� 35C */
  {
    warnCount[3]++;
    if (warnCount[3] >= 15) /* ��ǰ�¶ȼ���(>=35��)  */
    {
      warnState = (unsigned char)(warnState | 0x11);     /* D4��D0 = 1 */
      healthState = (unsigned char)(healthState | 0x01); /* D0 = 1 */

      healthData[4] = (unsigned char)((float)((unsigned int)heatTN[2] + (unsigned int)heatTN[3] + (unsigned int)heatTN[4]) / 3.0); /* �쳣ʱ�¶�ƽ������ */
      flag = TRUE;
    }
  }
  else
  {
    warnCount[3] = 0;
    healthState = (unsigned char)(healthState & 0xFE); /* D0 = 0 */
  }

  if (flag == TRUE) /* ��ϵͳΣ�ձ�־ */
  {
    healthState = (unsigned char)(healthState | 0x80); /* D7 = 1 */
  }
  else
  {
    healthState = (unsigned char)(healthState & 0x7F); /* D7 = 0 */
  }
}

/*******************************************************
  ��������:  ONOFFHook
  ��������:  
  �޸ļ�¼:  
********************************************************/
void ONOFFHook(void)
{
  unsigned char i;

  if ((onoffNumber > 0) && (onoffNumber < 37) && (fgONOFFExecute == TRUE)) /* 1 - 36 ָ�Χ �� ָ������*/
  {
    if (onoffNumber < 17) /* OCָ��  1-16 */
    {
	  if(onoffNumber == 0x05)	/* ������ŵ翪�ضϿ�ָ�� */
	  {
	    if(fgDischargeEn == TRUE)  	/* ���ű�������ʱ */
		{
	       ONOFFOutput(onoffNumber); /* ִ��OCָ����� */	
		}
           rsCount++;        /* ң�����+1 */

	  }	
      else
	  {
         ONOFFOutput(onoffNumber); /* OCָ����� */		  
  		 rsCount++;          /* ң�����+1 */
	  }
    }
    else
    {
      if (onoffNumber < 24) /* ��·ָ�Χ  17-23 */
      {
        equOutputWord = (unsigned char)(equOutputWord | (_crol_((unsigned char)0x01, (unsigned char)(onoffNumber - 17)))); /* ��·������� */
        equControlEn = 0;                                                                                                  /* ��·����ʹ����Ч */
        EQU_ADDR = equOutputWord;                                                                                          /* ������·��� */

        equControlState[onoffNumber - 17] = TRUE; /* ������·���Ƶ�·����״̬ */

        rsCount++;        /* ң�����+1 */
      }
      else /* ����ָ�Χ */
      {
        switch (onoffNumber)
        {
        case 24: /* ����������� */
          equEn = TRUE;
          equEnB = TRUE;
          equEnC = TRUE;

          rsCount++;       /* ң�����+1 */
          break;

        case 25: /* ������ƽ�ֹ */
          equEn = FALSE;
          equEnB = FALSE;
          equEnC = FALSE;

          equOutputWord = 0x00; /* �Ͽ�7����·��� */
          equControlEn = 0;
          EQU_ADDR = equOutputWord; /* ���þ��������� */
          for (i = 0; i < 7; i++)
          {
            equControlState[i] = FALSE; /* ���õ�����·���Ƶ�·����״̬ */
          }

          rsCount++;        /* ң�����+1 */
          break;

        case 26: /* ��ʱ�ƿ������� */
          fgAhEn = TRUE;
          fgAhEnB = TRUE;
          fgAhEnC = TRUE;

          rsCount++;       /* ң�����+1 */
          break;

        case 27: /* ��ʱ�ƿ��ƽ�ֹ */
          fgAhEn = FALSE;
          fgAhEnB = FALSE;
          fgAhEnC = FALSE;

          fgAh = FALSE; /* ���ð�ʱ�ƿ��Ʊ�־(��ʾ0) */
          fgAhB = FALSE;
          fgAhC = FALSE;

          fgTinyCurrent = FALSE; /* ���������־(��ʾ0) */
          fgTinyCurrentB = FALSE;
          fgTinyCurrentC = FALSE;

          powerSave[0] = FULLPOWER_H; /* ��ǰ����150Ah  ���´�ֵ�� */
          powerSave[1] = FULLPOWER_L;
          powerSave[2] = 0; /* ������0Ah */
          powerSave[3] = 0;
          powerSave[4] = 0; /* �ŵ����0Ah */
          powerSave[5] = 0;

          currentPower = (float)FULLPOWER; /* ��ǰ����������ʼֵ ���������ʹ��ֵ�� */

          chargePower = (float)0; /* ������������ʼֵ	 */
          chargePowerB = (float)0;
          chargePowerC = (float)0;

          dischargePower = (float)0; /* �ŵ����������ʼֵ */
          dischargePowerB = (float)0;
          dischargePowerC = (float)0;

          rsCount++;         /* ң�����+1 */
          break;

        case 28: /* ���ű����������� */
          fgDischargeEn = TRUE;
          fgDischargeEnB = TRUE;
          fgDischargeEnC = TRUE;

          rsCount++;       /* ң�����+1 */
          break;

        case 29: /* ���ű������ƽ�ֹ */
          fgDischargeEn = FALSE;
          fgDischargeEnB = FALSE;
          fgDischargeEnC = FALSE;

          rsCount++;       /* ң�����+1 */
          break;

        case 30: /* �����־ */
          workEnState = (unsigned char)(workEnState & 0xFB);
          warnCount[4] = 0;
          warnCount[5] = 0;

          rsCount++;       /* ң�����+1 */
          break;

        case 31: /* ���䱣������ */
          fgChargeEn = TRUE;
          fgChargeEnB = TRUE;
          fgChargeEnC = TRUE;

          rsCount++;     /* ң�����+1 */
          break;

        case 32: /* ���䱣����ֹ */
          fgChargeEn = FALSE;
          fgChargeEnB = FALSE;
          fgChargeEnC = FALSE;

          rsCount++;      /* ң�����+1 */
          break;

        case 33: /* ����״̬����ָ�� */
          for (i = 0; i < 4; i++)
          {
            warnCount[i] = 0;      /* ��Դ��ȫ״̬�쳣�������� */
            healthData[i + 1] = 0; /* ����״̬�������� */
          }
          warnState = 0;   /* ��Դ��ȫ״̬������ */
          healthState = 0; /* ����״̬��־���� */

          rsCount++;     /* ң�����+1 */
          break;

        case 34:
          anCheckEn = TRUE; /* ����ģ�����Լ칦������ */
          anCheckEnB = TRUE;
          anCheckEnC = TRUE;

          rsCount++;       /* ң�����+1 */
          break;

        case 35:              /* ���߳�ʼ��ָ�� */
          EX0 = 0;            /* ���ӹ��жϲ���   */
          CANInit(BUS_ADDR); /* �A���߿��������³�ʼ�� */
          errorCount = 0;
          receiveSum = 0; /* ���߽������ݰ���������    */
          receiveCount = 0;
          receiveFrame = 0;

          canResetCount++;                         /* CAN��λ����++    */
          EX0 = 1;

          rsCount++;      /* ң�����+1 */
          break;

        case 36:             /* Ԥ��ֵ����ָ�� */
          fgPreWarn = FALSE; /* �����־ */
          warnCount[8] = 0;

          rsCount++;      /* ң�����+1 */
		  break;

        default:
          break;
        }
      }
    }

    onoffNumber = 0; /* ָ������� */
    onoffNumberB = 0;
    onoffNumberC = 0;

    fgONOFFExecute = FALSE; /* ����ָ��������Ч */
    fgONOFFExecuteB = FALSE;
    fgONOFFExecuteC = FALSE;

    rsCount = (unsigned char)(rsCount & 0x0F); /* ��Ч��Χ 0- F    */
  }
}

/*******************************************************
  ��������:  ONOFFOutput
  ��������:  
  �޸ļ�¼:  
********************************************************/
void ONOFFOutput(unsigned char index)
{
  ONOFF_ADDR = onoffTab[index - 1]; /* ����ָ�����ͨ�� */
  onoffEn = 0;                      /* ָ�����ʹ����Ч */
  Delay(11340);                     /* ��ʱԼ160ms */
  ONOFF_ADDR = 0xFF;                /* ������Чָ�����ͨ�� */
  onoffEn = 1;                      /* ָ�����ʹ����Ч */
}

/*******************************************************
  ��������:  DataLoad
  ��������:  
  �޸ļ�¼:  
********************************************************/
void DataLoad(void)
{
  unsigned char i;

  if (fgDataRequest == TRUE) /* ���ݿ���ע��ѯ���� */
  {
    if (uploadData[0] == 0x01) /* 01HΪ��ŵ�������ݿ� */
    {
      if ((uploadData[2] >= 50) && (uploadData[2] <= 250)) /* ����������ֹ���� W=X*100/2 ��Χ1-5 */
      {
        chargeEndValue = uploadData[2]; /*  */
        chargeEndValueB = uploadData[2];
        chargeEndValueC = uploadData[2];
      }

      if (uploadData[3] <= 250) /* ��ű�Ϊ0.9-1.15 X = W*1000 - 900 */
      {
        proportion = uploadData[3]; /* ���ó�ű� */
        proportionB = uploadData[3];
        proportionC = uploadData[3];
      }

      if ((uploadData[4] >= 100) && (uploadData[4] <= 150)) /* ��עֵ 100 - 150 */
      {
        fgAh = TRUE; /* ǿ��������ʱ�� */
        fgAhB = TRUE;
        fgAhC = TRUE;

        fgTinyCurrent = FALSE;
        fgTinyCurrentB = FALSE;
        fgTinyCurrentC = FALSE;

        dischargePower = (float)(FULLPOWER + chargePower - (float)(uploadData[4])); 
        dischargePowerB = dischargePower;
        dischargePowerC = dischargePower;
      }

      AD558set = (unsigned int)uploadData[5]; /* ���������õ�ƽ   ����������ִ�� */
      AD558setB = (unsigned int)uploadData[5];
      AD558setC = (unsigned int)uploadData[5];
    }

    if (uploadData[0] == 0x02) /* 02HΪ�¿ز������ݿ� */
    {
        if ((uploadData[3] & 0xC0) == 0x40) /* �ж�Ϊ�ջ�ģʽ */
        {
          if (((uploadData[3] & 0x0F) < 8) && ((uploadData[3] & 0x0F) > 0)) /* �ж�Ϊ��Чѡ����������� */
          {
            heatPara[uploadData[2] - 1].Mode = uploadData[3];
            heatParaB[uploadData[2] - 1].Mode = uploadData[3];
            heatParaC[uploadData[2] - 1].Mode = uploadData[3];
          }
        }

        if ((uploadData[3] & 0xC0) == 0x80) /* �жϿ��� */
        {
          heatPara[uploadData[2] - 1].Mode = uploadData[3];
          heatParaB[uploadData[2] - 1].Mode = uploadData[3];
          heatParaC[uploadData[2] - 1].Mode = uploadData[3];
        }

        if ((uploadData[4] >= TEMPE_35C) && (uploadData[5] <= TEMPE_10C)) /* �жϹر�����С�ڿ������� */
        {
          heatPara[uploadData[2] - 1].Close = uploadData[4]; /* �¿عرա���������  */
          heatPara[uploadData[2] - 1].Open = uploadData[5];

          heatParaB[uploadData[2] - 1].Close = uploadData[4]; /* �¿عرա��������� ����B */
          heatParaB[uploadData[2] - 1].Open = uploadData[5];

          heatParaC[uploadData[2] - 1].Close = uploadData[4]; /* �¿عرա��������� ����C */
          heatParaC[uploadData[2] - 1].Open = uploadData[5];
        }
    

      if ((uploadData[1] >= ACCUMULATOR_VMIN) && (uploadData[1] <= ACCUMULATOR_VMAX)) /* �������ѹ */
      {
        voltageWarnValue = uploadData[1];
        voltageWarnValueB = uploadData[1];
        voltageWarnValueC = uploadData[1];
      }
    }

    fgDataRequest = FALSE;

    rsCount++;
    rsCount = (unsigned char)(rsCount & 0x0F);

    for (i = 0; i < 8; i++) /* ��ע���ݿ����� */
    {
      uploadData[i] = 0;
    }
  }
}

/*******************************************************
  ��������:  Get2_3_X
  ��������:  
  �޸ļ�¼:  
********************************************************/
unsigned char Get2_3_X(unsigned char *a, unsigned char *b, unsigned char *c)
{
  if ((*a == *b) && (*b == *c)) /* �����Ա�һ�� */
  {
    return (TRUE); /* 3ȡ2�ж���ȷ */
  }
  else
  {
    if (*a == *b) /* ǰ���������	 */
    {
      EA = 0;  /* ���ж� ��ֹ�ж϶�д��ͻ    */
      *c = *a; /* ��ֵ���� */
      EA = 1;
      return (TRUE); /* 3ȡ2�ж���ȷ */
    }
    else
    {
      if (*a == *c) /* ����������� */
      {
        EA = 0;  /* ���ж� ��ֹ�ж϶�д��ͻ      */
        *b = *a; /* ��ֵ���� */
        EA = 1;
        return (TRUE); /* 3ȡ2�ж���ȷ */
      }
      else
      {
        if (*b == *c) /* ʣ����������� */
        {
          EA = 0;  /* ���ж� ��ֹ�ж϶�д��ͻ    */
          *a = *b; /* ��ֵ���� */
          EA = 1;
          return (TRUE); /* 3ȡ2�ж���ȷ */
        }
        else
        {
          return (FALSE); /* 3ȡ2�ж��쳣 */
        }
      }
    }
  }
}

/*******************************************************
  ��������:  Get2_3_I
  ��������:  
  �޸ļ�¼:  
********************************************************/
unsigned char Get2_3_I(unsigned int *a, unsigned int *b, unsigned int *c)
{
  if ((*a == *b) && (*b == *c)) /* �����Ա�һ�� */
  {
    return (TRUE); /* 3ȡ2�ж���ȷ */
  }
  else
  {
    if (*a == *b) /* ǰ���������	 */
    {
      *c = *a;       /* ��ֵ���� */
      return (TRUE); /* 3ȡ2�ж���ȷ */
    }
    else
    {
      if (*a == *c) /* ����������� */
      {
        *b = *a;       /* ��ֵ���� */
        return (TRUE); /* 3ȡ2�ж���ȷ */
      }
      else
      {
        if (*b == *c) /* ʣ����������� */
        {
          *a = *b;       /* ��ֵ���� */
          return (TRUE); /* 3ȡ2�ж���ȷ */
        }
        else
        {
          return (FALSE); /* 3ȡ2�ж��쳣 */
        }
      }
    }
  }
}

/*******************************************************
  ��������:   Get2_3_F
  ��������:  
  �޸ļ�¼:  
********************************************************/
unsigned char Get2_3_F(float *a, float *b, float *c)
{
  float temp;

  temp = *a - *b; /* ȡ����ֵ�ľ���ֵ     */
  if (temp < 0.0001)    /* �ж���������ֵ֮��С�ں�С������Ϊ��ȣ����Ϲ淶Ҫ��  */
  {
    *c = *a;       /* ��ֵ���� */
    return (TRUE); /* 3ȡ2�ж���ȷ */
  }
  else
  {
    temp = *a - *c; /* ������ֵ֮�����ֵ */
    if (temp < 0.0001)    /* �����С����ֵ */
    {
      *b = *a;       /* ��ֵ���� */
      return (TRUE); /* 3ȡ2�ж���ȷ */
    }
    else
    {
      temp = *b - *c; /* ʣ���������� */
      if (temp < 0.0001)    /* �����С����ֵ */
      {
        *a = *b;       /* ��ֵ���� */
        return (TRUE); /* 3ȡ2�ж���ȷ */
      }
      else /* 3��ֵ������� */
      {
        return (FALSE); /* 3ȡ2�ж��쳣 */
      }
    }
  }
}

/*******************************************************
  ��������:  Delay
  ��������:  
  �޸ļ�¼:  
********************************************************/
void Delay(unsigned int time)
{
  unsigned int k;
  for (k = 0; k < time; k++) /* ѭ����ʱ */
  {
    _nop_(); /* ִ�п���� */
  }
}

/*******************************************************
  ��������:  CAN_ISR
  ��������:  
  �޸ļ�¼:  
********************************************************/
void CAN_ISR(void) interrupt 0 using 1
{
  unsigned char IR;

  IR = XBYTE[BUS_ADDR + 3]; /* �������жϼĴ��� */
  if ((IR & 0x01) == 0x01)   /* �жϼĴ�������״̬���ж� */
  {
    CANRXD(); /* ���߽��� */
  }
  else if ((IR & 0x02) == 0x02) /* �жϼĴ�������״̬���ж� */
  {
    CANTXD(); /* ���߷��� */
  }
  else
  {
    _nop_(); /* �쳣�жϴ��� */
  }
}

/*******************************************************
  ��������:  CANRXD
  ��������:  
  �޸ļ�¼:  
********************************************************/
void CANRXD(void) using 1
{
  unsigned char i; /* ������ʱ���� */
  unsigned char DLC;
  unsigned char length;
  unsigned char fgEndFrame;
  unsigned char xdata *p;

  if ((XBYTE[0x5014] == 0x86) || (XBYTE[0x5014] == 0x46)) /* �ж�Ϊ��Դ��λ��ʶ���� */
  {
    fgEndFrame = FALSE;                 /* �����ݰ����ս�����־ */
    DLC = (XBYTE[0x5015] & 0x0F);       /* ֡��Ч���ݳ��� */
    if ((XBYTE[0x5015] & 0x20) == 0x00) /* �жϵ�֡/��֡����֡/��֡�ж�֡ */
    {
      length = DLC;                                    /* �ý������ݳ��� */
      p = 0x5016;                                      /* �ý�������ָ�� */
      receiveSum = 0;                                 /* ���������ۼӺ����� */
      receiveBuffer[0] = (unsigned char)(length - 2); /* �洢������ */
      receiveCount = 1;
    }
    else
    {
      if (DLC < 8) /* ����֡Ϊ��֡����֡ */
      {
        length = (unsigned char)(DLC - 1);             /* �ý������ݳ��� */
        receiveSum = receiveSum - receiveBuffer[0]; /* ���������ۼӺͼ�ȥ��֡���ݵ�length�ֽ� */
      }
      else /* ����֡Ϊ��֡�м�֡ */
      {
        length = 7;        /* �ý������ݳ��� */
        fgEndFrame = TRUE; /* �����ݰ����ڽ��ձ�־ */
      }
      p = 0x5017; /* �ý�������ָ��(������֡index) */

      if ((XBYTE[0x5014] != 0x46) || (XBYTE[0x5016] == 0) || (XBYTE[0x5016] != receiveFrame))
      {
        receiveSum = 0; /* �ۼӺ͡��������� */
        receiveCount = 0;
        receiveFrame = 0;
      }
      receiveFrame++; /* ��֡�������� */
    }

    for (i = 0; i < length; i++) /* ��������(TITLE+DATA+SUM)�������ۼӺ� */
    {
      receiveBuffer[receiveCount] = *p;
      receiveSum = receiveSum + *p; /* �ۼӺͼ���(Title + Data + Sum) */
      p++;
      receiveCount++; /* �洢DATA���� */
    }

    if (fgEndFrame == FALSE)
    {
      receiveSum = ((receiveSum - receiveBuffer[receiveCount - 1]) & 0x00FF); /* �����ۼӺ� */

      if ((receiveSum == receiveBuffer[receiveCount - 1]) /* �ж����ݰ����ۼӺ��볤�ȶ���ȷ */
          && (receiveCount == (unsigned char)(receiveBuffer[0] + 3)))
      {
        errorCount = 0; /* ͨѶ����������� */
        switch (receiveBuffer[1])
        {
        case 0x00: /* ��ѯ���ƴ��� */
          if ((XBYTE[0x5014] == 0x86) && (receiveBuffer[2] == 0x10))
          {
            RsManage();
          }
          break;

        case 0x20: /* ���ָ��� */
          if (XBYTE[0x5014] == 0x46)
          {
            ONOFFManage();
          }
          break;

        case 0x40: /* �������ݿ�(���²�����������)����鴦��   */
          if (XBYTE[0x5014] == 0x46)
          {
            DataManage();
          }
          break;

        default:
          break;
        }
      }
      receiveSum = 0; /* �ۼӺ����� */
      receiveCount = 0;
      receiveFrame = 0;
    }
  }
  XBYTE[0x5001] = 0x0C; /* �ͷŽ��ջ����� */
}

/*******************************************************
  ��������:  CANTXD
  ��������:  
  �޸ļ�¼:  
********************************************************/
void CANTXD(void) using 1
{
  unsigned char i;

  if (sendIndex != sendFrame) /* �ж�Ӧ��֡�Ƿ��� */
  {
    for (i = 0; i < 10; i++) /* ���ɷ���֡ */
    {
      XBYTE[0x500A + i] = *pSend;
      pSend++;
    }
    XBYTE[0x5001] = 0x01;
    sendIndex++; /* ����֡������+1 */
  }
  else
  {
    sendIndex = 1; /* ���÷��ͽ��� */
    sendFrame = 1;
  }
}

/*******************************************************
  ��������:  RsManage
  ��������:  
  �޸ļ�¼:  
********************************************************/
void RsManage(void) using 1
{
  unsigned char i;
  unsigned char xdata *p;
  p = 0x500A; /* ָ��CAN���߷��ͻ����� */

  switch (receiveBuffer[3]) /* �жϿ�����ѯ�������� */
  {
  case 0x01:               /* ״̬ң����� */
    if (fgSwitch == FALSE) /* �л�B���������� */
    {
      pSend = &soonFrameB[0];
    }
    else
    {
      pSend = &soonFrameA[0]; /* �л�A���������� */
    }
    for (i = 0; i < 10; i++) /* д�뷢�ͻ����� */
    {
      *p = *pSend;
      p++;
      pSend++;
    }
    XBYTE[0x5001] = 0x01; /* ��������֡ */
    sendIndex = 1;
    sendFrame = 5; /* 5֡�ٱ���� */
    break;

  case 0x02:               /* ��Դ����ң����� */
    if (fgSwitch == FALSE) /* �л�B������ */
    {
      pSend = &slowFrameB[0];
    }
    else
    {
      pSend = &slowFrameA[0]; /* �л�A������ */
    }

    for (i = 0; i < 10; i++) /* д�뷢�ͻ����� */
    {
      *p = *pSend;
      p++;
      pSend++;
    }
    XBYTE[0x5001] = 0x01; /* ��������֡ */
    sendIndex = 1;
    sendFrame = 11; /* 11֡������� */

    fgDownRs = TRUE; /* Ĭ�Ͻ�ֹ�л� */
    break;

  default:
    break;
  }
}

/*******************************************************
  ��������:  ONOFFManage
  ��������:  
  �޸ļ�¼:  
********************************************************/
void ONOFFManage(void) using 1
{
  unsigned char xdata *p;

  if ((receiveBuffer[2] == 0xA3) && (receiveBuffer[3] > 0) && (receiveBuffer[3] < 37 
      && (receiveBuffer[4] == 0xAA) && (receiveBuffer[5] == 0xAA))                     /* ��ʶ����Χ������� */
  {
    p = 0x500A; /* ָ��CAN���߷��ͻ����� */
    *p = 0x46;  /* Ӧ����    */
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

    XBYTE[0x5001] = 0x01; /* ��������֡ */
    sendIndex = 1;       /* ���ɷ�����Ϣ */
    sendFrame = 1;

    if ((onoffNumber == 0) && (fgONOFFExecute == FALSE)) /* �жϵ�ǰ��ָ��ִ��ʱ */
    {
      onoffNumber = receiveBuffer[3]; /* ��ȡָ��� */
      onoffNumberB = receiveBuffer[3];
      onoffNumberC = receiveBuffer[3];

      fgONOFFExecute = TRUE; /* ����ָ��ִ�б�־ */
      fgONOFFExecuteB = TRUE;
      fgONOFFExecuteC = TRUE;
    }
  }
}

/*******************************************************
  ��������:  DataManage
  ��������:  
  �޸ļ�¼:   
********************************************************/
void DataManage(void) using 1
{
  unsigned char i;
  unsigned char xdata *p;

  p = 0x500A; /* ָ��CAN���߷��ͻ����� */
  *p = 0x46;  /* Ӧ���� */
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

  XBYTE[0x5001] = 0x01; /* ��������֡ */
  sendIndex = 1;       /* ���ɷ�����Ϣ */
  sendFrame = 1;

  if (receiveBuffer[2] == 0x55) /* �ж�����ע���ݿ� */
  {
    for (i = 0; i < 8; i++) /* ��ȡ��ע������Ч���� */
    {
      uploadData[i] = receiveBuffer[i + 3];
    }

    fgDataRequest = TRUE; /* �������ݿ������־ */
  }
}

/*******************************************************
  ��������:  TIMER_ISR
  ��������:  
  �޸ļ�¼:  
********************************************************/
void TIMER_ISR(void) interrupt 1 using 2
{
  TR0 = 0;    /* ��ʱ�����¼��� */
  TH0 = 0xDC; /* װ����ֵ */
  TL0 = 0x00;
  TR0 = 1;
  clockCount++; /* ʱ�Ӽ��� */
}


