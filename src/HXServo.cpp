#include "HXServo.h"
#include "Wire.h"

HXServo::HXServo()
{
    Mode = I2C;
    serial = NULL;
    TxEn = NULL;
    RxEn = NULL;
    Wire.begin(); // join i2c bus (address optional for master)
}

HXServo::HXServo(HardwareSerial *serial1, uint16_t baud)
{
    Mode=USART_2_WIRE;
    Baudrate=baud;
    serial=serial1;
    TxEn = NULL;
    RxEn = NULL;
    serial->begin(Baudrate);
    serial->setTimeout(50);
    while (!(*serial));
}

HXServo::HXServo(HardwareSerial *serial1, uint16_t baud, uint8_t txEn, uint8_t rxEn)
{
    Mode=USART_1_WIRE;
    Baudrate=baud;
    serial=serial1;
    RxEn = rxEn;
    TxEn = txEn;

    pinMode(RxEn, OUTPUT);
    digitalWrite(RxEn, LOW);

    pinMode(TxEn, OUTPUT);
    digitalWrite(TxEn, LOW);

    serial->begin(Baudrate);
    serial->setTimeout(50);
    while (!(*serial));
}

uint8_t checkSumCal (uint8_t buf[],uint8_t offset, uint8_t length)
{
    uint8_t i = offset;
    uint8_t sum = 0;
    for (i = offset; i < length; i++)
    {
        sum ^= buf[i];
    }
    return sum;
}

uint16_t HXServo::readServo(uint16_t devAddr,uint8_t regAddr)
{
    uint8_t i=0;
    uint8_t tmp[4]={0};
    uint8_t checkSum=0;
    uint8_t index=0;
    uint8_t buff[32];
    uint16_t ret=0xffff;
    if(Mode==I2C)
    {
        if(devAddr==0)
        {
	           return ret;
        }

        Wire.beginTransmission((uint8_t)devAddr);   // transmit to device #8
        Wire.write(regAddr);              // sends one byte
        Wire.write(regAddr);              // sends one byte
        Wire.endTransmission();        // stop transmitting
      //delay(1);
        delayMicroseconds(500);

        Wire.requestFrom((uint8_t)devAddr, 4);     // request 6 bytes from slave device #8

        i=0;
        while (Wire.available()) { // slave may send less than requested
	           tmp[i] = Wire.read(); // receive a byte as character
	              i++;
        }
        if((tmp[0]>=POSITION_REGADDR)&&(tmp[0]<=COMMUNICATION_MODE_REGADDR))
        {
	           checkSum^=tmp[0];
               checkSum^=tmp[1];
               checkSum^=tmp[2];
               if(checkSum==tmp[3])
               {
                   ret = (tmp[1]<<8)|tmp[2];
               }
        }
    }
    else
    {
        if((devAddr>=BROADCAST_BUSADDR)||(devAddr<=MASTER_BUSADDR))
        {
	           return ret;
        }
        index=0;
        buff[index++]=0xaa; //帧头
        buff[index++]=0x55; //帧头
        buff[index++] = (MASTER_BUSADDR >> 8) & 0xff;  //主机地址高8位
        if(buff[index-1]==0xaa)
	    {
            buff[index++] = 0;     //补0
        }
        buff[index++] = MASTER_BUSADDR & 0xff;         //主机地址低8位
        if(buff[index-1]==0xaa)
        {
            buff[index++] = 0;     //补0
        }
        buff[index++] = (devAddr >> 8) & 0xff; //目的地址高8位 即舵机地址
        if(buff[index-1]==0xaa)
        {
            buff[index++] = 0;     //补0
        }
        buff[index++] = devAddr & 0xff;        //目的地址低8位 即舵机地址
        if(buff[index-1]==0xaa)
        {
            buff[index++] = 0;                 //补0
        }
        buff[index++] = 8;                    //帧长度
        if(buff[index-1]==0xaa)
        {
            buff[index++] = 0;     //补0
        }
        buff[index++] = READ_REG & 0xff;             //读寄存器
        if(buff[index-1]==0xaa)
        {
            buff[index++] = 0;     //补0
        }
        buff[index++] = regAddr & 0xff;               //寄存器地址或者指令码
        if(buff[index-1]==0xaa)
        {
            buff[index++] = 0;     //补0
        }
        checkSum = checkSumCal (buff, 2, index);  //数据帧校验码
        buff[index++] = checkSum;
        if(buff[index-1]==0xaa)
        {
            buff[index++] = 0;     //补0
        }
        buff[index++]=0xaa; //帧尾
        buff[index++]=0x81; //帧尾

        if(Mode == USART_1_WIRE){
            digitalWrite(RxEn, LOW);
            digitalWrite(TxEn, HIGH);
        }
        serial->write (buff,index);

        if (Mode == USART_1_WIRE) {
            serial->flush();
        }

        if (Mode == USART_1_WIRE) {
            digitalWrite(TxEn, LOW);
            digitalWrite(RxEn, HIGH);
        }

        index=serial->readBytes(buff,32);

        if (Mode == USART_1_WIRE) {
            digitalWrite(TxEn, LOW);
            digitalWrite(RxEn, LOW);
        }

        uint8_t buffData[32]={0};
        uint8_t i=0;

        if((index>=14)&&(buff[0]=0xaa)&&(buff[1]=0x55)&&(buff[index-1]=0x81)&&(buff[index-2]=0xaa))
        {
            for(i=2;i<index-2;i++)
            {
                buffData[i-2]=buff[i];
                if(buff[i]==0xaa)
                {
                    i++;
                }
            }
            if((((buffData[0]<<8)|(buffData[1]))==devAddr)&&(((buffData[2]<<8)|(buffData[3]))==MASTER_BUSADDR))
            {
                if((buffData[4]==10)&&(buffData[5]==ANSWER)&&(buffData[6]==regAddr))
                {
                    checkSum = checkSumCal (buffData, 0, 9);  //数据帧校验码
                    if(checkSum==buffData[9])
                    {
                        ret = (buffData[7]<<8)|buffData[8];
                    }
                }
            }
        }
    }
    return ret;
}

uint8_t HXServo::writeServo(uint16_t devAddr,uint8_t regAddr,uint16_t data)
{
    uint8_t checkSum=0;
    uint8_t tmp=0;
    uint8_t index=0;
    uint8_t buff[32];
    uint8_t ret=SETTING_FAIL;
    if(Mode==I2C)
    {
        Wire.beginTransmission((uint8_t)devAddr); // transmit to device #8
        Wire.write(regAddr);              // sends one byte
        checkSum^=regAddr;
        tmp=(data>>8)&0xff;
        checkSum^=tmp;
        Wire.write(tmp);              // sends one byte
        tmp=data&0xff;
        checkSum^=tmp;
        Wire.write(tmp);              // sends one byte
        Wire.write(checkSum);              // sends one byte
        Wire.endTransmission();        // stop transmitting
        ret = SETTING_OK;
    }
    else
    {
        if((devAddr>BROADCAST_BUSADDR)||(devAddr<=MASTER_BUSADDR))
        {
            return ret;
        }
        index=0;
        buff[index++]=0xaa; //帧头
        buff[index++]=0x55; //帧头
        buff[index++] = (MASTER_BUSADDR >> 8) & 0xff;  //主机地址高8位
        if(buff[index-1]==0xaa)
        {
            buff[index++] = 0;     //补0
        }
        buff[index++] = MASTER_BUSADDR & 0xff;         //主机地址低8位
        if(buff[index-1]==0xaa)
        {
            buff[index++] = 0;     //补0
        }
        buff[index++] = (devAddr >> 8) & 0xff; //目的地址高8位 即舵机地址
        if(buff[index-1]==0xaa)
        {
            buff[index++] = 0;     //补0
        }
        buff[index++] = devAddr & 0xff;        //目的地址低8位 即舵机地址
        if(buff[index-1]==0xaa)
        {
            buff[index++] = 0;                 //补0
        }
        buff[index++] = 10;                    //帧长度
        if(buff[index-1]==0xaa)
        {
            buff[index++] = 0;     //补0
        }
        buff[index++] = WRITE_REG & 0xff;             //写寄存器
        if(buff[index-1]==0xaa)
        {
            buff[index++] = 0;     //补0
        }
        buff[index++] = regAddr & 0xff;               //寄存器地址或者指令码
        if(buff[index-1]==0xaa)
        {
            buff[index++] = 0;     //补0
        }
        buff[index++] = (data>>8) & 0xff;            //数据高8位
        if(buff[index-1]==0xaa)
        {
            buff[index++] = 0;     //补0
        }
        buff[index++] = data & 0xff;               //数据低8位
        if(buff[index-1]==0xaa)
        {
            buff[index++] = 0;     //补0
        }
        checkSum = checkSumCal (buff, 2, index);  //数据帧校验码
        buff[index++] = checkSum;
        if(buff[index-1]==0xaa)
        {
            buff[index++] = 0;     //补0
        }
        buff[index++]=0xaa; //帧尾
        buff[index++]=0x81; //帧尾

        if (Mode == USART_1_WIRE) {
            digitalWrite(RxEn, LOW);
            digitalWrite(TxEn, HIGH);
        }
        serial->write (buff,index);
        if (Mode == USART_1_WIRE) {
            serial->flush();
        }

        if (Mode == USART_1_WIRE) {
            digitalWrite(RxEn, LOW);
            digitalWrite(TxEn, LOW);
        }

        if(devAddr!=BROADCAST_BUSADDR)
        {
            if (Mode == USART_1_WIRE) {
                digitalWrite(RxEn, HIGH);
                digitalWrite(TxEn, LOW);
            }
            index=serial->readBytes(buff,24);
            if (Mode == USART_1_WIRE) {
                digitalWrite(RxEn, LOW);
                digitalWrite(TxEn, LOW);
            }

            uint8_t buffData[24]={0};
            uint8_t i=0;
	  //ret=index;

            if((index>=13)&&(buff[0]=0xaa)&&(buff[1]=0x55)&&(buff[index-1]=0x81)&&(buff[index-2]=0xaa))
            {
                for(i=2;i<index-2;i++)
                {
                    buffData[i-2]=buff[i];
                    if(buff[i]==0xaa)
                    {
                        i++;
                    }
                }
                if((((buffData[0]<<8)|(buffData[1]))==devAddr)&&(((buffData[2]<<8)|(buffData[3]))==MASTER_BUSADDR))
                {
                    if((buffData[4]==9)&&(buffData[5]==ANSWER)&&(buffData[6]==regAddr))
                    {
                        checkSum = checkSumCal (buffData, 0, 8);  //数据帧校验码
                        if((checkSum==buffData[8])&&(buffData[7]==SETTING_OK))
                        {
                            ret = SETTING_OK;
                        }
                    }
                }
            }
        }
    }
    return ret;
}

void HXServo::changeToI2C()
{
    if(serial!=NULL)
    {
        serial->end();
    }
    Mode=I2C;
    serial=NULL;
    Wire.begin(); // join i2c bus (address optional for master)
}

void HXServo::changeToSerial(HardwareSerial *serial1,uint16_t baud)
{
    if(Mode==I2C)
    {
        Wire.end();
    }
    Mode=USART_2_WIRE;
    Baudrate=baud;
    if(serial!=NULL)
    {
        serial->end();
    }
    TxEn = NULL;
    RxEn = NULL;
    serial=serial1;
    serial->begin(Baudrate);
    serial->setTimeout(50);
    while (!(*serial));
}

void HXServo::changeToSerial(HardwareSerial *serial1,uint16_t baud, uint8_t txEn, uint8_t rxEn)
{
    if(Mode==I2C)
    {
        Wire.end();
    }
    Mode=USART_1_WIRE;
    Baudrate=baud;
    if(serial!=NULL)
    {
        serial->end();
    }

    TxEn = txEn;
    RxEn = rxEn;
    pinMode(RxEn, OUTPUT);
    digitalWrite(RxEn, LOW);
    pinMode(TxEn, OUTPUT);
    digitalWrite(TxEn, LOW);

    serial=serial1;
    serial->begin(Baudrate);
    serial->setTimeout(50);
    while (!(*serial));
}

void HXServo::changeSerialTimeout(uint16_t tim)
{
    if((Mode==USART_2_WIRE)||(Mode==USART_1_WIRE))
    {
        if(serial!=NULL)
        {
            serial->setTimeout(tim);
        }
    }
}
