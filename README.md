# 接线图

![img.png](asset/img.png)

## imu

- VCC,3.3V,电源正极,供电
- GND,GND,电源地,共地
- SCL,PB13,SPI_SCK (时钟),提供通信时钟
- SDA,PC1,SPI_MOSI (主机输出),STM32 发数据给 MPU
- AD0,PC2,SPI_MISO (主机输入),MPU 发数据给 STM32
- NCS (或 CS),PC0,MPU_CS (片选),拉低时激活 SPI 模式

## lidar
- VCC——>5v
- GND——>GND
- 黄TX——>板载RX——>PA10
- 绿RX——>板载TX——>PA9

## motor

- TIM2·ch1ch2左轮——>PA0、PA1
- TIM2·ch3ch4右轮——>PB2、PB10
- TIM3·左轮——>PA6、PA7
- TIM4·右轮——>PB6、PB7



## bluetooth

- VCC——>3.3v
- GND——>GND
- TXD——>板载RX——>PC5
- RXD——>板载TX——>PC10
