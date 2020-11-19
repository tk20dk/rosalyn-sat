EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "RosalynSAT"
Date "2020-11-17"
Rev "v1.0"
Comp "www.2-0.dk"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L rosalyn-sat:E22-EBYTE U2
U 1 1 5FA008B5
P 7350 3350
F 0 "U2" H 7350 3865 50  0000 C CNN
F 1 "E22-EBYTE" H 7350 3774 50  0000 C CNN
F 2 "rosalyn-sat:E22-EBYTE" H 7400 3350 50  0001 C CNN
F 3 "" H 7400 3350 50  0001 C CNN
	1    7350 3350
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J2
U 1 1 5FA0237F
P 7900 5350
F 0 "J2" H 7980 5342 50  0000 L CNN
F 1 "Serial" H 7800 5550 50  0000 L CNN
F 2 "rosalyn-sat:JST_SH_SM04B-SRSS-TB_1x04-1MP_P1.00mm_Horizontal" H 7900 5350 50  0001 C CNN
F 3 "~" H 7900 5350 50  0001 C CNN
	1    7900 5350
	1    0    0    -1  
$EndComp
Text GLabel 8050 3100 2    50   Input ~ 0
e22_rxen
Text GLabel 8050 3300 2    50   BiDi ~ 0
e22_dio1
Text GLabel 8050 3500 2    50   Output ~ 0
e22_busy
Text GLabel 8050 3600 2    50   Input ~ 0
e22_nrst
Text GLabel 6650 3100 0    50   Input ~ 0
e22_nss
Text GLabel 6650 3200 0    50   Input ~ 0
spi_sck
Text GLabel 6650 3300 0    50   Output ~ 0
spi_miso
Text GLabel 6650 3400 0    50   Input ~ 0
spi_mosi
NoConn ~ 6650 3550
Wire Wire Line
	7750 3950 7750 4000
Wire Wire Line
	7750 4000 7650 4000
Wire Wire Line
	6850 4000 6850 3950
Wire Wire Line
	6950 3950 6950 4000
Connection ~ 6950 4000
Wire Wire Line
	6950 4000 6850 4000
Wire Wire Line
	7050 3950 7050 4000
Connection ~ 7050 4000
Wire Wire Line
	7050 4000 6950 4000
Wire Wire Line
	7150 3950 7150 4000
Connection ~ 7150 4000
Wire Wire Line
	7150 4000 7050 4000
Wire Wire Line
	7250 3950 7250 4000
Connection ~ 7250 4000
Wire Wire Line
	7250 4000 7150 4000
Wire Wire Line
	7350 3950 7350 4000
Connection ~ 7350 4000
Wire Wire Line
	7350 4000 7300 4000
Wire Wire Line
	7450 3950 7450 4000
Connection ~ 7450 4000
Wire Wire Line
	7450 4000 7350 4000
Wire Wire Line
	7550 3950 7550 4000
Connection ~ 7550 4000
Wire Wire Line
	7550 4000 7450 4000
Wire Wire Line
	7650 3950 7650 4000
Connection ~ 7650 4000
Wire Wire Line
	7650 4000 7550 4000
$Comp
L power:GND #PWR0101
U 1 1 5FA08F6B
P 7300 4050
F 0 "#PWR0101" H 7300 3800 50  0001 C CNN
F 1 "GND" H 7300 3900 50  0000 C CNN
F 2 "" H 7300 4050 50  0001 C CNN
F 3 "" H 7300 4050 50  0001 C CNN
	1    7300 4050
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0102
U 1 1 5FA09361
P 8100 4050
F 0 "#PWR0102" H 8100 3900 50  0001 C CNN
F 1 "+3.3V" V 8100 4200 50  0000 L CNN
F 2 "" H 8100 4050 50  0001 C CNN
F 3 "" H 8100 4050 50  0001 C CNN
	1    8100 4050
	0    1    1    0   
$EndComp
Wire Wire Line
	7300 4050 7300 4000
Connection ~ 7300 4000
Wire Wire Line
	7300 4000 7250 4000
Wire Wire Line
	7850 4050 7850 3950
Text GLabel 7700 5450 0    50   Input ~ 0
uart1_tx
Text GLabel 7700 5550 0    50   Output ~ 0
uart1_rx
$Comp
L power:GND #PWR0104
U 1 1 5FA1A6BB
P 6050 5200
F 0 "#PWR0104" H 6050 4950 50  0001 C CNN
F 1 "GND" V 6050 5050 50  0000 R CNN
F 2 "" H 6050 5200 50  0001 C CNN
F 3 "" H 6050 5200 50  0001 C CNN
	1    6050 5200
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 5FA31D1A
P 7700 5350
F 0 "#PWR0105" H 7700 5100 50  0001 C CNN
F 1 "GND" V 7700 5200 50  0000 R CNN
F 2 "" H 7700 5350 50  0001 C CNN
F 3 "" H 7700 5350 50  0001 C CNN
	1    7700 5350
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 5FA38D6C
P 6000 1450
F 0 "#PWR0106" H 6000 1200 50  0001 C CNN
F 1 "GND" H 6000 1300 50  0000 C CNN
F 2 "" H 6000 1450 50  0001 C CNN
F 3 "" H 6000 1450 50  0001 C CNN
	1    6000 1450
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 5FA3BBE3
P 6450 1300
F 0 "C2" H 6565 1346 50  0000 L CNN
F 1 "100n" H 6565 1255 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6488 1150 50  0001 C CNN
F 3 "~" H 6450 1300 50  0001 C CNN
	1    6450 1300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0107
U 1 1 5FA3BBE9
P 6450 1450
F 0 "#PWR0107" H 6450 1200 50  0001 C CNN
F 1 "GND" H 6450 1300 50  0000 C CNN
F 2 "" H 6450 1450 50  0001 C CNN
F 3 "" H 6450 1450 50  0001 C CNN
	1    6450 1450
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0110
U 1 1 5FA3DE4F
P 7700 5250
F 0 "#PWR0110" H 7700 5100 50  0001 C CNN
F 1 "+5V" V 7700 5400 50  0000 L CNN
F 2 "" H 7700 5250 50  0001 C CNN
F 3 "" H 7700 5250 50  0001 C CNN
	1    7700 5250
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C5
U 1 1 5FA4BDA7
P 7850 4200
F 0 "C5" H 7965 4246 50  0000 L CNN
F 1 "100n" H 7965 4155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7888 4050 50  0001 C CNN
F 3 "~" H 7850 4200 50  0001 C CNN
	1    7850 4200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 5FA4BDAD
P 7850 4350
F 0 "#PWR05" H 7850 4100 50  0001 C CNN
F 1 "GND" H 7850 4200 50  0000 C CNN
F 2 "" H 7850 4350 50  0001 C CNN
F 3 "" H 7850 4350 50  0001 C CNN
	1    7850 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	8100 4050 7850 4050
Connection ~ 7850 4050
$Comp
L Regulator_Linear:XC6206PxxxMR U3
U 1 1 5FA52713
P 3850 5400
F 0 "U3" H 3850 5642 50  0000 C CNN
F 1 "XC6206P330MR" H 3850 5551 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 3850 5625 50  0001 C CIN
F 3 "https://www.torexsemi.com/file/xc6206/XC6206.pdf" H 3850 5400 50  0001 C CNN
	1    3850 5400
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5FA54C64
P 3350 5550
F 0 "C3" H 3465 5596 50  0000 L CNN
F 1 "4,7u" H 3465 5505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3388 5400 50  0001 C CNN
F 3 "~" H 3350 5550 50  0001 C CNN
	1    3350 5550
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 5FA56783
P 4400 5550
F 0 "C4" H 4515 5596 50  0000 L CNN
F 1 "10u" H 4515 5505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4438 5400 50  0001 C CNN
F 3 "~" H 4400 5550 50  0001 C CNN
	1    4400 5550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 5FA57CFC
P 3850 5700
F 0 "#PWR03" H 3850 5450 50  0001 C CNN
F 1 "GND" H 3855 5527 50  0000 C CNN
F 2 "" H 3850 5700 50  0001 C CNN
F 3 "" H 3850 5700 50  0001 C CNN
	1    3850 5700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5FA59C21
P 4400 5700
F 0 "#PWR04" H 4400 5450 50  0001 C CNN
F 1 "GND" H 4405 5527 50  0000 C CNN
F 2 "" H 4400 5700 50  0001 C CNN
F 3 "" H 4400 5700 50  0001 C CNN
	1    4400 5700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR02
U 1 1 5FA5A0DF
P 3350 5700
F 0 "#PWR02" H 3350 5450 50  0001 C CNN
F 1 "GND" H 3355 5527 50  0000 C CNN
F 2 "" H 3350 5700 50  0001 C CNN
F 3 "" H 3350 5700 50  0001 C CNN
	1    3350 5700
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR01
U 1 1 5FA5B853
P 2850 5400
F 0 "#PWR01" H 2850 5250 50  0001 C CNN
F 1 "+5V" V 2850 5550 50  0000 L CNN
F 2 "" H 2850 5400 50  0001 C CNN
F 3 "" H 2850 5400 50  0001 C CNN
	1    2850 5400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3550 5400 3350 5400
Wire Wire Line
	3350 5400 3150 5400
Connection ~ 3350 5400
$Comp
L power:+3.3V #PWR06
U 1 1 5FA5F4E2
P 4650 5400
F 0 "#PWR06" H 4650 5250 50  0001 C CNN
F 1 "+3.3V" V 4650 5550 50  0000 L CNN
F 2 "" H 4650 5400 50  0001 C CNN
F 3 "" H 4650 5400 50  0001 C CNN
	1    4650 5400
	0    1    1    0   
$EndComp
Wire Wire Line
	4150 5400 4400 5400
Wire Wire Line
	4400 5400 4650 5400
Connection ~ 4400 5400
$Comp
L Device:LED D1
U 1 1 5FAA80D2
P 7200 1600
F 0 "D1" V 7147 1680 50  0000 L CNN
F 1 "LED" V 7238 1680 50  0000 L CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 7200 1600 50  0001 C CNN
F 3 "~" H 7200 1600 50  0001 C CNN
	1    7200 1600
	0    1    1    0   
$EndComp
Text GLabel 7200 1150 1    50   Input ~ 0
hmi_status
$Comp
L Device:R R11
U 1 1 5FAAEC5C
P 7200 1300
F 0 "R11" H 7270 1346 50  0000 L CNN
F 1 "1k" H 7270 1255 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7130 1300 50  0001 C CNN
F 3 "~" H 7200 1300 50  0001 C CNN
	1    7200 1300
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 5FADFCA6
P 5100 1300
F 0 "#FLG0101" H 5100 1375 50  0001 C CNN
F 1 "PWR_FLAG" H 5100 1450 50  0000 C CNN
F 2 "" H 5100 1300 50  0001 C CNN
F 3 "~" H 5100 1300 50  0001 C CNN
	1    5100 1300
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0113
U 1 1 5FAE1C6E
P 5100 1300
F 0 "#PWR0113" H 5100 1150 50  0001 C CNN
F 1 "+5V" H 5100 1450 50  0000 C CNN
F 2 "" H 5100 1300 50  0001 C CNN
F 3 "" H 5100 1300 50  0001 C CNN
	1    5100 1300
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 5FAE256D
P 4700 1300
F 0 "#FLG0102" H 4700 1375 50  0001 C CNN
F 1 "PWR_FLAG" H 4700 1450 50  0000 C CNN
F 2 "" H 4700 1300 50  0001 C CNN
F 3 "~" H 4700 1300 50  0001 C CNN
	1    4700 1300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0114
U 1 1 5FAE3269
P 4700 1300
F 0 "#PWR0114" H 4700 1050 50  0001 C CNN
F 1 "GND" H 4700 1150 50  0000 C CNN
F 2 "" H 4700 1300 50  0001 C CNN
F 3 "" H 4700 1300 50  0001 C CNN
	1    4700 1300
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D2
U 1 1 5FA3A733
P 7550 1600
F 0 "D2" V 7497 1680 50  0000 L CNN
F 1 "LED" V 7588 1680 50  0000 L CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 7550 1600 50  0001 C CNN
F 3 "~" H 7550 1600 50  0001 C CNN
	1    7550 1600
	0    1    1    0   
$EndComp
Text GLabel 7550 1150 1    50   Input ~ 0
hmi_error
$Comp
L Device:R R12
U 1 1 5FA3A740
P 7550 1300
F 0 "R12" H 7620 1346 50  0000 L CNN
F 1 "1k" H 7620 1255 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7480 1300 50  0001 C CNN
F 3 "~" H 7550 1300 50  0001 C CNN
	1    7550 1300
	1    0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 5FA4AD22
P 5550 1300
F 0 "C6" H 5665 1346 50  0000 L CNN
F 1 "100n" H 5665 1255 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5588 1150 50  0001 C CNN
F 3 "~" H 5550 1300 50  0001 C CNN
	1    5550 1300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR08
U 1 1 5FA4AD28
P 5550 1450
F 0 "#PWR08" H 5550 1200 50  0001 C CNN
F 1 "GND" H 5550 1300 50  0000 C CNN
F 2 "" H 5550 1450 50  0001 C CNN
F 3 "" H 5550 1450 50  0001 C CNN
	1    5550 1450
	1    0    0    -1  
$EndComp
Text GLabel 5550 1150 1    50   UnSpc ~ 0
nrst
$Comp
L MCU_ST_STM32F0:STM32F030F4Px U1
U 1 1 5FB02EA0
P 4000 3450
F 0 "U1" H 4250 4200 50  0000 C CNN
F 1 "STM32F030F4Px" H 4500 4100 50  0000 C CNN
F 2 "Package_SO:TSSOP-20_4.4x6.5mm_P0.65mm" H 3600 2750 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00088500.pdf" H 4000 3450 50  0001 C CNN
	1    4000 3450
	1    0    0    -1  
$EndComp
Text GLabel 4500 3750 2    50   Output ~ 0
uart1_tx
Text GLabel 4500 3850 2    50   Input ~ 0
uart1_rx
Text GLabel 4500 3450 2    50   Output ~ 0
spi_sck
Text GLabel 4500 3650 2    50   Output ~ 0
spi_mosi
Text GLabel 4500 3550 2    50   Input ~ 0
spi_miso
Text GLabel 5050 4150 2    50   Output ~ 0
hmi_status
Text GLabel 5050 3850 2    50   Output ~ 0
hmi_error
Text GLabel 3500 4050 0    50   Output ~ 0
e22_nss
Text GLabel 4500 2950 2    50   Output ~ 0
e22_nrst
Text GLabel 3500 2950 0    50   UnSpc ~ 0
nrst
$Comp
L power:+3.3V #PWR09
U 1 1 5FB2A297
P 4050 2700
F 0 "#PWR09" H 4050 2550 50  0001 C CNN
F 1 "+3.3V" H 3950 2850 50  0000 L CNN
F 2 "" H 4050 2700 50  0001 C CNN
F 3 "" H 4050 2700 50  0001 C CNN
	1    4050 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 2750 4100 2700
Wire Wire Line
	4100 2700 4050 2700
Wire Wire Line
	4000 2750 4000 2700
Wire Wire Line
	4000 2700 4050 2700
Connection ~ 4050 2700
Text GLabel 3500 3150 0    50   Input ~ 0
boot0
Text GLabel 6000 1150 1    50   Input ~ 0
boot0
$Comp
L Device:R R1
U 1 1 5FB474FA
P 6000 1300
F 0 "R1" H 6070 1346 50  0000 L CNN
F 1 "100k" H 6070 1255 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5930 1300 50  0001 C CNN
F 3 "~" H 6000 1300 50  0001 C CNN
	1    6000 1300
	1    0    0    -1  
$EndComp
Text GLabel 5050 3950 2    50   BiDi ~ 0
swdio
Text GLabel 5050 4050 2    50   Input ~ 0
swclk
Text GLabel 6050 5400 0    50   Output ~ 0
swclk
Text GLabel 6050 5600 0    50   BiDi ~ 0
swdio
$Comp
L power:GND #PWR012
U 1 1 5FB65C99
P 4000 4250
F 0 "#PWR012" H 4000 4000 50  0001 C CNN
F 1 "GND" H 4000 4100 50  0000 C CNN
F 2 "" H 4000 4250 50  0001 C CNN
F 3 "" H 4000 4250 50  0001 C CNN
	1    4000 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 3950 5000 3950
Wire Wire Line
	5050 3850 5000 3850
Wire Wire Line
	5000 3850 5000 3950
Connection ~ 5000 3950
Wire Wire Line
	5000 3950 4500 3950
Wire Wire Line
	5050 4150 5000 4150
Wire Wire Line
	5000 4150 5000 4050
Wire Wire Line
	5000 4050 5050 4050
Wire Wire Line
	4500 4050 5000 4050
Connection ~ 5000 4050
NoConn ~ 4500 3350
$Comp
L power:+3.3V #PWR011
U 1 1 5FB5E039
P 6450 1150
F 0 "#PWR011" H 6450 1000 50  0001 C CNN
F 1 "+3.3V" H 6350 1300 50  0000 L CNN
F 2 "" H 6450 1150 50  0001 C CNN
F 3 "" H 6450 1150 50  0001 C CNN
	1    6450 1150
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR07
U 1 1 5FBDD719
P 7200 1750
F 0 "#PWR07" H 7200 1600 50  0001 C CNN
F 1 "+3.3V" H 7100 1900 50  0000 L CNN
F 2 "" H 7200 1750 50  0001 C CNN
F 3 "" H 7200 1750 50  0001 C CNN
	1    7200 1750
	-1   0    0    1   
$EndComp
$Comp
L power:+3.3V #PWR013
U 1 1 5FBDE17E
P 7550 1750
F 0 "#PWR013" H 7550 1600 50  0001 C CNN
F 1 "+3.3V" H 7450 1900 50  0000 L CNN
F 2 "" H 7550 1750 50  0001 C CNN
F 3 "" H 7550 1750 50  0001 C CNN
	1    7550 1750
	-1   0    0    1   
$EndComp
$Comp
L Connector:TestPoint TP1
U 1 1 5FBE1761
P 6050 5200
F 0 "TP1" V 6004 5388 50  0000 L CNN
F 1 "TestPoint" V 6095 5388 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_1.0x1.0mm" H 6250 5200 50  0001 C CNN
F 3 "~" H 6250 5200 50  0001 C CNN
	1    6050 5200
	0    1    1    0   
$EndComp
$Comp
L Connector:TestPoint TP2
U 1 1 5FBE2117
P 6050 5400
F 0 "TP2" V 6004 5588 50  0000 L CNN
F 1 "TestPoint" V 6095 5588 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_1.0x1.0mm" H 6250 5400 50  0001 C CNN
F 3 "~" H 6250 5400 50  0001 C CNN
	1    6050 5400
	0    1    1    0   
$EndComp
$Comp
L Connector:TestPoint TP3
U 1 1 5FBE25C3
P 6050 5600
F 0 "TP3" V 6004 5788 50  0000 L CNN
F 1 "TestPoint" V 6095 5788 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_1.0x1.0mm" H 6250 5600 50  0001 C CNN
F 3 "~" H 6250 5600 50  0001 C CNN
	1    6050 5600
	0    1    1    0   
$EndComp
Text GLabel 3500 3850 0    50   Input ~ 0
e22_busy
Text GLabel 3500 3750 0    50   BiDi ~ 0
e22_dio1
Text GLabel 4500 3250 2    50   Output ~ 0
e22_rxen
Text GLabel 8050 3200 2    50   Input ~ 0
e22_txen
Text GLabel 8050 3400 2    50   BiDi ~ 0
e22_dio2
Text GLabel 4500 3050 2    50   BiDi ~ 0
e22_dio2
Text GLabel 4500 3150 2    50   Output ~ 0
e22_txen
$Comp
L Device:Ferrite_Bead FB1
U 1 1 5FC0B9DD
P 3000 5400
F 0 "FB1" V 2726 5400 50  0000 C CNN
F 1 "Ferrite_Bead" V 2817 5400 50  0000 C CNN
F 2 "Inductor_SMD:L_0805_2012Metric" V 2930 5400 50  0001 C CNN
F 3 "~" H 3000 5400 50  0001 C CNN
	1    3000 5400
	0    1    1    0   
$EndComp
$Comp
L power:PWR_FLAG #FLG0103
U 1 1 5FB42719
P 3350 5400
F 0 "#FLG0103" H 3350 5475 50  0001 C CNN
F 1 "PWR_FLAG" H 3350 5550 50  0000 C CNN
F 2 "" H 3350 5400 50  0001 C CNN
F 3 "~" H 3350 5400 50  0001 C CNN
	1    3350 5400
	1    0    0    -1  
$EndComp
$EndSCHEMATC
