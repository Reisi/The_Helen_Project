EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Helen KD2"
Date "2022-02-18"
Rev "1.0"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Sensor_Motion:BMI160 U1
U 1 1 61E6C70B
P 6650 4000
F 0 "U1" H 6600 4581 50  0000 C CNN
F 1 "BMI160" H 6600 4490 50  0000 C CNN
F 2 "Package_LGA:Bosch_LGA-14_3x2.5mm_P0.5mm" H 6650 4000 50  0001 C CNN
F 3 "https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BMI160-DS000.pdf" H 5950 4850 50  0001 C CNN
F 4 "LGA-14" H 6650 4000 50  0001 C CNN "JLC"
F 5 "C94021" H 6650 4000 50  0001 C CNN "LCSC"
	1    6650 4000
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW1
U 1 1 61E6D845
P 8900 1350
F 0 "SW1" V 8946 1302 50  0000 R CNN
F 1 "SW_Push" V 8855 1302 50  0000 R CNN
F 2 "rt_button:JTP-1138EM" H 8900 1550 50  0001 C CNN
F 3 "~" H 8900 1550 50  0001 C CNN
F 4 "TASTER 9316" H 8900 1350 50  0001 C CNN "Reichelt"
F 5 "SMD-4_6.0x6.0x4.5P" H 8900 1350 50  0001 C CNN "JLC"
F 6 "C381064" H 8900 1350 50  0001 C CNN "LCSC"
	1    8900 1350
	0    -1   -1   0   
$EndComp
$Comp
L Regulator_Linear:MCP1703A-3302_SOT23 U4
U 1 1 61E6E411
P 2500 1200
F 0 "U4" H 2500 1442 50  0000 C CNN
F 1 "SE8530X2-HF" H 2500 1351 50  0000 C CNN
F 2 "drv:SOT-23_MCP1703" H 2500 1400 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/20005122B.pdf" H 2500 1150 50  0001 C CNN
F 4 "MCP 1703T-3302E" H 2500 1200 50  0001 C CNN "Reichelt"
F 5 "SOT-23-3" H 2500 1200 50  0001 C CNN "JLC"
F 6 "C115014" H 2500 1200 50  0001 C CNN "LCSC"
	1    2500 1200
	1    0    0    -1  
$EndComp
$Comp
L Device:C C9
U 1 1 61E6EC85
P 2000 1450
F 0 "C9" H 2115 1496 50  0000 L CNN
F 1 "4µ7/16V" H 2115 1405 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2038 1300 50  0001 C CNN
F 3 "~" H 2000 1450 50  0001 C CNN
F 4 "0603" H 2000 1450 50  0001 C CNN "JLC"
F 5 "C19666" H 2000 1450 50  0001 C CNN "LCSC"
F 6 "X5R-G0603 2,2/16" H 2000 1450 50  0001 C CNN "Reichelt"
	1    2000 1450
	1    0    0    -1  
$EndComp
$Comp
L Device:C C10
U 1 1 61E6FA03
P 3000 1450
F 0 "C10" H 3115 1496 50  0000 L CNN
F 1 "4µ7/16V" H 3115 1405 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3038 1300 50  0001 C CNN
F 3 "~" H 3000 1450 50  0001 C CNN
F 4 "0603" H 3000 1450 50  0001 C CNN "JLC"
F 5 "C19666" H 3000 1450 50  0001 C CNN "LCSC"
F 6 "X5R-G0603 2,2/16" H 3000 1450 50  0001 C CNN "Reichelt"
	1    3000 1450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR011
U 1 1 61E7007C
P 2500 1800
F 0 "#PWR011" H 2500 1550 50  0001 C CNN
F 1 "GND" H 2505 1627 50  0000 C CNN
F 2 "" H 2500 1800 50  0001 C CNN
F 3 "" H 2500 1800 50  0001 C CNN
	1    2500 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 1200 2000 1200
Wire Wire Line
	2000 1200 2000 1300
Wire Wire Line
	2000 1600 2000 1700
Wire Wire Line
	2000 1700 2500 1700
Wire Wire Line
	2500 1700 2500 1500
Wire Wire Line
	2500 1800 2500 1700
Connection ~ 2500 1700
Wire Wire Line
	2500 1700 3000 1700
Wire Wire Line
	3000 1700 3000 1600
Wire Wire Line
	3000 1200 2800 1200
Wire Wire Line
	3000 1300 3000 1200
$Comp
L power:VCC #PWR08
U 1 1 61E71135
P 2000 1000
F 0 "#PWR08" H 2000 850 50  0001 C CNN
F 1 "VCC" H 2015 1173 50  0000 C CNN
F 2 "" H 2000 1000 50  0001 C CNN
F 3 "" H 2000 1000 50  0001 C CNN
	1    2000 1000
	1    0    0    -1  
$EndComp
$Comp
L power:+3V0 #PWR09
U 1 1 61E722A7
P 3000 1000
F 0 "#PWR09" H 3000 850 50  0001 C CNN
F 1 "+3V0" H 3015 1173 50  0000 C CNN
F 2 "" H 3000 1000 50  0001 C CNN
F 3 "" H 3000 1000 50  0001 C CNN
	1    3000 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 1000 3000 1200
Connection ~ 3000 1200
Wire Wire Line
	2000 1000 2000 1200
Connection ~ 2000 1200
$Comp
L power:GND #PWR05
U 1 1 61E72DD4
P 700 5500
F 0 "#PWR05" H 700 5250 50  0001 C CNN
F 1 "GND" H 705 5327 50  0000 C CNN
F 2 "" H 700 5500 50  0001 C CNN
F 3 "" H 700 5500 50  0001 C CNN
	1    700  5500
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 61E737A6
P 700 4050
F 0 "C2" H 815 4096 50  0000 L CNN
F 1 "4µ7/16V" H 815 4005 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 738 3900 50  0001 C CNN
F 3 "~" H 700 4050 50  0001 C CNN
F 4 "0603" H 700 4050 50  0001 C CNN "JLC"
F 5 "C19666" H 700 4050 50  0001 C CNN "LCSC"
F 6 "X5R-G0603 2,2/16" H 700 4050 50  0001 C CNN "Reichelt"
	1    700  4050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 61E74DBD
P 1000 4300
F 0 "C4" H 1115 4346 50  0000 L CNN
F 1 "100n" H 1115 4255 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1038 4150 50  0001 C CNN
F 3 "~" H 1000 4300 50  0001 C CNN
F 4 "0603" H 1000 4300 50  0001 C CNN "JLC"
F 5 "C14663" H 1000 4300 50  0001 C CNN "LCSC"
F 6 "X7R-G0603 100N" H 1000 4300 50  0001 C CNN "Reichelt"
	1    1000 4300
	1    0    0    -1  
$EndComp
$Comp
L Device:C C7
U 1 1 61E7A722
P 5700 4650
F 0 "C7" H 5815 4696 50  0000 L CNN
F 1 "100n" H 5815 4605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5738 4500 50  0001 C CNN
F 3 "~" H 5700 4650 50  0001 C CNN
F 4 "0603" H 5700 4650 50  0001 C CNN "JLC"
F 5 "C14663" H 5700 4650 50  0001 C CNN "LCSC"
F 6 "X7R-G0603 100N" H 5700 4650 50  0001 C CNN "Reichelt"
	1    5700 4650
	1    0    0    -1  
$EndComp
NoConn ~ 7050 3900
NoConn ~ 7050 4000
NoConn ~ 7050 4100
NoConn ~ 7050 4200
Wire Wire Line
	6550 3600 6550 3500
Wire Wire Line
	6550 3500 5700 3500
Wire Wire Line
	5700 3500 5700 4100
Wire Wire Line
	5700 4100 6000 4100
Wire Wire Line
	6550 3500 6650 3500
Wire Wire Line
	6650 3500 6650 3600
Connection ~ 6550 3500
Wire Wire Line
	6150 3800 5600 3800
Wire Wire Line
	5700 4800 5700 4900
Wire Wire Line
	5700 4900 6000 4900
Wire Wire Line
	6550 4500 6550 4900
Wire Wire Line
	6650 4500 6650 4900
Wire Wire Line
	6650 4900 6550 4900
Connection ~ 6550 4900
Wire Wire Line
	5700 4100 5700 4500
Connection ~ 5700 4100
$Comp
L power:GND #PWR04
U 1 1 61E83F4E
P 5700 5000
F 0 "#PWR04" H 5700 4750 50  0001 C CNN
F 1 "GND" H 5705 4827 50  0000 C CNN
F 2 "" H 5700 5000 50  0001 C CNN
F 3 "" H 5700 5000 50  0001 C CNN
	1    5700 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 4900 5700 5000
Connection ~ 5700 4900
Wire Wire Line
	5700 3000 5700 3100
Connection ~ 5700 3500
$Comp
L Device:LED D4
U 1 1 61E92FB7
P 9200 1550
F 0 "D4" V 9239 1432 50  0000 R CNN
F 1 "LED_RED" V 9148 1432 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 9200 1550 50  0001 C CNN
F 3 "~" H 9200 1550 50  0001 C CNN
F 4 "LED_0603" H 9200 1550 50  0001 C CNN "JLC"
F 5 "C2286" H 9200 1550 50  0001 C CNN "LCSC"
F 6 "LED EL 0603 RT" H 9200 1550 50  0001 C CNN "Reichelt"
	1    9200 1550
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D3
U 1 1 61E93A08
P 9500 1350
F 0 "D3" V 9539 1232 50  0000 R CNN
F 1 "LED_BLUE" V 9448 1232 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 9500 1350 50  0001 C CNN
F 3 "~" H 9500 1350 50  0001 C CNN
F 4 "LED_0603" H 9500 1350 50  0001 C CNN "JLC"
F 5 "C72041" H 9500 1350 50  0001 C CNN "LCSC"
F 6 "LED EL 0603 BL" H 9500 1350 50  0001 C CNN "Reichelt"
	1    9500 1350
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R8
U 1 1 61E9559B
P 9200 1050
F 0 "R8" H 9270 1096 50  0000 L CNN
F 1 "820" H 9270 1005 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 9130 1050 50  0001 C CNN
F 3 "~" H 9200 1050 50  0001 C CNN
F 4 "SMD-0603 820" H 9200 1050 50  0001 C CNN "Reichelt"
F 5 "0603" H 9200 1050 50  0001 C CNN "JLC"
F 6 "C23253" H 9200 1050 50  0001 C CNN "LCSC"
	1    9200 1050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R7
U 1 1 61E95E86
P 9500 950
F 0 "R7" H 9570 996 50  0000 L CNN
F 1 "120" H 9570 905 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 9430 950 50  0001 C CNN
F 3 "~" H 9500 950 50  0001 C CNN
F 4 "SMD-0603 120" H 9500 950 50  0001 C CNN "Reichelt"
F 5 "0603" H 9500 950 50  0001 C CNN "JLC"
F 6 "C22787" H 9500 950 50  0001 C CNN "LCSC"
	1    9500 950 
	1    0    0    -1  
$EndComp
$Comp
L Device:Ferrite_Bead_Small FB1
U 1 1 61E967B0
P 700 2900
F 0 "FB1" H 800 2946 50  0000 L CNN
F 1 "Ferrite_Bead_Small" H 800 2855 50  0000 L CNN
F 2 "Inductor_SMD:L_0603_1608Metric" V 630 2900 50  0001 C CNN
F 3 "~" H 700 2900 50  0001 C CNN
F 4 "0603" H 700 2900 50  0001 C CNN "JLC"
F 5 "C1002" H 700 2900 50  0001 C CNN "LCSC"
F 6 "BLM18PG 471" H 700 2900 50  0001 C CNN "Reichelt"
	1    700  2900
	1    0    0    -1  
$EndComp
$Comp
L Device:Ferrite_Bead_Small FB2
U 1 1 61E9927E
P 5700 3300
F 0 "FB2" H 5800 3346 50  0000 L CNN
F 1 "Ferrite_Bead_Small" H 5800 3255 50  0000 L CNN
F 2 "Inductor_SMD:L_0603_1608Metric" V 5630 3300 50  0001 C CNN
F 3 "~" H 5700 3300 50  0001 C CNN
F 4 "0603" H 5700 3300 50  0001 C CNN "JLC"
F 5 "C1002" H 5700 3300 50  0001 C CNN "LCSC"
F 6 "BLM18PG 471" H 5700 3300 50  0001 C CNN "Reichelt"
	1    5700 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 3400 5700 3500
$Comp
L Device:R R10
U 1 1 61E9B691
P 5850 1550
F 0 "R10" H 5920 1596 50  0000 L CNN
F 1 "47k" H 5920 1505 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5780 1550 50  0001 C CNN
F 3 "~" H 5850 1550 50  0001 C CNN
F 4 "SMD-0603 47K" H 5850 1550 50  0001 C CNN "Reichelt"
F 5 "0603" H 5850 1550 50  0001 C CNN "JLC"
F 6 "C25819" H 5850 1550 50  0001 C CNN "LCSC"
	1    5850 1550
	1    0    0    -1  
$EndComp
$Comp
L Device:C C11
U 1 1 61E9BD6B
P 6150 1550
F 0 "C11" H 6265 1596 50  0000 L CNN
F 1 "100n" H 6265 1505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6188 1400 50  0001 C CNN
F 3 "~" H 6150 1550 50  0001 C CNN
F 4 "0603" H 6150 1550 50  0001 C CNN "JLC"
F 5 "C14663" H 6150 1550 50  0001 C CNN "LCSC"
F 6 "X7R-G0603 100N" H 6150 1550 50  0001 C CNN "Reichelt"
	1    6150 1550
	1    0    0    -1  
$EndComp
$Comp
L Device:R R9
U 1 1 61E9D0CF
P 5850 1050
F 0 "R9" H 5920 1096 50  0000 L CNN
F 1 "1M" H 5920 1005 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5780 1050 50  0001 C CNN
F 3 "~" H 5850 1050 50  0001 C CNN
F 4 "SMD-0603 1,0M" H 5850 1050 50  0001 C CNN "Reichelt"
F 5 "0603" H 5850 1050 50  0001 C CNN "JLC"
F 6 "C22935" H 5850 1050 50  0001 C CNN "LCSC"
	1    5850 1050
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x06 J3
U 1 1 61E9F81D
P 6250 6850
F 0 "J3" H 6330 6842 50  0000 L CNN
F 1 "Conn_01x06" H 6330 6751 50  0000 L CNN
F 2 "Connector_PinHeader_2.00mm:PinHeader_1x06_P2.00mm_Vertical" H 6250 6850 50  0001 C CNN
F 3 "~" H 6250 6850 50  0001 C CNN
F 4 "BKL 10120732" H 6250 6850 50  0001 C CNN "Reichelt"
	1    6250 6850
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR010
U 1 1 61EA02B7
P 5750 6550
F 0 "#PWR010" H 5750 6400 50  0001 C CNN
F 1 "VCC" H 5765 6723 50  0000 C CNN
F 2 "" H 5750 6550 50  0001 C CNN
F 3 "" H 5750 6550 50  0001 C CNN
	1    5750 6550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR014
U 1 1 61EA1BC0
P 5950 7250
F 0 "#PWR014" H 5950 7000 50  0001 C CNN
F 1 "GND" H 5955 7077 50  0000 C CNN
F 2 "" H 5950 7250 50  0001 C CNN
F 3 "" H 5950 7250 50  0001 C CNN
	1    5950 7250
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 7150 5950 7150
Wire Wire Line
	5950 7150 5950 7250
Wire Wire Line
	6050 7050 5750 7050
Wire Wire Line
	5750 7050 5750 6550
Wire Wire Line
	6050 6850 5650 6850
Wire Wire Line
	6050 6750 5650 6750
Wire Wire Line
	6050 6650 5650 6650
Text Label 5650 6650 2    50   ~ 0
SDA
Text Label 5650 6750 2    50   ~ 0
SCL
Text Label 5650 6850 2    50   ~ 0
COM
$Comp
L Device:R R3
U 1 1 61EA7CAC
P 5400 3650
F 0 "R3" H 5470 3696 50  0000 L CNN
F 1 "4k7" H 5470 3605 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5330 3650 50  0001 C CNN
F 3 "~" H 5400 3650 50  0001 C CNN
F 4 "SMD-0603 4,7K" H 5400 3650 50  0001 C CNN "Reichelt"
F 5 "0603" H 5400 3650 50  0001 C CNN "JLC"
F 6 "C23162" H 5400 3650 50  0001 C CNN "LCSC"
	1    5400 3650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 61EA844D
P 5100 3650
F 0 "R2" H 5170 3696 50  0000 L CNN
F 1 "4k7" H 5170 3605 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5030 3650 50  0001 C CNN
F 3 "~" H 5100 3650 50  0001 C CNN
F 4 "SMD-0603 4,7K" H 5100 3650 50  0001 C CNN "Reichelt"
F 5 "0603" H 5100 3650 50  0001 C CNN "JLC"
F 6 "C23162" H 5100 3650 50  0001 C CNN "LCSC"
	1    5100 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 3100 5400 3100
Wire Wire Line
	5400 3100 5400 3500
Connection ~ 5700 3100
Wire Wire Line
	5700 3100 5700 3200
Wire Wire Line
	5400 3100 5100 3100
Wire Wire Line
	5100 3100 5100 3500
Connection ~ 5400 3100
Wire Wire Line
	5400 3900 5400 3800
Wire Wire Line
	5400 3900 6150 3900
Wire Wire Line
	5400 3900 5000 3900
Connection ~ 5400 3900
Wire Wire Line
	5100 4000 5100 3800
Wire Wire Line
	5100 4000 6150 4000
Wire Wire Line
	5100 4000 5000 4000
Connection ~ 5100 4000
Text Label 5000 3900 2    50   ~ 0
SDA
Text Label 5000 4000 2    50   ~ 0
SCL
$Comp
L power:GND #PWR012
U 1 1 61EB00A0
P 9200 1900
F 0 "#PWR012" H 9200 1650 50  0001 C CNN
F 1 "GND" H 9205 1727 50  0000 C CNN
F 2 "" H 9200 1900 50  0001 C CNN
F 3 "" H 9200 1900 50  0001 C CNN
	1    9200 1900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR013
U 1 1 61EB066E
P 5850 1900
F 0 "#PWR013" H 5850 1650 50  0001 C CNN
F 1 "GND" H 5855 1727 50  0000 C CNN
F 2 "" H 5850 1900 50  0001 C CNN
F 3 "" H 5850 1900 50  0001 C CNN
	1    5850 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	6150 1700 6150 1800
Wire Wire Line
	6150 1800 5850 1800
Wire Wire Line
	5850 1800 5850 1700
Wire Wire Line
	5850 1800 5850 1900
Connection ~ 5850 1800
Wire Wire Line
	6150 1400 6150 1300
Wire Wire Line
	6150 1300 5850 1300
Wire Wire Line
	5850 1300 5850 1400
Wire Wire Line
	5850 1200 5850 1300
Connection ~ 5850 1300
Wire Wire Line
	9200 1200 9200 1400
Wire Wire Line
	9200 1700 9200 1800
Wire Wire Line
	9200 1800 9500 1800
Wire Wire Line
	9500 1800 9500 1500
Wire Wire Line
	9500 1200 9500 1100
Connection ~ 9200 1800
Wire Wire Line
	9200 1800 9200 1900
Wire Wire Line
	9200 1800 8900 1800
Wire Wire Line
	8900 1800 8900 1550
Wire Wire Line
	8900 1150 8900 900 
Wire Wire Line
	8900 900  8800 900 
Wire Wire Line
	9200 900  9200 800 
Wire Wire Line
	9200 800  8800 800 
Wire Wire Line
	9500 800  9500 700 
Wire Wire Line
	9500 700  8800 700 
$Comp
L power:VCC #PWR07
U 1 1 61EC3CD5
P 5850 800
F 0 "#PWR07" H 5850 650 50  0001 C CNN
F 1 "VCC" H 5865 973 50  0000 C CNN
F 2 "" H 5850 800 50  0001 C CNN
F 3 "" H 5850 800 50  0001 C CNN
	1    5850 800 
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 800  5850 900 
$Comp
L Connector_Generic:Conn_01x01 J5
U 1 1 61EDD4CB
P 800 6600
F 0 "J5" H 950 6600 50  0000 C CNN
F 1 "Conn_01x01" H 718 6466 50  0001 C CNN
F 2 "rt_pins:1,5x3mm_SMD" H 800 6600 50  0001 C CNN
F 3 "~" H 800 6600 50  0001 C CNN
	1    800  6600
	-1   0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J7
U 1 1 61EDE326
P 800 6800
F 0 "J7" H 950 6800 50  0000 C CNN
F 1 "Conn_01x01" H 718 6666 50  0001 C CNN
F 2 "rt_pins:1,5x3mm_SMD" H 800 6800 50  0001 C CNN
F 3 "~" H 800 6800 50  0001 C CNN
	1    800  6800
	-1   0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J8
U 1 1 61EDEA11
P 800 7000
F 0 "J8" H 950 7000 50  0000 C CNN
F 1 "Conn_01x01" H 718 6866 50  0001 C CNN
F 2 "rt_pins:1,5x3mm_SMD" H 800 7000 50  0001 C CNN
F 3 "~" H 800 7000 50  0001 C CNN
	1    800  7000
	-1   0    0    1   
$EndComp
$Comp
L power:VCC #PWR017
U 1 1 61EDED24
P 1100 6500
F 0 "#PWR017" H 1100 6350 50  0001 C CNN
F 1 "VCC" H 1115 6673 50  0000 C CNN
F 2 "" H 1100 6500 50  0001 C CNN
F 3 "" H 1100 6500 50  0001 C CNN
	1    1100 6500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR018
U 1 1 61EDF97D
P 1100 7500
F 0 "#PWR018" H 1100 7250 50  0001 C CNN
F 1 "GND" H 1105 7327 50  0000 C CNN
F 2 "" H 1100 7500 50  0001 C CNN
F 3 "" H 1100 7500 50  0001 C CNN
	1    1100 7500
	1    0    0    -1  
$EndComp
$Comp
L Device:R R12
U 1 1 61EE01F6
P 1250 6800
F 0 "R12" V 1043 6800 50  0000 C CNN
F 1 "10" V 1134 6800 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1180 6800 50  0001 C CNN
F 3 "~" H 1250 6800 50  0001 C CNN
F 4 "SMD-0603 10" H 1250 6800 50  0001 C CNN "Reichelt"
F 5 "0603" H 1250 6800 50  0001 C CNN "JLC"
F 6 "C22859" H 1250 6800 50  0001 C CNN "LCSC"
	1    1250 6800
	0    1    1    0   
$EndComp
$Comp
L Device:D_Zener D5
U 1 1 61EE1537
P 1500 7150
F 0 "D5" V 1454 7230 50  0000 L CNN
F 1 "18V" V 1545 7230 50  0000 L CNN
F 2 "Diode_SMD:D_SOT-23_ANK" H 1500 7150 50  0001 C CNN
F 3 "~" H 1500 7150 50  0001 C CNN
F 4 "SOT-23-3" H 1500 7150 50  0001 C CNN "JLC"
F 5 "C841166" H 1500 7150 50  0001 C CNN "LCSC"
F 6 "SMD ZD 18" H 1500 7150 50  0001 C CNN "Reichelt"
	1    1500 7150
	0    1    1    0   
$EndComp
$Comp
L Device:Q_NMOS_GSD Q2
U 1 1 61EE249F
P 1800 6700
F 0 "Q2" V 2049 6700 50  0000 C CNN
F 1 "2N7002" V 2140 6700 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 2000 6800 50  0001 C CNN
F 3 "~" H 1800 6700 50  0001 C CNN
F 4 "2N 7002 SMD" H 1800 6700 50  0001 C CNN "Reichelt"
F 5 "SOT-23-3" H 1800 6700 50  0001 C CNN "JLC"
F 6 "C8545" H 1800 6700 50  0001 C CNN "LCSC"
	1    1800 6700
	0    -1   1    0   
$EndComp
$Comp
L Device:R R11
U 1 1 61EE3067
P 2050 6400
F 0 "R11" V 1843 6400 50  0000 C CNN
F 1 "4k7" V 1934 6400 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1980 6400 50  0001 C CNN
F 3 "~" H 2050 6400 50  0001 C CNN
F 4 "SMD-0603 4,7K" H 2050 6400 50  0001 C CNN "Reichelt"
F 5 "0603" H 2050 6400 50  0001 C CNN "JLC"
F 6 "C23162" H 2050 6400 50  0001 C CNN "LCSC"
	1    2050 6400
	0    1    1    0   
$EndComp
Wire Wire Line
	1000 7000 1100 7000
Wire Wire Line
	1100 7000 1100 7400
Wire Wire Line
	1100 7400 1500 7400
Wire Wire Line
	1500 7400 1500 7300
Connection ~ 1100 7400
Wire Wire Line
	1100 7400 1100 7500
Wire Wire Line
	1000 6800 1100 6800
Wire Wire Line
	1400 6800 1500 6800
Wire Wire Line
	1800 6300 1800 6400
Wire Wire Line
	1800 6400 1900 6400
Wire Wire Line
	2300 6400 2300 6800
Wire Wire Line
	2300 6800 2000 6800
Wire Wire Line
	2200 6400 2300 6400
Connection ~ 1800 6400
Wire Wire Line
	1800 6400 1800 6500
Wire Wire Line
	2300 6800 2400 6800
Connection ~ 2300 6800
Wire Wire Line
	1000 6600 1100 6600
Wire Wire Line
	1100 6600 1100 6500
Text Label 2400 6800 0    50   ~ 0
COM
Wire Wire Line
	1500 6800 1500 7000
Connection ~ 1500 6800
Wire Wire Line
	1500 6800 1600 6800
$Comp
L Device:R R4
U 1 1 61EFDEED
P 8550 5000
F 0 "R4" V 8343 5000 50  0000 C CNN
F 1 "10" V 8434 5000 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8480 5000 50  0001 C CNN
F 3 "~" H 8550 5000 50  0001 C CNN
F 4 "SMD-0603 10" H 8550 5000 50  0001 C CNN "Reichelt"
F 5 "0603" H 8550 5000 50  0001 C CNN "JLC"
F 6 "C22859" H 8550 5000 50  0001 C CNN "LCSC"
	1    8550 5000
	0    1    1    0   
$EndComp
$Comp
L Device:R R5
U 1 1 61EFE6E3
P 8800 5250
F 0 "R5" H 8730 5204 50  0000 R CNN
F 1 "47k" H 8730 5295 50  0000 R CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8730 5250 50  0001 C CNN
F 3 "~" H 8800 5250 50  0001 C CNN
F 4 "SMD-0603 47K" H 8800 5250 50  0001 C CNN "Reichelt"
F 5 "0603" H 8800 5250 50  0001 C CNN "JLC"
F 6 "C25819" H 8800 5250 50  0001 C CNN "LCSC"
	1    8800 5250
	-1   0    0    1   
$EndComp
$Comp
L Device:Q_NMOS_GSD Q1
U 1 1 61EFEC0E
P 9100 5000
F 0 "Q1" H 9304 5046 50  0000 L CNN
F 1 "AO3400A" H 9304 4955 50  0000 L CNN
F 2 "drv:SOT-23_AO3400A" H 9300 5100 50  0001 C CNN
F 3 "~" H 9100 5000 50  0001 C CNN
F 4 "IRLML 0030" H 9100 5000 50  0001 C CNN "Reichelt"
F 5 "SOT-23-3L" H 9100 5000 50  0001 C CNN "JLC"
F 6 "C20917" H 9100 5000 50  0001 C CNN "LCSC"
	1    9100 5000
	1    0    0    -1  
$EndComp
$Comp
L Device:L L1
U 1 1 61F003B6
P 9200 3450
F 0 "L1" H 9253 3496 50  0000 L CNN
F 1 "3µ-18" H 9253 3405 50  0000 L CNN
F 2 "rt_chokes:Choke_SMD_Fastron_242418_FPS" H 9200 3450 50  0001 C CNN
F 3 "~" H 9200 3450 50  0001 C CNN
F 4 "IND-SMD" H 9200 3450 50  0001 C CNN "JLC"
F 5 "C83456" H 9200 3450 50  0001 C CNN "LCSC"
F 6 "L-242418FPS 3,0µ" H 9200 3450 50  0001 C CNN "Reichelt"
	1    9200 3450
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 61F04CEE
P 9200 4250
F 0 "C3" H 9315 4296 50  0000 L CNN
F 1 "10u/25V" H 9315 4205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 9238 4100 50  0001 C CNN
F 3 "~" H 9200 4250 50  0001 C CNN
F 4 "0805" H 9200 4250 50  0001 C CNN "JLC"
F 5 "C15850" H 9200 4250 50  0001 C CNN "LCSC"
F 6 "X5R-G0805 10/16" H 9200 4250 50  0001 C CNN "Reichelt"
	1    9200 4250
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Schottky D1
U 1 1 61F06683
P 8900 3700
F 0 "D1" V 8854 3780 50  0000 L CNN
F 1 "SS34" V 8945 3780 50  0000 L CNN
F 2 "Diode_SMD:D_SMA" H 8900 3700 50  0001 C CNN
F 3 "~" H 8900 3700 50  0001 C CNN
F 4 "SMA,DO-214AC" H 8900 3700 50  0001 C CNN "JLC"
F 5 "C8678" H 8900 3700 50  0001 C CNN "LCSC"
F 6 "SK 34SMA DIO" H 8900 3700 50  0001 C CNN "Reichelt"
	1    8900 3700
	0    1    1    0   
$EndComp
$Comp
L Device:C C5
U 1 1 61F06E75
P 7900 4450
F 0 "C5" H 8015 4496 50  0000 L CNN
F 1 "10u/25V" H 8015 4405 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 7938 4300 50  0001 C CNN
F 3 "~" H 7900 4450 50  0001 C CNN
F 4 "0805" H 7900 4450 50  0001 C CNN "JLC"
F 5 "C15850" H 7900 4450 50  0001 C CNN "LCSC"
F 6 "X5R-G0805 10/16" H 7900 4450 50  0001 C CNN "Reichelt"
	1    7900 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	8800 5400 8800 5500
Wire Wire Line
	8800 5500 7900 5500
Wire Wire Line
	7900 5500 7900 4600
Wire Wire Line
	7900 3200 8900 3200
Wire Wire Line
	9200 3200 9200 3300
Wire Wire Line
	9200 5500 8800 5500
Wire Wire Line
	8800 5000 8900 5000
Wire Wire Line
	8800 5100 8800 5000
Wire Wire Line
	9200 3600 9200 3700
Wire Wire Line
	9200 4400 9200 4600
Wire Wire Line
	7900 4300 7900 3200
Connection ~ 8800 5500
Wire Wire Line
	9200 5200 9200 5500
Wire Wire Line
	8800 5000 8700 5000
Connection ~ 8800 5000
Wire Wire Line
	8400 5000 8300 5000
Wire Wire Line
	9200 4600 8900 4600
Wire Wire Line
	8900 4600 8900 3850
Connection ~ 9200 4600
Wire Wire Line
	9200 4600 9200 4800
Wire Wire Line
	8900 3550 8900 3200
Connection ~ 8900 3200
Wire Wire Line
	8900 3200 9200 3200
$Comp
L Device:R R1
U 1 1 61F1DFF8
P 9750 3700
F 0 "R1" V 9543 3700 50  0000 C CNN
F 1 "1k5" V 9634 3700 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 9680 3700 50  0001 C CNN
F 3 "~" H 9750 3700 50  0001 C CNN
F 4 "SMD-0603 1,5K" H 9750 3700 50  0001 C CNN "Reichelt"
F 5 "0603" H 9750 3700 50  0001 C CNN "JLC"
F 6 "C22843" H 9750 3700 50  0001 C CNN "LCSC"
	1    9750 3700
	0    1    1    0   
$EndComp
$Comp
L Device:C C1
U 1 1 61F1E7E2
P 10000 3450
F 0 "C1" H 10115 3496 50  0000 L CNN
F 1 "100n" H 10115 3405 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 10038 3300 50  0001 C CNN
F 3 "~" H 10000 3450 50  0001 C CNN
F 4 "0603" H 10000 3450 50  0001 C CNN "JLC"
F 5 "C14663" H 10000 3450 50  0001 C CNN "LCSC"
F 6 "X7R-G0603 100N" H 10000 3450 50  0001 C CNN "Reichelt"
	1    10000 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	9600 3700 9200 3700
Connection ~ 9200 3700
Wire Wire Line
	9200 3700 9200 3900
Wire Wire Line
	9200 3200 10000 3200
Connection ~ 9200 3200
Wire Wire Line
	10000 3200 10000 3300
Wire Wire Line
	10000 3700 9900 3700
Wire Wire Line
	10000 3600 10000 3700
Connection ~ 10000 3200
$Comp
L power:VCC #PWR02
U 1 1 61FB531B
P 7900 3100
F 0 "#PWR02" H 7900 2950 50  0001 C CNN
F 1 "VCC" H 7915 3273 50  0000 C CNN
F 2 "" H 7900 3100 50  0001 C CNN
F 3 "" H 7900 3100 50  0001 C CNN
	1    7900 3100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 61FB5EC5
P 7900 5600
F 0 "#PWR06" H 7900 5350 50  0001 C CNN
F 1 "GND" H 7905 5427 50  0000 C CNN
F 2 "" H 7900 5600 50  0001 C CNN
F 3 "" H 7900 5600 50  0001 C CNN
	1    7900 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 5500 7900 5600
Connection ~ 7900 5500
Wire Wire Line
	7900 3100 7900 3200
Connection ~ 7900 3200
$Comp
L Amplifier_Current:ZXCT1009F U3
U 1 1 620003E1
P 10200 4150
F 0 "U3" H 9912 4196 50  0000 R CNN
F 1 "ZXCT1009F" H 9912 4105 50  0000 R CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 10200 4150 50  0001 C CNN
F 3 "https://www.diodes.com/assets/Datasheets/ZXCT1009.pdf" H 10150 4150 50  0001 C CNN
F 4 "ZXCT 1009FTA" H 10200 4150 50  0001 C CNN "Reichelt"
F 5 "SOT-23-3" H 10200 4150 50  0001 C CNN "JLC"
F 6 "C83670" H 10200 4150 50  0001 C CNN "LCSC"
	1    10200 4150
	-1   0    0    -1  
$EndComp
Wire Wire Line
	10000 3700 10000 3950
Connection ~ 10000 3700
Wire Wire Line
	10400 3200 10400 3950
Wire Wire Line
	10000 3200 10400 3200
$Comp
L Device:R R6
U 1 1 62025BF0
P 10200 5250
F 0 "R6" H 10130 5204 50  0000 R CNN
F 1 "820" H 10130 5295 50  0000 R CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 10130 5250 50  0001 C CNN
F 3 "~" H 10200 5250 50  0001 C CNN
F 4 "SMD-0603 820" H 10200 5250 50  0001 C CNN "Reichelt"
F 5 "0603" H 10200 5250 50  0001 C CNN "JLC"
F 6 "C23253" H 10200 5250 50  0001 C CNN "LCSC"
	1    10200 5250
	-1   0    0    1   
$EndComp
$Comp
L Device:C C8
U 1 1 62026785
P 10500 5250
F 0 "C8" H 10615 5296 50  0000 L CNN
F 1 "100n" H 10615 5205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 10538 5100 50  0001 C CNN
F 3 "~" H 10500 5250 50  0001 C CNN
F 4 "0603" H 10500 5250 50  0001 C CNN "JLC"
F 5 "C14663" H 10500 5250 50  0001 C CNN "LCSC"
F 6 "X7R-G0603 100N" H 10500 5250 50  0001 C CNN "Reichelt"
	1    10500 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	9200 5500 9900 5500
Wire Wire Line
	10500 5500 10500 5400
Wire Wire Line
	10500 4700 10200 4700
Wire Wire Line
	10200 4700 10200 4350
Wire Wire Line
	10500 5100 10500 4700
Connection ~ 9200 5500
Wire Wire Line
	10200 4700 10200 5100
Wire Wire Line
	10200 5400 10200 5500
Connection ~ 10200 4700
Connection ~ 10200 5500
Wire Wire Line
	10200 5500 10500 5500
Wire Wire Line
	10200 4700 9900 4700
Wire Wire Line
	9900 4700 9900 4800
Wire Wire Line
	9900 5400 9900 5500
Connection ~ 9900 5500
Wire Wire Line
	9900 5500 10200 5500
Wire Wire Line
	2900 5200 2900 5400
Wire Wire Line
	700  5400 700  4200
Wire Wire Line
	2900 3200 2900 3100
Wire Wire Line
	2900 3100 1100 3100
Wire Wire Line
	700  3100 1000 3100
Connection ~ 700  3100
Wire Wire Line
	700  3100 700  3000
Connection ~ 1000 3100
Wire Wire Line
	1000 3100 1000 4150
Wire Wire Line
	1000 4450 1000 5400
Connection ~ 1000 5400
Wire Wire Line
	1000 5400 700  5400
Wire Wire Line
	700  5400 700  5500
Connection ~ 700  5400
$Comp
L Device:C C6
U 1 1 61F9F24A
P 6000 4550
F 0 "C6" H 6115 4596 50  0000 L CNN
F 1 "100n" H 6115 4505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6038 4400 50  0001 C CNN
F 3 "~" H 6000 4550 50  0001 C CNN
F 4 "0603" H 6000 4550 50  0001 C CNN "JLC"
F 5 "C14663" H 6000 4550 50  0001 C CNN "LCSC"
F 6 "X7R-G0603 100N" H 6000 4550 50  0001 C CNN "Reichelt"
	1    6000 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 4900 6000 4700
Wire Wire Line
	6000 4400 6000 4100
Connection ~ 6000 4900
Wire Wire Line
	6000 4900 6550 4900
Connection ~ 6000 4100
Wire Wire Line
	6000 4100 6150 4100
$Comp
L Connector_Generic:Conn_01x01 J1
U 1 1 61FC024A
P 9500 3900
F 0 "J1" H 9650 3900 50  0000 C CNN
F 1 "Conn_01x01" H 9418 3766 50  0001 C CNN
F 2 "rt_pins:1,5x3mm_SMD" H 9500 3900 50  0001 C CNN
F 3 "~" H 9500 3900 50  0001 C CNN
	1    9500 3900
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J2
U 1 1 61FCB426
P 9500 4600
F 0 "J2" H 9650 4600 50  0000 C CNN
F 1 "Conn_01x01" H 9418 4466 50  0001 C CNN
F 2 "rt_pins:1,5x3mm_SMD" H 9500 4600 50  0001 C CNN
F 3 "~" H 9500 4600 50  0001 C CNN
	1    9500 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	9300 3900 9200 3900
Connection ~ 9200 3900
Wire Wire Line
	9200 3900 9200 4100
Wire Wire Line
	9300 4600 9200 4600
$Comp
L Device:C C12
U 1 1 61EDBAA2
P 3100 7150
F 0 "C12" H 3215 7196 50  0000 L CNN
F 1 "10u/25V" H 3215 7105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3138 7000 50  0001 C CNN
F 3 "~" H 3100 7150 50  0001 C CNN
F 4 "0805" H 3100 7150 50  0001 C CNN "JLC"
F 5 "C15850" H 3100 7150 50  0001 C CNN "LCSC"
F 6 "X5R-G0805 10/16" H 3100 7150 50  0001 C CNN "Reichelt"
	1    3100 7150
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J4
U 1 1 61EDBE31
P 4700 6400
F 0 "J4" H 4850 6400 50  0000 C CNN
F 1 "Conn_01x01" H 4618 6266 50  0001 C CNN
F 2 "rt_pins:1,5x3mm_SMD" H 4700 6400 50  0001 C CNN
F 3 "~" H 4700 6400 50  0001 C CNN
	1    4700 6400
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J6
U 1 1 61EDC0F7
P 4700 6600
F 0 "J6" H 4850 6600 50  0000 C CNN
F 1 "Conn_01x01" H 4618 6466 50  0001 C CNN
F 2 "rt_pins:1,5x3mm_SMD" H 4700 6600 50  0001 C CNN
F 3 "~" H 4700 6600 50  0001 C CNN
	1    4700 6600
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NMOS_GSD Q3
U 1 1 61EDC419
P 4300 6900
F 0 "Q3" H 4504 6946 50  0000 L CNN
F 1 "AO3400A" H 4504 6855 50  0000 L CNN
F 2 "drv:SOT-23_AO3400A" H 4500 7000 50  0001 C CNN
F 3 "~" H 4300 6900 50  0001 C CNN
F 4 "SOT-23-3L" H 4300 6900 50  0001 C CNN "JLC"
F 5 "C20917" H 4300 6900 50  0001 C CNN "LCSC"
F 6 "IRLML 0030" H 4300 6900 50  0001 C CNN "Reichelt"
	1    4300 6900
	1    0    0    -1  
$EndComp
$Comp
L Device:R R14
U 1 1 61EDC91F
P 4000 7150
F 0 "R14" H 3930 7104 50  0000 R CNN
F 1 "47k" H 3930 7195 50  0000 R CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3930 7150 50  0001 C CNN
F 3 "~" H 4000 7150 50  0001 C CNN
F 4 "SMD-0603 47K" H 4000 7150 50  0001 C CNN "Reichelt"
F 5 "0603" H 4000 7150 50  0001 C CNN "JLC"
F 6 "C25819" H 4000 7150 50  0001 C CNN "LCSC"
	1    4000 7150
	-1   0    0    1   
$EndComp
$Comp
L Device:R R13
U 1 1 61EDCB66
P 3750 6900
F 0 "R13" V 3543 6900 50  0000 C CNN
F 1 "10" V 3634 6900 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3680 6900 50  0001 C CNN
F 3 "~" H 3750 6900 50  0001 C CNN
F 4 "SMD-0603 10" H 3750 6900 50  0001 C CNN "Reichelt"
F 5 "0603" H 3750 6900 50  0001 C CNN "JLC"
F 6 "C22859" H 3750 6900 50  0001 C CNN "LCSC"
	1    3750 6900
	0    1    1    0   
$EndComp
$Comp
L power:VCC #PWR015
U 1 1 61EDCF5B
P 3100 6300
F 0 "#PWR015" H 3100 6150 50  0001 C CNN
F 1 "VCC" H 3115 6473 50  0000 C CNN
F 2 "" H 3100 6300 50  0001 C CNN
F 3 "" H 3100 6300 50  0001 C CNN
	1    3100 6300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR019
U 1 1 61EE78EB
P 3100 7500
F 0 "#PWR019" H 3100 7250 50  0001 C CNN
F 1 "GND" H 3105 7327 50  0000 C CNN
F 2 "" H 3100 7500 50  0001 C CNN
F 3 "" H 3100 7500 50  0001 C CNN
	1    3100 7500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 6300 3100 6400
Wire Wire Line
	3100 7300 3100 7400
Wire Wire Line
	4500 6400 3100 6400
Connection ~ 3100 6400
Wire Wire Line
	4500 6600 4400 6600
Wire Wire Line
	4400 6600 4400 6700
Wire Wire Line
	4400 7100 4400 7400
Wire Wire Line
	4400 7400 4000 7400
Connection ~ 3100 7400
Wire Wire Line
	3100 7400 3100 7500
Wire Wire Line
	3900 6900 4000 6900
Wire Wire Line
	4000 6900 4000 7000
Wire Wire Line
	4000 7300 4000 7400
Connection ~ 4000 6900
Wire Wire Line
	4000 6900 4100 6900
Connection ~ 4000 7400
Wire Wire Line
	4000 7400 3100 7400
Wire Wire Line
	3100 6400 3100 7000
Text Label 3900 5000 0    50   ~ 0
SDA
Text Label 3900 4900 0    50   ~ 0
SCL
Text Label 3900 4600 0    50   ~ 0
COM
Wire Wire Line
	5000 4200 6150 4200
Wire Wire Line
	5000 4300 6150 4300
Text Label 5000 4200 2    50   ~ 0
BMI_INT1
Text Label 5000 4300 2    50   ~ 0
BMI_INT2
Text Label 3900 4500 0    50   ~ 0
BMI_INT1
Text Label 3900 4400 0    50   ~ 0
BMI_INT2
Wire Wire Line
	10500 4700 10600 4700
Connection ~ 10500 4700
Text Label 10600 4700 0    50   ~ 0
LED_CURR
Text Label 3900 4800 0    50   ~ 0
LED_CURR
Wire Wire Line
	3900 4500 3800 4500
Wire Wire Line
	3900 4600 3800 4600
Wire Wire Line
	3900 4700 3800 4700
Wire Wire Line
	3900 4800 3800 4800
Wire Wire Line
	3900 4900 3800 4900
Wire Wire Line
	3900 5000 3800 5000
Wire Wire Line
	1000 5400 1100 5400
Text Label 3500 6900 2    50   ~ 0
AUX_PWM
Text Label 3900 4200 0    50   ~ 0
LED_PWM
Text Label 8800 900  2    50   ~ 0
BUTTON
Text Label 8800 800  2    50   ~ 0
STATUS_RED
Text Label 8800 700  2    50   ~ 0
STATUS_BLUE
Text Label 6250 1300 0    50   ~ 0
VIN_VOLT
Text Label 3900 4700 0    50   ~ 0
VIN_VOLT
Wire Wire Line
	3800 4400 3900 4400
Text Label 3900 4000 0    50   ~ 0
AUX_PWM
Text Label 3900 4100 0    50   ~ 0
STATUS_RED
Text Label 3900 5300 0    50   ~ 0
BUTTON
Text Label 3900 4300 0    50   ~ 0
STATUS_BLUE
Wire Wire Line
	3800 4000 3900 4000
Wire Wire Line
	3900 4100 3800 4100
Wire Wire Line
	3800 4200 3900 4200
Wire Wire Line
	3900 4300 3800 4300
Text Label 8300 5000 2    50   ~ 0
LED_PWM
Wire Wire Line
	700  3900 700  3100
Wire Wire Line
	1900 5300 1900 4700
Wire Wire Line
	1900 4700 2000 4700
Wire Wire Line
	1900 5300 3900 5300
NoConn ~ 3800 3900
NoConn ~ 3800 3800
NoConn ~ 3800 3700
NoConn ~ 3800 3600
NoConn ~ 3800 3500
NoConn ~ 3800 3400
Connection ~ 6150 1300
Wire Wire Line
	6150 1300 6250 1300
Wire Wire Line
	3500 6900 3600 6900
$Comp
L Diode:BAV99 D2
U 1 1 61F3EB28
P 9900 5100
F 0 "D2" V 9854 5179 50  0000 L CNN
F 1 "BAV99" V 9945 5179 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 9900 4600 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/BAV99_SER.pdf" H 9900 5100 50  0001 C CNN
F 4 "SOT-23-3" H 9900 5100 50  0001 C CNN "JLC"
F 5 "C2500" H 9900 5100 50  0001 C CNN "LCSC"
F 6 "BAV 99 SMD" H 9900 5100 50  0001 C CNN "Reichelt"
	1    9900 5100
	0    1    1    0   
$EndComp
NoConn ~ 9700 5100
$Comp
L power:+3V0 #PWR0101
U 1 1 61F8F116
P 1800 6300
F 0 "#PWR0101" H 1800 6150 50  0001 C CNN
F 1 "+3V0" H 1815 6473 50  0000 C CNN
F 2 "" H 1800 6300 50  0001 C CNN
F 3 "" H 1800 6300 50  0001 C CNN
	1    1800 6300
	1    0    0    -1  
$EndComp
$Comp
L power:+3V0 #PWR0102
U 1 1 61F8F6B8
P 700 2600
F 0 "#PWR0102" H 700 2450 50  0001 C CNN
F 1 "+3V0" H 715 2773 50  0000 C CNN
F 2 "" H 700 2600 50  0001 C CNN
F 3 "" H 700 2600 50  0001 C CNN
	1    700  2600
	1    0    0    -1  
$EndComp
$Comp
L power:+3V0 #PWR0103
U 1 1 61F8FD6A
P 5700 3000
F 0 "#PWR0103" H 5700 2850 50  0001 C CNN
F 1 "+3V0" H 5715 3173 50  0000 C CNN
F 2 "" H 5700 3000 50  0001 C CNN
F 3 "" H 5700 3000 50  0001 C CNN
	1    5700 3000
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x02_Odd_Even J9
U 1 1 61F71F16
P 1400 3500
F 0 "J9" H 1450 3717 50  0000 C CNN
F 1 "dnp" H 1450 3626 50  0000 C CNN
F 2 "Connector_PinHeader_1.27mm:PinHeader_2x02_P1.27mm_Vertical" H 1400 3500 50  0001 C CNN
F 3 "~" H 1400 3500 50  0001 C CNN
	1    1400 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 3500 1800 3500
Wire Wire Line
	1800 3500 1900 3600
Wire Wire Line
	1700 3600 1800 3600
Wire Wire Line
	1800 3600 1900 3500
Wire Wire Line
	2000 3600 1900 3600
Wire Wire Line
	2000 3500 1900 3500
Wire Wire Line
	1100 3500 1200 3500
Wire Wire Line
	1200 3600 1100 3600
Wire Wire Line
	1100 3600 1100 5400
Connection ~ 1100 5400
Wire Wire Line
	1100 5400 2900 5400
Wire Wire Line
	700  2600 700  2800
Wire Wire Line
	1100 3500 1100 3100
Connection ~ 1100 3100
Wire Wire Line
	1100 3100 1000 3100
$Comp
L rt_RF_Module:E104-BT5032A U2
U 1 1 61EBB2B6
P 2900 4200
F 0 "U2" H 2900 5381 50  0000 C CNN
F 1 "E104-BT5032A" H 3350 5150 50  0000 C CNN
F 2 "rt_modules:E104-BT5032A" H 2900 4650 50  0001 C CNN
F 3 "https://www.ebyte.com/en/pdf-down.aspx?id=1511" H 2900 4650 50  0001 C CNN
F 4 "WIRELM-SMD" H 2900 4200 50  0001 C CNN "JLC"
F 5 "C518912" H 2900 4200 50  0001 C CNN "LCSC"
	1    2900 4200
	1    0    0    -1  
$EndComp
$Comp
L power:+3V0 #PWR0104
U 1 1 6200E0DA
P 5950 6550
F 0 "#PWR0104" H 5950 6400 50  0001 C CNN
F 1 "+3V0" H 5965 6723 50  0000 C CNN
F 2 "" H 5950 6550 50  0001 C CNN
F 3 "" H 5950 6550 50  0001 C CNN
	1    5950 6550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 6950 5950 6950
Wire Wire Line
	5950 6950 5950 6550
Wire Notes Line
	2800 6000 2800 7800
Wire Notes Line
	5200 6000 5200 7800
Text Notes 2400 6150 0    50   ~ 0
com pin
Text Notes 4650 6150 0    50   ~ 0
PWM output
Text Notes 6050 6150 0    50   ~ 0
Helena driver board connector
Wire Notes Line
	500  6000 7300 6000
Wire Notes Line
	500  2200 11200 2200
Wire Notes Line
	4500 500  4500 6000
Wire Notes Line
	7300 6500 7300 500 
Text Notes 3700 2400 0    50   ~ 0
nrf52832 module
Text Notes 7100 2350 0    50   ~ 0
IMU
Text Notes 3700 650  0    50   ~ 0
local power supply
Text Notes 7200 700  2    50   ~ 0
input voltage \nmeasurement
Text Notes 11100 2350 2    50   ~ 0
LED current regulator
Text Notes 11100 650  2    50   ~ 0
HMI
Wire Notes Line
	5550 3750 5650 3750
Wire Notes Line
	5650 3750 5650 3850
Wire Notes Line
	5650 3850 5550 3850
Text Notes 4700 4700 0    50   ~ 0
connect SD0 to \nGND manually!
Wire Notes Line
	5550 3850 5550 3750
Wire Notes Line
	5550 3800 5500 3800
Wire Notes Line
	5500 3800 5300 4500
$EndSCHEMATC
