EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 2
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Regulator_Linear:AZ1117-3.3 U1
U 1 1 608766F4
P 2800 850
F 0 "U1" H 2800 1092 50  0000 C CNN
F 1 "AZ1117-3.3" H 2800 1001 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-223-3_TabPin2" H 2800 1100 50  0001 C CIN
F 3 "https://www.diodes.com/assets/Datasheets/AZ1117.pdf" H 2800 850 50  0001 C CNN
	1    2800 850 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR02
U 1 1 6087868B
P 750 1000
F 0 "#PWR02" H 750 750 50  0001 C CNN
F 1 "GND" H 755 827 50  0000 C CNN
F 2 "" H 750 1000 50  0001 C CNN
F 3 "" H 750 1000 50  0001 C CNN
	1    750  1000
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR01
U 1 1 608792E6
P 650 800
F 0 "#PWR01" H 650 650 50  0001 C CNN
F 1 "+12V" H 665 973 50  0000 C CNN
F 2 "" H 650 800 50  0001 C CNN
F 3 "" H 650 800 50  0001 C CNN
	1    650  800 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR011
U 1 1 6087D954
P 2800 1400
F 0 "#PWR011" H 2800 1150 50  0001 C CNN
F 1 "GND" H 2805 1227 50  0000 C CNN
F 2 "" H 2800 1400 50  0001 C CNN
F 3 "" H 2800 1400 50  0001 C CNN
	1    2800 1400
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR09
U 1 1 6087E175
P 2400 850
F 0 "#PWR09" H 2400 700 50  0001 C CNN
F 1 "+12V" H 2415 1023 50  0000 C CNN
F 2 "" H 2400 850 50  0001 C CNN
F 3 "" H 2400 850 50  0001 C CNN
	1    2400 850 
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 850  2500 850 
$Comp
L power:+3V3 #PWR013
U 1 1 6087F073
P 3200 850
F 0 "#PWR013" H 3200 700 50  0001 C CNN
F 1 "+3V3" H 3215 1023 50  0000 C CNN
F 2 "" H 3200 850 50  0001 C CNN
F 3 "" H 3200 850 50  0001 C CNN
	1    3200 850 
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 6087FED7
P 2400 1250
F 0 "C2" H 2515 1296 50  0000 L CNN
F 1 "22u" H 2515 1205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 2438 1100 50  0001 C CNN
F 3 "~" H 2400 1250 50  0001 C CNN
	1    2400 1250
	1    0    0    -1  
$EndComp
Connection ~ 2400 850 
Wire Wire Line
	2400 850  2400 1100
Wire Wire Line
	2400 1400 2800 1400
Wire Wire Line
	2800 1150 2800 1400
Connection ~ 2800 1400
$Comp
L Device:C C4
U 1 1 60881C2E
P 3500 1500
F 0 "C4" H 3615 1546 50  0000 L CNN
F 1 "22u" H 3615 1455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 3538 1350 50  0001 C CNN
F 3 "~" H 3500 1500 50  0001 C CNN
	1    3500 1500
	1    0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 60882533
P 3900 1500
F 0 "C6" H 4015 1546 50  0000 L CNN
F 1 "0.1u" H 4015 1455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 3938 1350 50  0001 C CNN
F 3 "~" H 3900 1500 50  0001 C CNN
	1    3900 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 850  3200 850 
$Comp
L A4:ESP32-S2-WROOM U3
U 1 1 608957CE
P 5050 1800
F 0 "U3" H 5200 2781 50  0000 C CNN
F 1 "ESP32-S2-WROOM" H 5200 2690 50  0000 C CNN
F 2 "A4:ESP32-S2-WROOM" H 5050 1800 50  0001 C CNN
F 3 "" H 5050 1800 50  0001 C CNN
	1    5050 1800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR024
U 1 1 608982CF
P 4550 1200
F 0 "#PWR024" H 4550 950 50  0001 C CNN
F 1 "GND" V 4555 1072 50  0000 R CNN
F 2 "" H 4550 1200 50  0001 C CNN
F 3 "" H 4550 1200 50  0001 C CNN
	1    4550 1200
	0    1    1    0   
$EndComp
Wire Wire Line
	4550 1000 4550 1200
Wire Wire Line
	5850 1000 5850 1200
$Comp
L power:+3V3 #PWR022
U 1 1 608996D9
P 3900 1300
F 0 "#PWR022" H 3900 1150 50  0001 C CNN
F 1 "+3V3" H 3915 1473 50  0000 C CNN
F 2 "" H 3900 1300 50  0001 C CNN
F 3 "" H 3900 1300 50  0001 C CNN
	1    3900 1300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR026
U 1 1 60899FEA
P 5650 3150
F 0 "#PWR026" H 5650 2900 50  0001 C CNN
F 1 "GND" H 5655 2977 50  0000 C CNN
F 2 "" H 5650 3150 50  0001 C CNN
F 3 "" H 5650 3150 50  0001 C CNN
	1    5650 3150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C8
U 1 1 6089A7F1
P 6450 1450
F 0 "C8" H 6565 1496 50  0000 L CNN
F 1 "1u" H 6565 1405 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 6488 1300 50  0001 C CNN
F 3 "~" H 6450 1450 50  0001 C CNN
	1    6450 1450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR029
U 1 1 6089B4F4
P 6450 1600
F 0 "#PWR029" H 6450 1350 50  0001 C CNN
F 1 "GND" H 6455 1427 50  0000 C CNN
F 2 "" H 6450 1600 50  0001 C CNN
F 3 "" H 6450 1600 50  0001 C CNN
	1    6450 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 1300 6450 1300
Text GLabel 6550 1300 2    50   Input ~ 0
EN
$Comp
L Device:R R6
U 1 1 6089C42D
P 6450 1050
F 0 "R6" H 6520 1096 50  0000 L CNN
F 1 "10K" H 6520 1005 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 6380 1050 50  0001 C CNN
F 3 "~" H 6450 1050 50  0001 C CNN
	1    6450 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 1200 6450 1300
Connection ~ 6450 1300
$Comp
L power:+3V3 #PWR028
U 1 1 6089D772
P 6450 900
F 0 "#PWR028" H 6450 750 50  0001 C CNN
F 1 "+3V3" H 6465 1073 50  0000 C CNN
F 2 "" H 6450 900 50  0001 C CNN
F 3 "" H 6450 900 50  0001 C CNN
	1    6450 900 
	1    0    0    -1  
$EndComp
Text GLabel 5850 1600 2    50   Input ~ 0
TX
Text GLabel 5850 1700 2    50   Input ~ 0
RX
$Comp
L Device:R R7
U 1 1 6089E963
P 7050 1050
F 0 "R7" H 7120 1096 50  0000 L CNN
F 1 "47K" H 7120 1005 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 6980 1050 50  0001 C CNN
F 3 "~" H 7050 1050 50  0001 C CNN
	1    7050 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 1300 7050 1200
Wire Wire Line
	7050 900  6450 900 
Connection ~ 6450 900 
Text GLabel 4550 1400 0    50   Input ~ 0
IO0
Text GLabel 4550 2700 0    50   Input ~ 0
CHARGE
Text GLabel 4550 2400 0    50   Input ~ 0
ENCAB
Text GLabel 4550 2300 0    50   Input ~ 0
ENCAA
Wire Wire Line
	6550 1300 6450 1300
Text GLabel 5150 3150 3    50   Input ~ 0
DAC2
$Comp
L power:+12V #PWR05
U 1 1 608A87BE
P 1150 1500
F 0 "#PWR05" H 1150 1350 50  0001 C CNN
F 1 "+12V" H 1165 1673 50  0000 C CNN
F 2 "" H 1150 1500 50  0001 C CNN
F 3 "" H 1150 1500 50  0001 C CNN
	1    1150 1500
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 608A96F1
P 1150 1650
F 0 "R2" H 1220 1696 50  0000 L CNN
F 1 "47k" H 1220 1605 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 1080 1650 50  0001 C CNN
F 3 "~" H 1150 1650 50  0001 C CNN
	1    1150 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 608A9DE9
P 1150 1950
F 0 "R3" H 1220 1996 50  0000 L CNN
F 1 "10K" H 1220 1905 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 1080 1950 50  0001 C CNN
F 3 "~" H 1150 1950 50  0001 C CNN
	1    1150 1950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 608AA270
P 1150 2100
F 0 "#PWR06" H 1150 1850 50  0001 C CNN
F 1 "GND" H 1155 1927 50  0000 C CNN
F 2 "" H 1150 2100 50  0001 C CNN
F 3 "" H 1150 2100 50  0001 C CNN
	1    1150 2100
	1    0    0    -1  
$EndComp
Text GLabel 1100 1800 0    50   Input ~ 0
CHARGE
Wire Wire Line
	1100 1800 1150 1800
Connection ~ 1150 1800
$Comp
L A4:TB67H451FNG,EL U2
U 1 1 608B1F4C
P 1600 4050
F 0 "U2" H 1600 4465 50  0000 C CNN
F 1 "TB67H451FNG,EL" H 1600 4374 50  0000 C CNN
F 2 "Package_SO:SOIC-8-1EP_3.9x4.9mm_P1.27mm_EP2.29x3mm" H 1600 4050 50  0001 C CNN
F 3 "" H 1600 4050 50  0001 C CNN
	1    1600 4050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 608B3E1B
P 2550 4350
F 0 "C5" H 2665 4396 50  0000 L CNN
F 1 "0.1u" H 2665 4305 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 2588 4200 50  0001 C CNN
F 3 "~" H 2550 4350 50  0001 C CNN
	1    2550 4350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR021
U 1 1 608B4891
P 2550 4500
F 0 "#PWR021" H 2550 4250 50  0001 C CNN
F 1 "GND" H 2555 4327 50  0000 C CNN
F 2 "" H 2550 4500 50  0001 C CNN
F 3 "" H 2550 4500 50  0001 C CNN
	1    2550 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 4500 2100 4500
Connection ~ 2550 4500
$Comp
L power:+12V #PWR020
U 1 1 608B5F1B
P 2550 4200
F 0 "#PWR020" H 2550 4050 50  0001 C CNN
F 1 "+12V" H 2565 4373 50  0000 C CNN
F 2 "" H 2550 4200 50  0001 C CNN
F 3 "" H 2550 4200 50  0001 C CNN
	1    2550 4200
	1    0    0    -1  
$EndComp
Connection ~ 2550 4200
Text GLabel 1950 4000 2    50   Input ~ 0
RSA
Text GLabel 2000 4750 2    50   Input ~ 0
RSA
$Comp
L Device:R R4
U 1 1 608B6A6E
P 1950 4900
F 0 "R4" H 2020 4946 50  0000 L CNN
F 1 "0.1" H 2020 4855 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 1880 4900 50  0001 C CNN
F 3 "~" H 1950 4900 50  0001 C CNN
	1    1950 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 4750 2000 4750
$Comp
L power:GND #PWR014
U 1 1 608B7A08
P 1950 5050
F 0 "#PWR014" H 1950 4800 50  0001 C CNN
F 1 "GND" H 1955 4877 50  0000 C CNN
F 2 "" H 1950 5050 50  0001 C CNN
F 3 "" H 1950 5050 50  0001 C CNN
	1    1950 5050
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x02 J7
U 1 1 608B7EBA
P 2650 3800
F 0 "J7" H 2730 3792 50  0000 L CNN
F 1 "Screw_Terminal_01x02" H 2730 3701 50  0000 L CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2_1x02_P5.00mm_Horizontal" H 2650 3800 50  0001 C CNN
F 3 "~" H 2650 3800 50  0001 C CNN
	1    2650 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 3800 1950 3800
Wire Wire Line
	1950 3800 1950 3900
Wire Wire Line
	1950 4100 2200 4100
Wire Wire Line
	2200 4100 2200 3900
Wire Wire Line
	2200 3900 2450 3900
$Comp
L power:GND #PWR010
U 1 1 608BB1B1
P 1250 3900
F 0 "#PWR010" H 1250 3650 50  0001 C CNN
F 1 "GND" V 1255 3772 50  0000 R CNN
F 2 "" H 1250 3900 50  0001 C CNN
F 3 "" H 1250 3900 50  0001 C CNN
	1    1250 3900
	0    1    1    0   
$EndComp
Text GLabel 5850 2000 2    50   Input ~ 0
MOTA2
Text GLabel 5850 1900 2    50   Input ~ 0
MOTA1
Text GLabel 1250 4100 0    50   Input ~ 0
MOTA1
Text GLabel 1250 4000 0    50   Input ~ 0
MOTA2
Wire Wire Line
	3900 1300 3900 1350
Wire Wire Line
	3500 1350 3900 1350
Connection ~ 3900 1350
$Comp
L power:GND #PWR023
U 1 1 608BF807
P 3900 1650
F 0 "#PWR023" H 3900 1400 50  0001 C CNN
F 1 "GND" H 3905 1477 50  0000 C CNN
F 2 "" H 3900 1650 50  0001 C CNN
F 3 "" H 3900 1650 50  0001 C CNN
	1    3900 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 1650 3900 1650
Connection ~ 3900 1650
Wire Wire Line
	4550 1300 3900 1300
Connection ~ 3900 1300
Text GLabel 5050 3150 3    50   Input ~ 0
DAC1
Text GLabel 1250 4200 0    50   Input ~ 0
DAC1
Wire Wire Line
	7550 1300 7050 1300
$Comp
L A4:TB67H451FNG,EL U4
U 1 1 6088D5A8
P 4750 4200
F 0 "U4" H 4750 4615 50  0000 C CNN
F 1 "TB67H451FNG,EL" H 4750 4524 50  0000 C CNN
F 2 "Package_SO:SOIC-8-1EP_3.9x4.9mm_P1.27mm_EP2.29x3mm" H 4750 4200 50  0001 C CNN
F 3 "" H 4750 4200 50  0001 C CNN
	1    4750 4200
	1    0    0    -1  
$EndComp
$Comp
L Device:C C9
U 1 1 6088D5D4
P 5700 4500
F 0 "C9" H 5815 4546 50  0000 L CNN
F 1 "0.1u" H 5815 4455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 5738 4350 50  0001 C CNN
F 3 "~" H 5700 4500 50  0001 C CNN
	1    5700 4500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR031
U 1 1 6088D5DE
P 5700 4650
F 0 "#PWR031" H 5700 4400 50  0001 C CNN
F 1 "GND" H 5705 4477 50  0000 C CNN
F 2 "" H 5700 4650 50  0001 C CNN
F 3 "" H 5700 4650 50  0001 C CNN
	1    5700 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 4650 5250 4650
Connection ~ 5700 4650
$Comp
L power:+12V #PWR030
U 1 1 6088D5ED
P 5700 4350
F 0 "#PWR030" H 5700 4200 50  0001 C CNN
F 1 "+12V" H 5715 4523 50  0000 C CNN
F 2 "" H 5700 4350 50  0001 C CNN
F 3 "" H 5700 4350 50  0001 C CNN
	1    5700 4350
	1    0    0    -1  
$EndComp
Connection ~ 5700 4350
Text GLabel 5100 4150 2    50   Input ~ 0
RSB
$Comp
L Device:R R5
U 1 1 6088D5FA
P 5100 5050
F 0 "R5" H 5170 5096 50  0000 L CNN
F 1 "0.1" H 5170 5005 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 5030 5050 50  0001 C CNN
F 3 "~" H 5100 5050 50  0001 C CNN
	1    5100 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 4900 5150 4900
$Comp
L power:GND #PWR027
U 1 1 6088D605
P 5100 5200
F 0 "#PWR027" H 5100 4950 50  0001 C CNN
F 1 "GND" H 5105 5027 50  0000 C CNN
F 2 "" H 5100 5200 50  0001 C CNN
F 3 "" H 5100 5200 50  0001 C CNN
	1    5100 5200
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x02 J8
U 1 1 6088D60F
P 5800 3950
F 0 "J8" H 5880 3942 50  0000 L CNN
F 1 "Screw_Terminal_01x02" H 5880 3851 50  0000 L CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2_1x02_P5.00mm_Horizontal" H 5800 3950 50  0001 C CNN
F 3 "~" H 5800 3950 50  0001 C CNN
	1    5800 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 3950 5100 3950
Wire Wire Line
	5100 3950 5100 4050
Wire Wire Line
	5100 4250 5350 4250
Wire Wire Line
	5350 4250 5350 4050
Wire Wire Line
	5350 4050 5600 4050
$Comp
L power:GND #PWR025
U 1 1 6088D61E
P 4400 4050
F 0 "#PWR025" H 4400 3800 50  0001 C CNN
F 1 "GND" V 4405 3922 50  0000 R CNN
F 2 "" H 4400 4050 50  0001 C CNN
F 3 "" H 4400 4050 50  0001 C CNN
	1    4400 4050
	0    1    1    0   
$EndComp
Text GLabel 4400 4250 0    50   Input ~ 0
MOTB1
Text GLabel 4400 4150 0    50   Input ~ 0
MOTB2
Text GLabel 4400 4350 0    50   Input ~ 0
DAC2
Text GLabel 5150 4900 2    50   Input ~ 0
RSB
$Comp
L Connector:Conn_01x06_Male J13
U 1 1 608A6E01
P 900 5950
F 0 "J13" H 1008 6331 50  0000 C CNN
F 1 "Conn_01x06_Male" H 1008 6240 50  0000 C CNN
F 2 "Connector_JST:JST_XH_B6B-XH-A_1x06_P2.50mm_Vertical" H 900 5950 50  0001 C CNN
F 3 "~" H 900 5950 50  0001 C CNN
	1    900  5950
	1    0    0    -1  
$EndComp
Text GLabel 1100 5950 2    50   Input ~ 0
ENCAA_5V
Text GLabel 1100 6150 2    50   Input ~ 0
ENCAB_5V
$Comp
L power:GND #PWR039
U 1 1 608A859B
P 1650 6300
F 0 "#PWR039" H 1650 6050 50  0001 C CNN
F 1 "GND" H 1655 6127 50  0000 C CNN
F 2 "" H 1650 6300 50  0001 C CNN
F 3 "" H 1650 6300 50  0001 C CNN
	1    1650 6300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1100 6250 1650 6250
Wire Wire Line
	1650 6250 1650 6300
Wire Wire Line
	1650 6250 1650 6050
Wire Wire Line
	1650 6050 1100 6050
Connection ~ 1650 6250
Wire Wire Line
	1650 5850 1650 6050
Connection ~ 1650 6050
Wire Wire Line
	1100 5750 1650 5750
$Comp
L Regulator_Linear:AZ1117-5.0 U5
U 1 1 6087BE91
P 9200 950
F 0 "U5" H 9200 1192 50  0000 C CNN
F 1 "AZ1117-5.0" H 9200 1101 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-223-3_TabPin2" H 9200 1200 50  0001 C CIN
F 3 "https://www.diodes.com/assets/Datasheets/AZ1117.pdf" H 9200 950 50  0001 C CNN
	1    9200 950 
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR037
U 1 1 6087E455
P 8700 950
F 0 "#PWR037" H 8700 800 50  0001 C CNN
F 1 "+12V" H 8715 1123 50  0000 C CNN
F 2 "" H 8700 950 50  0001 C CNN
F 3 "" H 8700 950 50  0001 C CNN
	1    8700 950 
	1    0    0    -1  
$EndComp
$Comp
L Device:C C10
U 1 1 6087EF9B
P 8700 1100
F 0 "C10" H 8815 1146 50  0000 L CNN
F 1 "22u" H 8815 1055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 8738 950 50  0001 C CNN
F 3 "~" H 8700 1100 50  0001 C CNN
	1    8700 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	8700 950  8900 950 
Connection ~ 8700 950 
$Comp
L power:GND #PWR041
U 1 1 608810A8
P 9200 1250
F 0 "#PWR041" H 9200 1000 50  0001 C CNN
F 1 "GND" H 9205 1077 50  0000 C CNN
F 2 "" H 9200 1250 50  0001 C CNN
F 3 "" H 9200 1250 50  0001 C CNN
	1    9200 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	9200 1250 8700 1250
Connection ~ 9200 1250
$Comp
L power:+5V #PWR042
U 1 1 608832A2
P 9550 950
F 0 "#PWR042" H 9550 800 50  0001 C CNN
F 1 "+5V" H 9565 1123 50  0000 C CNN
F 2 "" H 9550 950 50  0001 C CNN
F 3 "" H 9550 950 50  0001 C CNN
	1    9550 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	9550 950  9500 950 
$Comp
L power:+5V #PWR038
U 1 1 608854D5
P 1650 5750
F 0 "#PWR038" H 1650 5600 50  0001 C CNN
F 1 "+5V" H 1665 5923 50  0000 C CNN
F 2 "" H 1650 5750 50  0001 C CNN
F 3 "" H 1650 5750 50  0001 C CNN
	1    1650 5750
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:2N7002 Q3
U 1 1 60889662
P 1950 7250
F 0 "Q3" H 2154 7296 50  0000 L CNN
F 1 "2N7002" H 2154 7205 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 2150 7175 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 1950 7250 50  0001 L CNN
	1    1950 7250
	1    0    0    -1  
$EndComp
Text GLabel 1750 7250 0    50   Input ~ 0
ENCAB_5V
$Comp
L power:GND #PWR047
U 1 1 6088C5DC
P 2050 7450
F 0 "#PWR047" H 2050 7200 50  0001 C CNN
F 1 "GND" H 2055 7277 50  0000 C CNN
F 2 "" H 2050 7450 50  0001 C CNN
F 3 "" H 2050 7450 50  0001 C CNN
	1    2050 7450
	1    0    0    -1  
$EndComp
Text GLabel 2100 7000 2    50   Input ~ 0
ENCAB
$Comp
L power:+3V3 #PWR046
U 1 1 6088D269
P 2050 6600
F 0 "#PWR046" H 2050 6450 50  0001 C CNN
F 1 "+3V3" H 2065 6773 50  0000 C CNN
F 2 "" H 2050 6600 50  0001 C CNN
F 3 "" H 2050 6600 50  0001 C CNN
	1    2050 6600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 6900 2050 7000
Wire Wire Line
	2100 7000 2050 7000
Connection ~ 2050 7000
Wire Wire Line
	2050 7000 2050 7050
$Comp
L Device:R R9
U 1 1 60890C15
P 2050 6750
F 0 "R9" H 2120 6796 50  0000 L CNN
F 1 "10K" H 2120 6705 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 1980 6750 50  0001 C CNN
F 3 "~" H 2050 6750 50  0001 C CNN
	1    2050 6750
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:2N7002 Q2
U 1 1 608927F6
P 2700 6150
F 0 "Q2" H 2904 6196 50  0000 L CNN
F 1 "2N7002" H 2904 6105 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 2900 6075 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 2700 6150 50  0001 L CNN
	1    2700 6150
	1    0    0    -1  
$EndComp
Text GLabel 2500 6150 0    50   Input ~ 0
ENCAA_5V
$Comp
L power:GND #PWR045
U 1 1 60892801
P 2800 6350
F 0 "#PWR045" H 2800 6100 50  0001 C CNN
F 1 "GND" H 2805 6177 50  0000 C CNN
F 2 "" H 2800 6350 50  0001 C CNN
F 3 "" H 2800 6350 50  0001 C CNN
	1    2800 6350
	1    0    0    -1  
$EndComp
Text GLabel 2850 5900 2    50   Input ~ 0
ENCAA
$Comp
L power:+3V3 #PWR044
U 1 1 6089280C
P 2800 5500
F 0 "#PWR044" H 2800 5350 50  0001 C CNN
F 1 "+3V3" H 2815 5673 50  0000 C CNN
F 2 "" H 2800 5500 50  0001 C CNN
F 3 "" H 2800 5500 50  0001 C CNN
	1    2800 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 5800 2800 5900
Wire Wire Line
	2850 5900 2800 5900
Connection ~ 2800 5900
Wire Wire Line
	2800 5900 2800 5950
$Comp
L Device:R R8
U 1 1 6089281A
P 2800 5650
F 0 "R8" H 2870 5696 50  0000 L CNN
F 1 "10K" H 2870 5605 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 2730 5650 50  0001 C CNN
F 3 "~" H 2800 5650 50  0001 C CNN
	1    2800 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1100 5850 1650 5850
Text GLabel 3800 3350 0    50   Input ~ 0
EN0
Text GLabel 3800 3250 0    50   Input ~ 0
STEP0
Text GLabel 3800 3150 0    50   Input ~ 0
DIR0
Wire Wire Line
	5100 4350 5250 4350
$Comp
L Device:CP C7
U 1 1 608B3B57
P 5250 4500
F 0 "C7" H 5368 4546 50  0000 L CNN
F 1 "100u" H 5368 4455 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_6.3x7.7" H 5288 4350 50  0001 C CNN
F 3 "~" H 5250 4500 50  0001 C CNN
	1    5250 4500
	1    0    0    -1  
$EndComp
Connection ~ 5250 4350
Wire Wire Line
	5250 4350 5700 4350
Wire Wire Line
	1950 4200 2100 4200
$Comp
L Device:CP C3
U 1 1 608B44B2
P 2100 4350
F 0 "C3" H 2218 4396 50  0000 L CNN
F 1 "100u" H 2218 4305 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_6.3x7.7" H 2138 4200 50  0001 C CNN
F 3 "~" H 2100 4350 50  0001 C CNN
	1    2100 4350
	1    0    0    -1  
$EndComp
Connection ~ 2100 4200
Wire Wire Line
	2100 4200 2550 4200
$Comp
L Connector:Conn_01x06_Male J15
U 1 1 608914D1
P 2650 2250
F 0 "J15" H 2622 2132 50  0000 R CNN
F 1 "Conn_01x06_Male" H 2622 2223 50  0000 R CNN
F 2 "Connector_JST:JST_PH_B6B-PH-K_1x06_P2.00mm_Vertical" H 2650 2250 50  0001 C CNN
F 3 "~" H 2650 2250 50  0001 C CNN
	1    2650 2250
	-1   0    0    1   
$EndComp
Text GLabel 2450 1950 0    50   Input ~ 0
TX
Text GLabel 2450 2050 0    50   Input ~ 0
RX
Text GLabel 2450 2250 0    50   Input ~ 0
EN
Text GLabel 2450 2350 0    50   Input ~ 0
IO0
$Comp
L power:GND #PWR0102
U 1 1 6089256A
P 2450 2450
F 0 "#PWR0102" H 2450 2200 50  0001 C CNN
F 1 "GND" H 2455 2277 50  0000 C CNN
F 2 "" H 2450 2450 50  0001 C CNN
F 3 "" H 2450 2450 50  0001 C CNN
	1    2450 2450
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0103
U 1 1 60892C43
P 2150 2150
F 0 "#PWR0103" H 2150 2000 50  0001 C CNN
F 1 "+3V3" H 2165 2323 50  0000 C CNN
F 2 "" H 2150 2150 50  0001 C CNN
F 3 "" H 2150 2150 50  0001 C CNN
	1    2150 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 2150 2450 2150
Text GLabel 900  2900 0    50   Input ~ 0
EN
$Comp
L dk_Tactile-Switches:TL3315NF160Q S2
U 1 1 608AAFAA
P 1300 3000
F 0 "S2" H 1300 3347 60  0000 C CNN
F 1 "TL3315NF160Q" H 1300 3241 60  0000 C CNN
F 2 "digikey-footprints:Switch_Tactile_SMD_4.5x4.5mm_TL3315NF160Q" H 1500 3200 60  0001 L CNN
F 3 "http://spec_sheets.e-switch.com/specs/P010337.pdf" H 1500 3300 60  0001 L CNN
F 4 "EG4621CT-ND" H 1500 3400 60  0001 L CNN "Digi-Key_PN"
F 5 "TL3315NF160Q" H 1500 3500 60  0001 L CNN "MPN"
F 6 "Switches" H 1500 3600 60  0001 L CNN "Category"
F 7 "Tactile Switches" H 1500 3700 60  0001 L CNN "Family"
F 8 "http://spec_sheets.e-switch.com/specs/P010337.pdf" H 1500 3800 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/e-switch/TL3315NF160Q/EG4621CT-ND/1870401" H 1500 3900 60  0001 L CNN "DK_Detail_Page"
F 10 "SWITCH TACTILE SPST-NO 0.05A 15V" H 1500 4000 60  0001 L CNN "Description"
F 11 "E-Switch" H 1500 4100 60  0001 L CNN "Manufacturer"
F 12 "Active" H 1500 4200 60  0001 L CNN "Status"
	1    1300 3000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 608B4405
P 1650 3100
F 0 "#PWR0105" H 1650 2850 50  0001 C CNN
F 1 "GND" H 1655 2927 50  0000 C CNN
F 2 "" H 1650 3100 50  0001 C CNN
F 3 "" H 1650 3100 50  0001 C CNN
	1    1650 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 3100 1650 3100
Wire Wire Line
	1100 2900 900  2900
$Comp
L power:GND #PWR0106
U 1 1 608D6BFE
P 1600 4400
F 0 "#PWR0106" H 1600 4150 50  0001 C CNN
F 1 "GND" V 1605 4272 50  0000 R CNN
F 2 "" H 1600 4400 50  0001 C CNN
F 3 "" H 1600 4400 50  0001 C CNN
	1    1600 4400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0107
U 1 1 608D71F5
P 4750 4550
F 0 "#PWR0107" H 4750 4300 50  0001 C CNN
F 1 "GND" V 4755 4422 50  0000 R CNN
F 2 "" H 4750 4550 50  0001 C CNN
F 3 "" H 4750 4550 50  0001 C CNN
	1    4750 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 1000 5850 1000
Wire Wire Line
	750  800  650  800 
Text GLabel 3800 2650 0    50   Input ~ 0
MOTB1
Text GLabel 3800 2750 0    50   Input ~ 0
MOTB2
Text GLabel 4550 2500 0    50   Input ~ 0
ENCBA
Text GLabel 4550 2600 0    50   Input ~ 0
ENCBB
Text GLabel 7950 1200 2    50   Input ~ 0
DAC2
$Comp
L dk_Slide-Switches:JS202011SCQN S1
U 1 1 60879F2F
P 7750 1500
F 0 "S1" H 7750 1983 50  0000 C CNN
F 1 "JS202011SCQN" H 7750 1892 50  0000 C CNN
F 2 "digikey-footprints:Switch_Slide_JS202011SCQN" H 7950 1700 50  0001 L CNN
F 3 "https://www.ckswitches.com/media/1422/js.pdf" H 7950 1800 60  0001 L CNN
F 4 "401-2002-1-ND" H 7950 1900 60  0001 L CNN "Digi-Key_PN"
F 5 "JS202011SCQN" H 7950 2000 60  0001 L CNN "MPN"
F 6 "Switches" H 7950 2100 60  0001 L CNN "Category"
F 7 "Slide Switches" H 7950 2200 60  0001 L CNN "Family"
F 8 "https://www.ckswitches.com/media/1422/js.pdf" H 7950 2300 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/c-k/JS202011SCQN/401-2002-1-ND/1640098" H 7950 2400 60  0001 L CNN "DK_Detail_Page"
F 10 "SWITCH SLIDE DPDT 300MA 6V" H 7950 2500 60  0001 L CNN "Description"
F 11 "C&K" H 7950 2600 60  0001 L CNN "Manufacturer"
F 12 "Active" H 7950 2700 60  0001 L CNN "Status"
	1    7750 1500
	1    0    0    -1  
$EndComp
Text GLabel 3800 3450 0    50   Input ~ 0
DIR1
Text GLabel 4750 3150 3    50   Input ~ 0
STEP1
Text GLabel 4850 3150 3    50   Input ~ 0
EN1
Text GLabel 4950 3150 3    50   Input ~ 0
DIR2
Text GLabel 5250 3150 3    50   Input ~ 0
STEP2
Text GLabel 5350 3150 3    50   Input ~ 0
EN2
Text GLabel 5450 3150 3    50   Input ~ 0
DIR3
Text GLabel 5550 3150 3    50   Input ~ 0
STEP3
Text GLabel 5850 2700 2    50   Input ~ 0
EN3
$Comp
L Connector:Conn_01x06_Male J3
U 1 1 608FB2D5
P 4000 5950
F 0 "J3" H 4108 6331 50  0000 C CNN
F 1 "Conn_01x06_Male" H 4108 6240 50  0000 C CNN
F 2 "Connector_JST:JST_XH_B6B-XH-A_1x06_P2.50mm_Vertical" H 4000 5950 50  0001 C CNN
F 3 "~" H 4000 5950 50  0001 C CNN
	1    4000 5950
	1    0    0    -1  
$EndComp
Text GLabel 4200 5950 2    50   Input ~ 0
ENCBA_5V
Text GLabel 4200 6150 2    50   Input ~ 0
ENCBB_5V
$Comp
L power:GND #PWR07
U 1 1 608FB2E1
P 4750 6300
F 0 "#PWR07" H 4750 6050 50  0001 C CNN
F 1 "GND" H 4755 6127 50  0000 C CNN
F 2 "" H 4750 6300 50  0001 C CNN
F 3 "" H 4750 6300 50  0001 C CNN
	1    4750 6300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 6250 4750 6250
Wire Wire Line
	4750 6250 4750 6300
Wire Wire Line
	4750 6250 4750 6050
Wire Wire Line
	4750 6050 4200 6050
Connection ~ 4750 6250
Wire Wire Line
	4750 5850 4750 6050
Connection ~ 4750 6050
Wire Wire Line
	4200 5750 4750 5750
$Comp
L power:+5V #PWR04
U 1 1 608FB2F3
P 4750 5750
F 0 "#PWR04" H 4750 5600 50  0001 C CNN
F 1 "+5V" H 4765 5923 50  0000 C CNN
F 2 "" H 4750 5750 50  0001 C CNN
F 3 "" H 4750 5750 50  0001 C CNN
	1    4750 5750
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:2N7002 Q1
U 1 1 608FB2FD
P 5050 7250
F 0 "Q1" H 5254 7296 50  0000 L CNN
F 1 "2N7002" H 5254 7205 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 5250 7175 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 5050 7250 50  0001 L CNN
	1    5050 7250
	1    0    0    -1  
$EndComp
Text GLabel 4850 7250 0    50   Input ~ 0
ENCBB_5V
$Comp
L power:GND #PWR012
U 1 1 608FB308
P 5150 7450
F 0 "#PWR012" H 5150 7200 50  0001 C CNN
F 1 "GND" H 5155 7277 50  0000 C CNN
F 2 "" H 5150 7450 50  0001 C CNN
F 3 "" H 5150 7450 50  0001 C CNN
	1    5150 7450
	1    0    0    -1  
$EndComp
Text GLabel 5200 7000 2    50   Input ~ 0
ENCBB
$Comp
L power:+3V3 #PWR08
U 1 1 608FB313
P 5150 6600
F 0 "#PWR08" H 5150 6450 50  0001 C CNN
F 1 "+3V3" H 5165 6773 50  0000 C CNN
F 2 "" H 5150 6600 50  0001 C CNN
F 3 "" H 5150 6600 50  0001 C CNN
	1    5150 6600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 6900 5150 7000
Wire Wire Line
	5200 7000 5150 7000
Connection ~ 5150 7000
Wire Wire Line
	5150 7000 5150 7050
$Comp
L Device:R R1
U 1 1 608FB321
P 5150 6750
F 0 "R1" H 5220 6796 50  0000 L CNN
F 1 "10K" H 5220 6705 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 5080 6750 50  0001 C CNN
F 3 "~" H 5150 6750 50  0001 C CNN
	1    5150 6750
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:2N7002 Q4
U 1 1 608FB32B
P 5800 6150
F 0 "Q4" H 6004 6196 50  0000 L CNN
F 1 "2N7002" H 6004 6105 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 6000 6075 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 5800 6150 50  0001 L CNN
	1    5800 6150
	1    0    0    -1  
$EndComp
Text GLabel 5600 6150 0    50   Input ~ 0
ENCBA_5V
$Comp
L power:GND #PWR016
U 1 1 608FB336
P 5900 6350
F 0 "#PWR016" H 5900 6100 50  0001 C CNN
F 1 "GND" H 5905 6177 50  0000 C CNN
F 2 "" H 5900 6350 50  0001 C CNN
F 3 "" H 5900 6350 50  0001 C CNN
	1    5900 6350
	1    0    0    -1  
$EndComp
Text GLabel 5950 5900 2    50   Input ~ 0
ENCBA
$Comp
L power:+3V3 #PWR015
U 1 1 608FB341
P 5900 5500
F 0 "#PWR015" H 5900 5350 50  0001 C CNN
F 1 "+3V3" H 5915 5673 50  0000 C CNN
F 2 "" H 5900 5500 50  0001 C CNN
F 3 "" H 5900 5500 50  0001 C CNN
	1    5900 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 5800 5900 5900
Wire Wire Line
	5950 5900 5900 5900
Connection ~ 5900 5900
Wire Wire Line
	5900 5900 5900 5950
$Comp
L Device:R R10
U 1 1 608FB34F
P 5900 5650
F 0 "R10" H 5970 5696 50  0000 L CNN
F 1 "10K" H 5970 5605 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 5830 5650 50  0001 C CNN
F 3 "~" H 5900 5650 50  0001 C CNN
	1    5900 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 5850 4750 5850
$Sheet
S 8600 1850 1100 350 
U 6090A275
F0 "STEPPERS" 50
F1 "A4_STEPPERS.sch" 50
$EndSheet
Text GLabel 6500 2700 2    50   Input ~ 0
HOM0
Text GLabel 6500 2600 2    50   Input ~ 0
HOM1
Text GLabel 6500 2500 2    50   Input ~ 0
HOM2
Text GLabel 6500 2400 2    50   Input ~ 0
HOM3
Text GLabel 6500 2300 2    50   Input ~ 0
HOMA
Text GLabel 6500 2200 2    50   Input ~ 0
HOMB
$Comp
L Connector:Conn_01x02_Male J2
U 1 1 609AF519
P 3000 4900
F 0 "J2" H 3108 5081 50  0000 C CNN
F 1 "Conn_01x02_Male" H 3108 4990 50  0000 C CNN
F 2 "Connector_JST:JST_XH_B2B-XH-A_1x02_P2.50mm_Vertical" H 3000 4900 50  0001 C CNN
F 3 "~" H 3000 4900 50  0001 C CNN
	1    3000 4900
	1    0    0    -1  
$EndComp
Text GLabel 3200 4900 2    50   Input ~ 0
HOMA
$Comp
L power:GND #PWR03
U 1 1 609B0E09
P 3200 5000
F 0 "#PWR03" H 3200 4750 50  0001 C CNN
F 1 "GND" H 3205 4827 50  0000 C CNN
F 2 "" H 3200 5000 50  0001 C CNN
F 3 "" H 3200 5000 50  0001 C CNN
	1    3200 5000
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Male J4
U 1 1 609B422F
P 6300 4850
F 0 "J4" H 6408 5031 50  0000 C CNN
F 1 "Conn_01x02_Male" H 6408 4940 50  0000 C CNN
F 2 "Connector_JST:JST_XH_B2B-XH-A_1x02_P2.50mm_Vertical" H 6300 4850 50  0001 C CNN
F 3 "~" H 6300 4850 50  0001 C CNN
	1    6300 4850
	1    0    0    -1  
$EndComp
Text GLabel 6500 4850 2    50   Input ~ 0
HOMB
$Comp
L power:GND #PWR017
U 1 1 609B4236
P 6500 4950
F 0 "#PWR017" H 6500 4700 50  0001 C CNN
F 1 "GND" H 6505 4777 50  0000 C CNN
F 2 "" H 6500 4950 50  0001 C CNN
F 3 "" H 6500 4950 50  0001 C CNN
	1    6500 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	750  900  750  1000
Connection ~ 750  1000
$Comp
L Connector:Barrel_Jack_Switch J1
U 1 1 60A08C81
P 1050 900
F 0 "J1" H 820 942 50  0000 R CNN
F 1 "Barrel_Jack_Switch" H 820 851 50  0000 R CNN
F 2 "Connector_BarrelJack:BarrelJack_Horizontal" H 1100 860 50  0001 C CNN
F 3 "~" H 1100 860 50  0001 C CNN
	1    1050 900 
	-1   0    0    -1  
$EndComp
Text GLabel 4550 1600 0    50   Input ~ 0
ENC3A
Text GLabel 4550 1500 0    50   Input ~ 0
ENC3B
Text GLabel 4550 1700 0    50   Input ~ 0
ENC2B
Text GLabel 4550 1800 0    50   Input ~ 0
ENC2A
Text GLabel 4550 1900 0    50   Input ~ 0
ENC1B
Text GLabel 4550 2000 0    50   Input ~ 0
ENC1A
Text GLabel 4550 2100 0    50   Input ~ 0
ENC0B
Text GLabel 4550 2200 0    50   Input ~ 0
ENC0A
$Comp
L Connector:Conn_01x04_Male J25
U 1 1 608ED1C5
P 8100 2500
AR Path="/608ED1C5" Ref="J25"  Part="1" 
AR Path="/6090A275/608ED1C5" Ref="J?"  Part="1" 
F 0 "J25" H 8208 2781 50  0000 C CNN
F 1 "Conn_01x04_Male" H 8208 2690 50  0000 C CNN
F 2 "Connector_JST:JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical" H 8100 2500 50  0001 C CNN
F 3 "~" H 8100 2500 50  0001 C CNN
	1    8100 2500
	-1   0    0    1   
$EndComp
$Comp
L power:+3V3 #PWR088
U 1 1 608EE90B
P 7900 2300
F 0 "#PWR088" H 7900 2150 50  0001 C CNN
F 1 "+3V3" H 7915 2473 50  0000 C CNN
F 2 "" H 7900 2300 50  0001 C CNN
F 3 "" H 7900 2300 50  0001 C CNN
	1    7900 2300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR087
U 1 1 608EF407
P 7550 2600
F 0 "#PWR087" H 7550 2350 50  0001 C CNN
F 1 "GND" H 7555 2427 50  0000 C CNN
F 2 "" H 7550 2600 50  0001 C CNN
F 3 "" H 7550 2600 50  0001 C CNN
	1    7550 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	7550 2600 7900 2600
Text GLabel 7900 2400 0    50   Input ~ 0
SCL
Text GLabel 7900 2500 0    50   Input ~ 0
SDA
Text GLabel 5850 2600 2    50   Input ~ 0
SCL
Text GLabel 5850 2500 2    50   Input ~ 0
SDA
$EndSCHEMATC
