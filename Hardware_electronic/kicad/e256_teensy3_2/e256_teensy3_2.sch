EESchema Schematic File Version 4
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
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
L power:+3.3V #PWR0101
U 1 1 5DEB6E43
P 5050 2100
F 0 "#PWR0101" H 5050 1950 50  0001 C CNN
F 1 "+3.3V" H 5065 2273 50  0000 C CNN
F 2 "" H 5050 2100 50  0001 C CNN
F 3 "" H 5050 2100 50  0001 C CNN
	1    5050 2100
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR0102
U 1 1 5DEB6EE2
P 1800 2650
F 0 "#PWR0102" H 1800 2400 50  0001 C CNN
F 1 "GNDREF" H 1805 2477 50  0000 C CNN
F 2 "" H 1800 2650 50  0001 C CNN
F 3 "" H 1800 2650 50  0001 C CNN
	1    1800 2650
	1    0    0    -1  
$EndComp
$Comp
L e256:e256_pad_16x16 U2
U 1 1 5DEF4BB3
P 9300 4250
F 0 "U2" H 10528 3401 50  0000 L CNN
F 1 "e256_pad_16x16" H 10528 3310 50  0000 L CNN
F 2 "e256:e256_pad_16X16" H 9350 4300 50  0001 C CNN
F 3 "" H 9350 4300 50  0001 C CNN
	1    9300 4250
	1    0    0    -1  
$EndComp
$Comp
L teensy:Teensy3.2 U1
U 1 1 5DEF4C87
P 3800 3850
F 0 "U1" H 3800 5593 60  0000 C CNN
F 1 "Teensy3.2" H 3800 5487 60  0000 C CNN
F 2 "teensy:Teensy30_31_32_LC" H 3800 5381 60  0000 C CNN
F 3 "" H 3800 3100 60  0000 C CNN
	1    3800 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 3600 5100 3600
Wire Wire Line
	1800 2500 1800 2650
Wire Wire Line
	4800 3700 5100 3700
Wire Wire Line
	4800 3200 5100 3200
Wire Wire Line
	4800 3300 5100 3300
Wire Wire Line
	4800 3400 5100 3400
Wire Wire Line
	4800 3500 5100 3500
Wire Wire Line
	2800 5200 2400 5200
Wire Wire Line
	2800 5100 2400 5100
Wire Wire Line
	2800 4800 2400 4800
Wire Wire Line
	2800 4700 2400 4700
Wire Wire Line
	2800 4600 2400 4600
Wire Wire Line
	2800 4500 2400 4500
Wire Wire Line
	4800 5100 5150 5100
Wire Wire Line
	4800 5200 5150 5200
Wire Wire Line
	4800 4400 5150 4400
Wire Wire Line
	4800 4500 5150 4500
Wire Wire Line
	9200 4500 9100 4500
Wire Wire Line
	9200 4400 9100 4400
Wire Wire Line
	9200 4600 9100 4600
Wire Wire Line
	9200 4700 9100 4700
Wire Wire Line
	9200 4800 9100 4800
Wire Wire Line
	9200 4900 9100 4900
Wire Wire Line
	9200 5000 9100 5000
Wire Wire Line
	9200 5100 9100 5100
Wire Wire Line
	9200 5200 9100 5200
Wire Wire Line
	9200 5300 9100 5300
Wire Wire Line
	9200 5400 9100 5400
Wire Wire Line
	9200 5500 9100 5500
Wire Wire Line
	9200 5600 9100 5600
Wire Wire Line
	9200 5700 9100 5700
Wire Wire Line
	9200 5800 9100 5800
Wire Wire Line
	9200 5900 9100 5900
Text GLabel 5100 3200 2    50   Input ~ 0
ROW_4
Text GLabel 5100 3300 2    50   Input ~ 0
ROW_2
Text GLabel 5100 3400 2    50   Input ~ 0
ROW_1
Text GLabel 5100 3500 2    50   Input ~ 0
ROW_0
Text GLabel 9100 4400 0    50   Input ~ 0
ROW_0
Text GLabel 9100 4500 0    50   Input ~ 0
ROW_1
Text GLabel 9100 4600 0    50   Input ~ 0
ROW_2
Text GLabel 9100 4700 0    50   Input ~ 0
ROW_3
Text GLabel 9100 4800 0    50   Input ~ 0
ROW_4
Text GLabel 9100 4900 0    50   Input ~ 0
ROW_5
Text GLabel 9100 5000 0    50   Input ~ 0
ROW_6
Text GLabel 9100 5100 0    50   Input ~ 0
ROW_7
Text GLabel 9100 5200 0    50   Input ~ 0
ROW_8
Text GLabel 9100 5300 0    50   Input ~ 0
ROW_9
Text GLabel 9100 5400 0    50   Input ~ 0
ROW_10
Text GLabel 9100 5500 0    50   Input ~ 0
ROW_11
Text GLabel 9100 5600 0    50   Input ~ 0
ROW_12
Text GLabel 9100 5700 0    50   Input ~ 0
ROW_13
Text GLabel 9100 5800 0    50   Input ~ 0
ROW_14
Text GLabel 9100 5900 0    50   Input ~ 0
ROW_15
Text GLabel 2400 4500 0    50   Input ~ 0
ROW_3
Text GLabel 2400 4600 0    50   Input ~ 0
ROW_5
Text GLabel 2400 4700 0    50   Input ~ 0
ROW_6
Text GLabel 2400 4800 0    50   Input ~ 0
ROW_7
Text GLabel 2400 4900 0    50   Input ~ 0
ROW_8
Text GLabel 2400 5000 0    50   Input ~ 0
ROW_9
Text GLabel 2400 5100 0    50   Input ~ 0
ROW_10
Text GLabel 2400 5200 0    50   Input ~ 0
ROW_11
Text GLabel 5150 4400 2    50   Input ~ 0
ROW_12
Text GLabel 5150 5200 2    50   Input ~ 0
ROW_13
Text GLabel 5150 4500 2    50   Input ~ 0
ROW_14
Text GLabel 5150 5100 2    50   Input ~ 0
ROW_15
Wire Wire Line
	10100 4400 10000 4400
Wire Wire Line
	10100 4500 10000 4500
Wire Wire Line
	10100 4600 10000 4600
Wire Wire Line
	10100 4700 10000 4700
Wire Wire Line
	10100 4800 10000 4800
Wire Wire Line
	10100 4900 10000 4900
Wire Wire Line
	10100 5000 10000 5000
Wire Wire Line
	10100 5100 10000 5100
Wire Wire Line
	10100 5200 10000 5200
Wire Wire Line
	10100 5300 10000 5300
Wire Wire Line
	10100 5400 10000 5400
Wire Wire Line
	10100 5500 10000 5500
Wire Wire Line
	10100 5600 10000 5600
Wire Wire Line
	10100 5700 10000 5700
Wire Wire Line
	10100 5800 10000 5800
Wire Wire Line
	10100 5900 10000 5900
Text GLabel 10000 4400 0    50   Input ~ 0
COL_0
Text GLabel 10000 4500 0    50   Input ~ 0
COL_1
Text GLabel 10000 4600 0    50   Input ~ 0
COL_2
Text GLabel 10000 4700 0    50   Input ~ 0
COL_3
Text GLabel 10000 4800 0    50   Input ~ 0
COL_4
Text GLabel 10000 4900 0    50   Input ~ 0
COL_5
Text GLabel 10000 5000 0    50   Input ~ 0
COL_6
Text GLabel 10000 5100 0    50   Input ~ 0
COL_7
Text GLabel 10000 5200 0    50   Input ~ 0
COL_8
Text GLabel 10000 5300 0    50   Input ~ 0
COL_9
Text GLabel 10000 5400 0    50   Input ~ 0
COL_10
Text GLabel 10000 5500 0    50   Input ~ 0
COL_11
Text GLabel 10000 5600 0    50   Input ~ 0
COL_12
Text GLabel 10000 5700 0    50   Input ~ 0
COL_13
Text GLabel 10000 5800 0    50   Input ~ 0
COL_14
Text GLabel 10000 5900 0    50   Input ~ 0
COL_15
Text GLabel 5100 3600 2    50   Input ~ 0
COL_0
Text GLabel 5100 3700 2    50   Input ~ 0
COL_1
Wire Wire Line
	4800 3800 5100 3800
Wire Wire Line
	4800 3900 5100 3900
Text GLabel 5100 3800 2    50   Input ~ 0
COL_2
Text GLabel 5100 3900 2    50   Input ~ 0
COL_3
Wire Wire Line
	2800 3800 2400 3800
Wire Wire Line
	2800 3500 2400 3500
Wire Wire Line
	2800 3400 2400 3400
Wire Wire Line
	2800 3300 2400 3300
Wire Wire Line
	2800 3200 2400 3200
Wire Wire Line
	2800 3100 2400 3100
Wire Wire Line
	4800 3000 5100 3000
Wire Wire Line
	1800 2500 2800 2500
Wire Wire Line
	2800 2600 2400 2600
Wire Wire Line
	2800 2700 2400 2700
Wire Wire Line
	2800 2800 2400 2800
Text GLabel 2400 2600 0    50   Input ~ 0
COL_15
Text GLabel 2400 2700 0    50   Input ~ 0
COL_14
Text GLabel 2400 2800 0    50   Input ~ 0
COL_13
Text GLabel 5100 3000 2    50   Input ~ 0
COL_12
Text GLabel 2400 3100 0    50   Input ~ 0
COL_11
Text GLabel 2400 3200 0    50   Input ~ 0
COL_10
Wire Wire Line
	2800 3600 2400 3600
Wire Wire Line
	2800 3700 2400 3700
Text GLabel 2400 3800 0    50   Input ~ 0
COL_4
Text GLabel 2400 3700 0    50   Input ~ 0
COL_5
Text GLabel 2400 3600 0    50   Input ~ 0
COL_6
Text GLabel 2400 3500 0    50   Input ~ 0
COL_7
Text GLabel 2400 3400 0    50   Input ~ 0
COL_8
Text GLabel 2400 3300 0    50   Input ~ 0
COL_9
$Comp
L Switch:SW_Push SW1
U 1 1 5DF03CDA
P 6700 2850
F 0 "SW1" H 6700 3135 50  0000 C CNN
F 1 "SW_Push" H 6700 3044 50  0000 C CNN
F 2 "" H 6700 3050 50  0001 C CNN
F 3 "" H 6700 3050 50  0001 C CNN
	1    6700 2850
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D1
U 1 1 5DF03D84
P 6700 2050
F 0 "D1" H 6692 1795 50  0000 C CNN
F 1 "LED" H 6692 1886 50  0000 C CNN
F 2 "" H 6700 2050 50  0001 C CNN
F 3 "~" H 6700 2050 50  0001 C CNN
	1    6700 2050
	-1   0    0    1   
$EndComp
$Comp
L power:GNDREF #PWR0103
U 1 1 5DF0661F
P 8450 4450
F 0 "#PWR0103" H 8450 4200 50  0001 C CNN
F 1 "GNDREF" H 8455 4277 50  0000 C CNN
F 2 "" H 8450 4450 50  0001 C CNN
F 3 "" H 8450 4450 50  0001 C CNN
	1    8450 4450
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR0104
U 1 1 5DF06666
P 8450 5400
F 0 "#PWR0104" H 8450 5150 50  0001 C CNN
F 1 "GNDREF" H 8455 5227 50  0000 C CNN
F 2 "" H 8450 5400 50  0001 C CNN
F 3 "" H 8450 5400 50  0001 C CNN
	1    8450 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	8450 5300 8450 5400
Wire Wire Line
	7900 4300 7800 4300
Wire Wire Line
	7900 4400 7800 4400
Wire Wire Line
	7900 4500 7800 4500
Wire Wire Line
	7900 4600 7800 4600
Wire Wire Line
	7900 4700 7800 4700
Wire Wire Line
	7900 4800 7800 4800
Wire Wire Line
	7900 4900 7800 4900
Wire Wire Line
	7900 5000 7800 5000
Wire Wire Line
	7900 5300 7800 5300
Wire Wire Line
	7900 5400 7800 5400
Wire Wire Line
	7900 5500 7800 5500
Wire Wire Line
	7900 5600 7800 5600
Wire Wire Line
	7900 5700 7800 5700
Wire Wire Line
	7900 5800 7800 5800
Wire Wire Line
	7900 5900 7800 5900
Wire Wire Line
	7900 6000 7800 6000
Text GLabel 7800 4300 0    50   Input ~ 0
COL_0
Text GLabel 7800 4400 0    50   Input ~ 0
COL_1
Text GLabel 7800 4500 0    50   Input ~ 0
COL_2
Text GLabel 7800 4600 0    50   Input ~ 0
COL_3
Text GLabel 7800 4700 0    50   Input ~ 0
COL_4
Text GLabel 7800 4800 0    50   Input ~ 0
COL_5
Text GLabel 7800 4900 0    50   Input ~ 0
COL_6
Text GLabel 7800 5000 0    50   Input ~ 0
COL_7
Text GLabel 7800 5300 0    50   Input ~ 0
COL_8
Text GLabel 7800 5400 0    50   Input ~ 0
COL_9
Text GLabel 7800 5500 0    50   Input ~ 0
COL_10
Text GLabel 7800 5600 0    50   Input ~ 0
COL_11
Text GLabel 7800 5700 0    50   Input ~ 0
COL_12
Text GLabel 7800 5800 0    50   Input ~ 0
COL_13
Text GLabel 7800 5900 0    50   Input ~ 0
COL_14
Text GLabel 7800 6000 0    50   Input ~ 0
COL_15
$Comp
L Device:R_Network08_US RN1
U 1 1 5DF1EF11
P 8100 4700
F 0 "RN1" V 7483 4700 50  0000 C CNN
F 1 "R_Network08_US" V 7574 4700 50  0000 C CNN
F 2 "Resistor_THT:R_Array_SIP9" V 8575 4700 50  0001 C CNN
F 3 "http://www.vishay.com/docs/31509/csc.pdf" H 8100 4700 50  0001 C CNN
	1    8100 4700
	0    1    1    0   
$EndComp
$Comp
L Device:R_Network08_US RN2
U 1 1 5DF1EFBB
P 8100 5700
F 0 "RN2" V 7483 5700 50  0000 C CNN
F 1 "R_Network08_US" V 7574 5700 50  0000 C CNN
F 2 "Resistor_THT:R_Array_SIP9" V 8575 5700 50  0001 C CNN
F 3 "http://www.vishay.com/docs/31509/csc.pdf" H 8100 5700 50  0001 C CNN
	1    8100 5700
	0    1    1    0   
$EndComp
Wire Wire Line
	8300 4300 8450 4300
Wire Wire Line
	8450 4300 8450 4450
Wire Wire Line
	8300 5300 8450 5300
$Comp
L Connector:Barrel_Jack_Switch J1
U 1 1 5DF3EDC1
P 9200 2500
F 0 "J1" H 8971 2450 50  0000 R CNN
F 1 "Barrel_Jack_Switch" H 8971 2541 50  0000 R CNN
F 2 "" H 9250 2460 50  0001 C CNN
F 3 "~" H 9250 2460 50  0001 C CNN
	1    9200 2500
	-1   0    0    1   
$EndComp
$Comp
L Device:R_POT RV1
U 1 1 5DF71EE5
P 7900 2850
F 0 "RV1" H 7830 2896 50  0000 R CNN
F 1 "R_POT" H 7830 2805 50  0000 R CNN
F 2 "" H 7900 2850 50  0001 C CNN
F 3 "~" H 7900 2850 50  0001 C CNN
	1    7900 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 4300 2800 4300
Text GLabel 1700 4300 0    50   Input ~ 0
DAC
$EndSCHEMATC
