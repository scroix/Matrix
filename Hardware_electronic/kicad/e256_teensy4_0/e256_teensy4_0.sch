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
P 4650 2000
F 0 "#PWR0101" H 4650 1850 50  0001 C CNN
F 1 "+3.3V" H 4665 2173 50  0000 C CNN
F 2 "" H 4650 2000 50  0001 C CNN
F 3 "" H 4650 2000 50  0001 C CNN
	1    4650 2000
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR0102
U 1 1 5DEB6EE2
P 800 2550
F 0 "#PWR0102" H 800 2300 50  0001 C CNN
F 1 "GNDREF" H 805 2377 50  0000 C CNN
F 2 "" H 800 2550 50  0001 C CNN
F 3 "" H 800 2550 50  0001 C CNN
	1    800  2550
	1    0    0    -1  
$EndComp
$Comp
L e256:e256_pad_16x16 U2
U 1 1 5DEF4BB3
P 9300 4650
F 0 "U2" H 10528 3801 50  0000 L CNN
F 1 "e256_pad_16x16" H 10528 3710 50  0000 L CNN
F 2 "e256:e256_pad_16X16" H 9350 4700 50  0001 C CNN
F 3 "" H 9350 4700 50  0001 C CNN
	1    9300 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 3500 4700 3500
Wire Wire Line
	800  2400 800  2550
Wire Wire Line
	4400 3600 4700 3600
Wire Wire Line
	4400 3100 4700 3100
Wire Wire Line
	4400 3200 4700 3200
Wire Wire Line
	4400 3300 4700 3300
Wire Wire Line
	4400 3400 4700 3400
Wire Wire Line
	1800 5100 1400 5100
Wire Wire Line
	1800 5000 1400 5000
Wire Wire Line
	1800 4700 1400 4700
Wire Wire Line
	1800 4600 1400 4600
Wire Wire Line
	1800 4500 1400 4500
Wire Wire Line
	1800 4400 1400 4400
Wire Wire Line
	4400 5000 4750 5000
Wire Wire Line
	4400 5100 4750 5100
Wire Wire Line
	4400 4300 4750 4300
Wire Wire Line
	4400 4400 4750 4400
Wire Wire Line
	9200 4900 9100 4900
Wire Wire Line
	9200 4800 9100 4800
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
Wire Wire Line
	9200 6000 9100 6000
Wire Wire Line
	9200 6100 9100 6100
Wire Wire Line
	9200 6200 9100 6200
Wire Wire Line
	9200 6300 9100 6300
Text GLabel 4700 3100 2    50   Input ~ 0
ROW_4
Text GLabel 4700 3200 2    50   Input ~ 0
ROW_2
Text GLabel 4700 3300 2    50   Input ~ 0
ROW_1
Text GLabel 4700 3400 2    50   Input ~ 0
ROW_0
Text GLabel 9100 4800 0    50   Input ~ 0
ROW_0
Text GLabel 9100 4900 0    50   Input ~ 0
ROW_1
Text GLabel 9100 5000 0    50   Input ~ 0
ROW_2
Text GLabel 9100 5100 0    50   Input ~ 0
ROW_3
Text GLabel 9100 5200 0    50   Input ~ 0
ROW_4
Text GLabel 9100 5300 0    50   Input ~ 0
ROW_5
Text GLabel 9100 5400 0    50   Input ~ 0
ROW_6
Text GLabel 9100 5500 0    50   Input ~ 0
ROW_7
Text GLabel 9100 5600 0    50   Input ~ 0
ROW_8
Text GLabel 9100 5700 0    50   Input ~ 0
ROW_9
Text GLabel 9100 5800 0    50   Input ~ 0
ROW_10
Text GLabel 9100 5900 0    50   Input ~ 0
ROW_11
Text GLabel 9100 6000 0    50   Input ~ 0
ROW_12
Text GLabel 9100 6100 0    50   Input ~ 0
ROW_13
Text GLabel 9100 6200 0    50   Input ~ 0
ROW_14
Text GLabel 9100 6300 0    50   Input ~ 0
ROW_15
Text GLabel 1400 4400 0    50   Input ~ 0
ROW_3
Text GLabel 1400 4500 0    50   Input ~ 0
ROW_5
Text GLabel 1400 4600 0    50   Input ~ 0
ROW_6
Text GLabel 1400 4700 0    50   Input ~ 0
ROW_7
Text GLabel 1400 4800 0    50   Input ~ 0
ROW_8
Text GLabel 1400 4900 0    50   Input ~ 0
ROW_9
Text GLabel 1400 5000 0    50   Input ~ 0
ROW_10
Text GLabel 1400 5100 0    50   Input ~ 0
ROW_11
Text GLabel 4750 4300 2    50   Input ~ 0
ROW_12
Text GLabel 4750 5100 2    50   Input ~ 0
ROW_13
Text GLabel 4750 4400 2    50   Input ~ 0
ROW_14
Text GLabel 4750 5000 2    50   Input ~ 0
ROW_15
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
Wire Wire Line
	10100 6000 10000 6000
Wire Wire Line
	10100 6100 10000 6100
Wire Wire Line
	10100 6200 10000 6200
Wire Wire Line
	10100 6300 10000 6300
Text GLabel 10000 4800 0    50   Input ~ 0
COL_0
Text GLabel 10000 4900 0    50   Input ~ 0
COL_1
Text GLabel 10000 5000 0    50   Input ~ 0
COL_2
Text GLabel 10000 5100 0    50   Input ~ 0
COL_3
Text GLabel 10000 5200 0    50   Input ~ 0
COL_4
Text GLabel 10000 5300 0    50   Input ~ 0
COL_5
Text GLabel 10000 5400 0    50   Input ~ 0
COL_6
Text GLabel 10000 5500 0    50   Input ~ 0
COL_7
Text GLabel 10000 5600 0    50   Input ~ 0
COL_8
Text GLabel 10000 5700 0    50   Input ~ 0
COL_9
Text GLabel 10000 5800 0    50   Input ~ 0
COL_10
Text GLabel 10000 5900 0    50   Input ~ 0
COL_11
Text GLabel 10000 6000 0    50   Input ~ 0
COL_12
Text GLabel 10000 6100 0    50   Input ~ 0
COL_13
Text GLabel 10000 6200 0    50   Input ~ 0
COL_14
Text GLabel 10000 6300 0    50   Input ~ 0
COL_15
Text GLabel 4700 3500 2    50   Input ~ 0
COL_0
Text GLabel 4700 3600 2    50   Input ~ 0
COL_1
Wire Wire Line
	4400 3700 4700 3700
Wire Wire Line
	4400 3800 4700 3800
Text GLabel 4700 3700 2    50   Input ~ 0
COL_2
Text GLabel 4700 3800 2    50   Input ~ 0
COL_3
Wire Wire Line
	1800 3700 1400 3700
Wire Wire Line
	1800 3400 1400 3400
Wire Wire Line
	1800 3300 1400 3300
Wire Wire Line
	1800 3200 1400 3200
Wire Wire Line
	1800 3100 1400 3100
Wire Wire Line
	1800 3000 1400 3000
Wire Wire Line
	4400 2900 4700 2900
Wire Wire Line
	800  2400 1800 2400
Wire Wire Line
	1800 2500 1400 2500
Wire Wire Line
	1800 2600 1400 2600
Wire Wire Line
	1800 2700 1400 2700
Text GLabel 1400 2500 0    50   Input ~ 0
COL_15
Text GLabel 1400 2600 0    50   Input ~ 0
COL_14
Text GLabel 1400 2700 0    50   Input ~ 0
COL_13
Text GLabel 4700 2900 2    50   Input ~ 0
COL_12
Text GLabel 1400 3000 0    50   Input ~ 0
COL_11
Text GLabel 1400 3100 0    50   Input ~ 0
COL_10
Wire Wire Line
	1800 3500 1400 3500
Wire Wire Line
	1800 3600 1400 3600
Text GLabel 1400 3700 0    50   Input ~ 0
COL_4
Text GLabel 1400 3600 0    50   Input ~ 0
COL_5
Text GLabel 1400 3500 0    50   Input ~ 0
COL_6
Text GLabel 1400 3400 0    50   Input ~ 0
COL_7
Text GLabel 1400 3300 0    50   Input ~ 0
COL_8
Text GLabel 1400 3200 0    50   Input ~ 0
COL_9
$Comp
L Switch:SW_Push SW1
U 1 1 5DF03CDA
P 1300 7150
F 0 "SW1" H 1300 7435 50  0000 C CNN
F 1 "SW_Push" H 1300 7344 50  0000 C CNN
F 2 "" H 1300 7350 50  0001 C CNN
F 3 "" H 1300 7350 50  0001 C CNN
	1    1300 7150
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D1
U 1 1 5DF03D84
P 1300 6350
F 0 "D1" H 1292 6095 50  0000 C CNN
F 1 "LED" H 1292 6186 50  0000 C CNN
F 2 "" H 1300 6350 50  0001 C CNN
F 3 "~" H 1300 6350 50  0001 C CNN
	1    1300 6350
	-1   0    0    1   
$EndComp
$Comp
L Connector:Barrel_Jack_Switch J1
U 1 1 5DF3EDC1
P 4250 7050
F 0 "J1" H 4021 7000 50  0000 R CNN
F 1 "Barrel_Jack_Switch" H 4021 7091 50  0000 R CNN
F 2 "" H 4300 7010 50  0001 C CNN
F 3 "~" H 4300 7010 50  0001 C CNN
	1    4250 7050
	-1   0    0    1   
$EndComp
$Comp
L Device:R_POT RV1
U 1 1 5DF71EE5
P 2500 7150
F 0 "RV1" H 2430 7196 50  0000 R CNN
F 1 "R_POT" H 2430 7105 50  0000 R CNN
F 2 "" H 2500 7150 50  0001 C CNN
F 3 "~" H 2500 7150 50  0001 C CNN
	1    2500 7150
	1    0    0    -1  
$EndComp
Wire Wire Line
	700  4200 1800 4200
Text GLabel 700  4200 0    50   Input ~ 0
DAC
$Comp
L teensy:Teensy4.0 U1
U 1 1 5DF7B870
P 3150 3750
F 0 "U1" H 3150 5365 50  0000 C CNN
F 1 "Teensy4.0" H 3150 5274 50  0000 C CNN
F 2 "teensy:Teensy40" H 2750 3950 50  0001 C CNN
F 3 "" H 2750 3950 50  0001 C CNN
	1    3150 3750
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC595 U4
U 1 1 5DF835B5
P 8150 1550
F 0 "U4" H 8150 2328 50  0000 C CNN
F 1 "74HC595" H 8150 2237 50  0000 C CNN
F 2 "" H 8150 1550 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn74hc595.pdf" H 8150 1550 50  0001 C CNN
	1    8150 1550
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC4051 U6
U 1 1 5DF83935
P 9650 1500
F 0 "U6" H 9700 2178 50  0000 C CNN
F 1 "74HC4051" H 9700 2087 50  0000 C CNN
F 2 "" H 9650 1100 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/cd74hc4051.pdf" H 9650 1100 50  0001 C CNN
	1    9650 1500
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC4051 U7
U 1 1 5DF8E7D9
P 9650 2900
F 0 "U7" H 9700 3578 50  0000 C CNN
F 1 "74HC4051" H 9700 3487 50  0000 C CNN
F 2 "" H 9650 2500 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/cd74hc4051.pdf" H 9650 2500 50  0001 C CNN
	1    9650 2900
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC595 U5
U 1 1 5DF8E845
P 8150 3250
F 0 "U5" H 8150 4028 50  0000 C CNN
F 1 "74HC595" H 8150 3937 50  0000 C CNN
F 2 "" H 8150 3250 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn74hc595.pdf" H 8150 3250 50  0001 C CNN
	1    8150 3250
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC595 U3
U 1 1 5DF8E8FB
P 6650 2150
F 0 "U3" H 6650 2928 50  0000 C CNN
F 1 "74HC595" H 6650 2837 50  0000 C CNN
F 2 "" H 6650 2150 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn74hc595.pdf" H 6650 2150 50  0001 C CNN
	1    6650 2150
	1    0    0    -1  
$EndComp
$EndSCHEMATC
