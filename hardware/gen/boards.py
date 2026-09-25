"""The three boards of the ANC helmet, described as circuits.

main      - left earcup: STM32H743, power, USB-C, charger, left mic front-ends, DAC
            reconstruction, stereo headphone amp, BLE header, UI
satellite - right earcup: right mic front-ends (differential preamp + anti-alias),
            speaker connector, cable to the main board
mic       - 10 x 10 mm carrier for one IM73A135 differential MEMS mic (x4 per helmet)

Analog front end per mic (values from sim/plant.py: 3rd-order ~9-10 kHz anti-alias):
    mic OUT+/OUT- -> 1 uF -> 22k -> difference amp (Rf 75k ref / 150k err, Cf 220p / 100p pole)
    -> unity-gain Sallen-Key low-pass (8.2k, 8.2k, 4.7n, 1n; f0 ~9 kHz, Q ~1.1)
    -> 47R + 2.2n at the ADC pin
"""
from circuit import Circuit, Part, Builder, FP_C0402, FP_C0603, FP_C0805, FP_R0402
from pinmap import PINS
import kisym

GND = "GND"


def pkg(fp):
    return fp.split(":")[1].split("_")[1]


def mic_frontend(b: Builder, name, mic_p, mic_n, out_net, rf, cf, opamp_ref, opamp_mpn="MCP6022-I/ST"):
    """Difference amp + Sallen-Key on one MCP6022 (units A, B). Returns the op-amp part."""
    n = name
    b.C(mic_p, f"{n}_AP", "1u", FP_C0603, "X7R")
    b.C(mic_n, f"{n}_AN", "1u", FP_C0603, "X7R")
    b.R(f"{n}_AP", f"{n}_P", "22k")
    b.R(f"{n}_AN", f"{n}_N", "22k")
    b.R(f"{n}_P", "VMID", rf)
    b.C(f"{n}_P", "VMID", cf, FP_C0402, "C0G")
    b.R(f"{n}_N", f"{n}_S1", rf)
    b.C(f"{n}_N", f"{n}_S1", cf, FP_C0402, "C0G")
    # Sallen-Key unity gain low-pass
    b.R(f"{n}_S1", f"{n}_SKA", "8.2k")
    b.R(f"{n}_SKA", f"{n}_SKB", "8.2k")
    b.C(f"{n}_SKA", f"{n}_OUT", "4.7n", FP_C0603, "C0G")
    b.C(f"{n}_SKB", GND, "1n", FP_C0402, "C0G")
    u = b.part("U", "Amplifier_Operational", "MCP6022", "MCP6022", "Package_SO:TSSOP-8_4.4x3mm_P0.65mm",
               {"1": f"{n}_S1", "2": f"{n}_N", "3": f"{n}_P",
                "5": f"{n}_SKB", "6": f"{n}_OUT", "7": f"{n}_OUT",
                "4": GND, "8": "2V8A"}, opamp_mpn)
    b.C("2V8A", GND, "100n")
    return u


def mcu_pins():
    """LQFP100 pin number -> net, from the pin map + power/system pins; the rest NC."""
    sym = kisym.resolve("MCU_ST_STM32H7", "STM32H743VITx")
    allpins = {p[0]: p[1] for p in kisym.pins(sym)}
    m = {str(v[1]): net for net, v in PINS.items()}
    for num, name in allpins.items():
        if num in m:
            continue
        if name == "VDD":
            m[num] = "3V3"
        elif name in ("VSS", "VSSA"):
            m[num] = GND
        elif name == "VBAT":
            m[num] = "3V3"
        elif name == "VCAP":
            m[num] = "VCAP"
        elif name in ("VDDA", "VREF+"):
            m[num] = "2V8A"
        elif name == "NRST":
            m[num] = "NRST"
        elif name == "BOOT0":
            m[num] = "BOOT0"
        else:
            m[num] = None
    return m


# =====================================================================================
def main_board():
    c = Circuit("main-board", "ANC Helmet - main board (left earcup)")
    b = Builder(c)

    # ---------------------------------------------------------------- USB-C + protection
    b.block = "USB-C input"
    b.part("J", "Connector", "USB_C_Receptacle_USB2.0_16P", "USB-C", "Connector_USB:USB_C_Receptacle_HRO_TYPE-C-31-M-12",
           {"A1": GND, "A12": GND, "B1": GND, "B12": GND, "A4": "VBUS", "A9": "VBUS", "B4": "VBUS", "B9": "VBUS",
            "A5": "CC1", "B5": "CC2", "A6": "USB_DP", "B6": "USB_DP", "A7": "USB_DM", "B7": "USB_DM",
            "A8": None, "B8": None, "S1": "SHIELD"}, "HRO TYPE-C-31-M-12")
    b.R("CC1", GND, "5.1k")
    b.R("CC2", GND, "5.1k")
    b.R("SHIELD", GND, "1M")
    b.C("SHIELD", GND, "4.7n", FP_C0402, "X7R 50V")
    b.part("U", "Power_Protection", "USBLC6-2SC6", "USBLC6-2SC6", "Package_TO_SOT_SMD:SOT-23-6",
           {"1": "USB_DP", "6": "USB_DP", "3": "USB_DM", "4": "USB_DM", "2": GND, "5": "VBUS"}, "USBLC6-2SC6")
    b.part("F", "Device", "Polyfuse", "0.5A", "Fuse:Fuse_1206_3216Metric", {"1": "VBUS", "2": "VBUS_F"},
           "Littelfuse 1206L050YR")
    b.R("VBUS_F", "VBUS_SNS", "100k")
    b.R("VBUS_SNS", GND, "200k")

    # ---------------------------------------------------------------- charger + power path
    b.block = "Li-ion charger + power path"
    b.part("U", "Battery_Management", "MCP73831-2-OT", "MCP73831-2", "Package_TO_SOT_SMD:SOT-23-5",
           {"1": "CHG_STAT_RAW", "2": GND, "3": "VBAT", "4": "VBUS_F", "5": "CHG_PROG"}, "MCP73831T-2ACI/OT")
    b.C("VBUS_F", GND, "4.7u", FP_C0603, "X5R 10V")
    b.C("VBAT", GND, "4.7u", FP_C0603, "X5R 10V")
    b.R("CHG_PROG", GND, "2.0k", note="500 mA charge current")
    b.R("VBUS_F", "CHG_LED_A", "1k")
    b.part("D", "Device", "LED", "orange", "LED_SMD:LED_0603_1608Metric", {"2": "CHG_LED_A", "1": "CHG_STAT_RAW"},
           "0603 orange LED")
    b.R("CHG_STAT_RAW", "CHG_STAT", "47k")
    b.R("CHG_STAT", GND, "100k")
    b.part("J", "Connector_Generic", "Conn_01x02", "BATTERY 1S Li-ion (protected)",
           "Connector_JST:JST_PH_S2B-PH-SM4-TB_1x02-1MP_P2.00mm_Horizontal", {"1": "VBAT", "2": GND},
           "JST S2B-PH-SM4-TB", note="pin 1 = BAT+ (check your pack's wiring!)")
    b.part("Q", "Transistor_FET", "AO3401A", "AO3401A", "Package_TO_SOT_SMD:SOT-23",
           {"1": "VBUS_F", "2": "VSYS", "3": "VBAT"}, "AO3401A", note="battery -> system when USB absent")
    b.R("VBUS_F", GND, "100k")
    b.part("D", "Device", "D_Schottky", "B5819W", "Diode_SMD:D_SOD-123", {"1": "VSYS", "2": "VBUS_F"}, "B5819W")
    b.R("VBAT", "VBAT_SNS", "100k")
    b.R("VBAT_SNS", GND, "100k")
    b.C("VBAT_SNS", GND, "100n")

    # ---------------------------------------------------------------- soft power + button
    b.block = "Soft power + button"
    b.part("SW", "Switch", "SW_Push", "POWER", "Button_Switch_SMD:SW_SPST_PTS810", {"1": "VSYS", "2": "BTN_HI"},
           "C&K PTS810 SJM 250 SMTR LFS")
    b.part("J", "Connector_Generic", "Conn_01x02", "EXT BUTTON", "Connector_JST:JST_SH_SM02B-SRSS-TB_1x02-1MP_P1.00mm_Horizontal",
           {"1": "VSYS", "2": "BTN_HI"}, "JST SM02B-SRSS-TB", note="optional panel button on the cup")
    b.part("D", "Diode", "BAT54C", "BAT54C", "Package_TO_SOT_SMD:SOT-23", {"1": "BTN_HI", "2": "PWR_HOLD", "3": "EN_REG"},
           "BAT54C")
    b.R("EN_REG", GND, "100k")
    b.R("BTN_HI", "BTN_B", "47k")
    b.R("BTN_B", GND, "100k")
    b.part("Q", "Transistor_BJT", "MMBT3904", "MMBT3904", "Package_TO_SOT_SMD:SOT-23", {"1": "BTN_B", "2": GND, "3": "BTN"},
           "MMBT3904")
    b.R("BTN", "3V3", "10k")

    # ---------------------------------------------------------------- regulators
    b.block = "3.3 V buck-boost + 2.8 V analog LDO"
    b.part("U", "Regulator_Switching", "TPS63001", "TPS63001", "Package_SON:Texas_DRC0010J_ThermalVias",
           {"1": "3V3", "10": "3V3", "2": "SW_L2", "4": "SW_L1", "3": GND, "11": GND, "9": GND,
            "5": "VSYS", "8": "VSYS", "6": "EN_REG", "7": "VSYS"}, "TPS63001DRCR",
           note="PS/SYNC high = forced 1.5 MHz PWM (no PFM bursts in the audio band)")
    b.part("L", "Device", "L", "2.2u", "Inductor_SMD:L_Changjiang_FNR3015S", {"1": "SW_L1", "2": "SW_L2"},
           "2.2 uH >=1.5 A shielded 3x3 (e.g. TDK VLS3015ET-2R2M)")
    b.C("VSYS", GND, "10u", FP_C0805, "X5R 10V")
    b.C("3V3", GND, "22u", FP_C0805, "X5R 10V")
    b.C("3V3", GND, "22u", FP_C0805, "X5R 10V")
    b.FB("3V3", "3V3_LDO")
    b.part("U", "Regulator_Linear", "LP5907MFX-2.8", "LP5907-2.8", "Package_TO_SOT_SMD:SOT-23-5",
           {"1": "3V3_LDO", "2": GND, "3": "3V3_LDO", "4": None, "5": "2V8A"}, "LP5907MFX-2.8/NOPB")
    b.C("3V3_LDO", GND, "1u", FP_C0603)
    b.C("2V8A", GND, "4.7u", FP_C0603, "X5R")
    b.C("2V8A", GND, "100n")

    # ---------------------------------------------------------------- MCU
    b.block = "MCU STM32H743"
    b.part("U", "MCU_ST_STM32H7", "STM32H743VITx", "STM32H743VIT6", "Package_QFP:LQFP-100_14x14mm_P0.5mm",
           mcu_pins(), "STM32H743VIT6")
    for _ in range(5):
        b.C("3V3", GND, "100n")
    b.C("3V3", GND, "4.7u", FP_C0603)
    b.C("3V3", GND, "100n", note="VBAT pin")
    b.C("VCAP", GND, "2.2u", FP_C0603)
    b.C("VCAP", GND, "2.2u", FP_C0603)
    b.C("2V8A", GND, "1u", FP_C0603, note="VDDA")
    b.C("2V8A", GND, "100n", note="VDDA")
    b.C("2V8A", GND, "1u", FP_C0603, note="VREF+")
    b.C("2V8A", GND, "100n", note="VREF+")
    b.C("NRST", GND, "100n")
    b.R("BOOT0", GND, "10k")
    b.part("SW", "Switch", "SW_Push", "BOOT", "Button_Switch_SMD:SW_SPST_PTS810", {"1": "3V3", "2": "BOOT0"},
           "C&K PTS810 SJM 250 SMTR LFS", note="hold while powering on -> USB DFU")
    b.part("Y", "Device", "Crystal_GND24", "25MHz", "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm",
           {"1": "HSE_IN", "3": "HSE_OUT", "2": GND, "4": GND}, "25 MHz 3225 CL=10pF +-20ppm (e.g. Abracon ABM8)")
    b.C("HSE_IN", GND, "12p", FP_C0402, "C0G")
    b.C("HSE_OUT", GND, "12p", FP_C0402, "C0G")
    b.part("J", "Connector", "Conn_ARM_JTAG_SWD_10", "SWD", "Connector_PinHeader_1.27mm:PinHeader_2x05_P1.27mm_Vertical_SMD",
           {"1": "3V3", "2": "SWDIO", "3": GND, "4": "SWCLK", "5": GND, "6": "SWO", "7": None, "8": None, "9": GND,
            "10": "NRST"}, "Cortex debug 2x5 1.27 mm SMD")

    # ---------------------------------------------------------------- UI
    b.block = "Status LEDs + test points"
    for col, net, r in (("red", "LED_R", "1k"), ("green", "LED_G", "1k"), ("blue", "LED_B", "470")):
        b.part("D", "Device", "LED", col, "LED_SMD:LED_0603_1608Metric", {"2": "3V3", "1": f"{net}_K"}, f"0603 {col} LED")
        b.R(f"{net}_K", net, r)
    for net in ("3V3", "2V8A", "VSYS", "VBAT", "VMID", GND, "DBG_TP", "REF_L", "ERR_L", "DACL_OUT", "I2C1_SCL", "I2C1_SDA"):
        b.TP(net)

    # ---------------------------------------------------------------- analog: bias + left mics
    b.block = "Analog bias (VMID) + left mic front-ends"
    b.R("2V8A", "VMID_DIV", "10k")
    b.R("VMID_DIV", GND, "10k")
    b.C("VMID_DIV", GND, "10u", FP_C0805, "X5R")
    b.part("U", "Amplifier_Operational", "MCP6022", "MCP6022", "Package_SO:TSSOP-8_4.4x3mm_P0.65mm",
           {"1": "VMID_BUF", "2": "VMID_BUF", "3": "VMID_DIV", "7": "U_SPARE_OUT", "6": "U_SPARE_OUT", "5": "VMID",
            "4": GND, "8": "2V8A"}, "MCP6022-I/ST", note="unit B unused (follower)")
    b.C("2V8A", GND, "100n")
    b.R("VMID_BUF", "VMID", "10")
    b.C("VMID", GND, "1u", FP_C0603)
    for net, label in (("MICL_REF", "LEFT OUTSIDE (REF) MIC"), ("MICL_ERR", "LEFT IN-CUP (ERR) MIC")):
        b.part("J", "Connector_Generic", "Conn_01x04", label, "Connector_JST:JST_SH_SM04B-SRSS-TB_1x04-1MP_P1.00mm_Horizontal",
               {"1": "2V8A", "2": GND, "3": f"{net}_P", "4": f"{net}_N"}, "JST SM04B-SRSS-TB")
    mic_frontend(b, "REFL", "MICL_REF_P", "MICL_REF_N", "REFL_OUT", "75k", "220p", "U")
    b.R("REFL_OUT", "REF_L", "47")
    b.C("REF_L", GND, "2.2n", FP_C0402, "C0G")
    mic_frontend(b, "ERRL", "MICL_ERR_P", "MICL_ERR_N", "ERRL_OUT", "150k", "100p", "U")
    b.R("ERRL_OUT", "ERR_L", "47")
    b.C("ERR_L", GND, "2.2n", FP_C0402, "C0G")

    # ---------------------------------------------------------------- right channel via satellite cable
    b.block = "Right earcup cable + ADC inputs"
    b.part("J", "Connector_Generic", "Conn_01x06", "TO RIGHT CUP", "Connector_JST:JST_SH_SM06B-SRSS-TB_1x06-1MP_P1.00mm_Horizontal",
           {"1": "2V8A", "2": GND, "3": "REF_R_CBL", "4": "ERR_R_CBL", "5": "SPK_R", "6": GND}, "JST SM06B-SRSS-TB")
    b.R("REF_R_CBL", "REF_R", "47")
    b.C("REF_R", GND, "2.2n", FP_C0402, "C0G")
    b.R("ERR_R_CBL", "ERR_R", "47")
    b.C("ERR_R", GND, "2.2n", FP_C0402, "C0G")

    # ---------------------------------------------------------------- DAC reconstruction + headphone amp
    b.block = "DAC reconstruction + headphone amplifier"
    for side, dac in (("L", "DAC_L"), ("R", "DAC_R")):
        n = f"DAC{side}"
        b.R(dac, f"{n}_A", "8.2k")
        b.R(f"{n}_A", f"{n}_B", "8.2k")
        b.C(f"{n}_A", f"{n}_OUT", "3.3n", FP_C0603, "C0G")
        b.C(f"{n}_B", GND, "1.5n", FP_C0402, "C0G")
        b.C(f"{n}_OUT", f"AMP_IN{side}P", "1u", FP_C0603)
        b.C(f"AMP_IN{side}N", GND, "1u", FP_C0603)
    b.part("U", "Amplifier_Operational", "MCP6022", "MCP6022", "Package_SO:TSSOP-8_4.4x3mm_P0.65mm",
           {"1": "DACL_OUT", "2": "DACL_OUT", "3": "DACL_B", "7": "DACR_OUT", "6": "DACR_OUT", "5": "DACR_B",
            "4": GND, "8": "2V8A"}, "MCP6022-I/ST")
    b.C("2V8A", GND, "100n")
    b.FB("3V3", "3V3_AMP")
    b.part("U", "Amplifier_Audio", "TPA6132A2RTE", "TPA6132A2", "Package_DFN_QFN:WQFN-16-1EP_3x3mm_P0.5mm_EP1.6x1.6mm_ThermalVias",
           {"1": "AMP_INLN", "2": "AMP_INLP", "3": "AMP_INRP", "4": "AMP_INRN", "5": "SPK_R", "16": "SPK_L",
            "6": "AMP_G0", "7": "AMP_G1", "13": "AMP_EN", "14": "3V3_AMP", "12": "HPVDD", "8": "HPVSS",
            "11": "CPP", "9": "CPN", "10": GND, "15": GND, "17": GND}, "TPA6132A2RTER")
    b.C("3V3_AMP", GND, "2.2u", FP_C0603, "X5R")
    b.C("3V3_AMP", GND, "100n")
    b.C("HPVDD", GND, "2.2u", FP_C0603, "X5R")
    b.C("HPVSS", GND, "2.2u", FP_C0603, "X5R")
    b.C("CPP", "CPN", "1u", FP_C0603, "X5R")
    b.part("J", "Connector_Generic", "Conn_01x02", "LEFT SPEAKER", "Connector_JST:JST_SH_SM02B-SRSS-TB_1x02-1MP_P1.00mm_Horizontal",
           {"1": "SPK_L", "2": GND}, "JST SM02B-SRSS-TB")

    # ---------------------------------------------------------------- BLE
    b.block = "BLE module header (optional)"
    b.part("J", "Connector_Generic", "Conn_01x06", "BLE UART MODULE", "Connector_JST:JST_SH_SM06B-SRSS-TB_1x06-1MP_P1.00mm_Horizontal",
           {"1": "3V3", "2": GND, "3": "BLE_TX", "4": "BLE_RX", "5": "BLE_EN", "6": "BLE_STATE"}, "JST SM06B-SRSS-TB")
    b.C("3V3", GND, "10u", FP_C0805, "X5R", note="BLE module bulk")

    # ---------------------------------------------------------------- mechanics
    b.block = "Mounting"
    for _ in range(3):
        b.part("H", "Mechanical", "MountingHole_Pad", "M2", "MountingHole:MountingHole_2.2mm_M2_Pad_Via", {"1": GND}, "")

    c.power_nets = {"VBUS", "VBUS_F", "VSYS", "VBAT", "3V3", "3V3_LDO", "3V3_AMP", "2V8A", "VCAP", GND, "HPVDD", "HPVSS"}
    return c


# =====================================================================================
def satellite_board():
    c = Circuit("satellite-board", "ANC Helmet - satellite board (right earcup)")
    b = Builder(c)
    b.block = "Cable to main board"
    b.part("J", "Connector_Generic", "Conn_01x06", "TO MAIN BOARD", "Connector_JST:JST_SH_SM06B-SRSS-TB_1x06-1MP_P1.00mm_Horizontal",
           {"1": "2V8A", "2": GND, "3": "REF_R_CBL", "4": "ERR_R_CBL", "5": "SPK_R", "6": GND}, "JST SM06B-SRSS-TB")
    b.C("2V8A", GND, "10u", FP_C0805, "X5R")
    b.part("J", "Connector_Generic", "Conn_01x02", "RIGHT SPEAKER", "Connector_JST:JST_SH_SM02B-SRSS-TB_1x02-1MP_P1.00mm_Horizontal",
           {"1": "SPK_R", "2": GND}, "JST SM02B-SRSS-TB")
    b.block = "Local analog bias"
    b.R("2V8A", "VMID_DIV", "10k")
    b.R("VMID_DIV", GND, "10k")
    b.C("VMID_DIV", GND, "10u", FP_C0805, "X5R")
    b.part("U", "Amplifier_Operational", "MCP6001-OT", "MCP6001", "Package_TO_SOT_SMD:SOT-23-5",
           {"1": "VMID_BUF", "4": "VMID_BUF", "3": "VMID_DIV", "2": GND, "5": "2V8A"}, "MCP6001T-I/OT")
    b.C("2V8A", GND, "100n")
    b.R("VMID_BUF", "VMID", "10")
    b.C("VMID", GND, "1u", FP_C0603)
    b.block = "Right mic front-ends"
    for net, label in (("MICR_REF", "RIGHT OUTSIDE (REF) MIC"), ("MICR_ERR", "RIGHT IN-CUP (ERR) MIC")):
        b.part("J", "Connector_Generic", "Conn_01x04", label, "Connector_JST:JST_SH_SM04B-SRSS-TB_1x04-1MP_P1.00mm_Horizontal",
               {"1": "2V8A", "2": GND, "3": f"{net}_P", "4": f"{net}_N"}, "JST SM04B-SRSS-TB")
    mic_frontend(b, "REFR", "MICR_REF_P", "MICR_REF_N", "REFR_OUT", "75k", "220p", "U")
    b.R("REFR_OUT", "REF_R_CBL", "100", note="isolates the cable capacitance")
    mic_frontend(b, "ERRR", "MICR_ERR_P", "MICR_ERR_N", "ERRR_OUT", "150k", "100p", "U")
    b.R("ERRR_OUT", "ERR_R_CBL", "100", note="isolates the cable capacitance")
    b.block = "Mounting"
    for _ in range(2):
        b.part("H", "Mechanical", "MountingHole_Pad", "M2", "MountingHole:MountingHole_2.2mm_M2_Pad_Via", {"1": GND}, "")
    c.power_nets = {"2V8A", GND}
    return c


# =====================================================================================
def mic_board():
    c = Circuit("mic-board", "ANC Helmet - MEMS mic carrier (x4)")
    b = Builder(c)
    b.block = "Differential MEMS microphone"
    b.part("MK", "Sensor_Audio", "IM73A135V01", "IM73A135V01", "Sensor_Audio:Infineon_PG-LLGA-5-2",
           {"1": "OUT_P", "2": "VDD", "3": "OUT_N", "4": GND, "5": GND}, "IM73A135V01XTSA1",
           note="bottom port: sound enters through the PCB hole - mount the plain back side toward the sound")
    b.C("VDD", GND, "100n")
    b.C("VDD", GND, "1u", FP_C0402, "X5R")
    b.part("J", "Connector_Generic", "Conn_01x04", "MIC", "Connector_JST:JST_SH_SM04B-SRSS-TB_1x04-1MP_P1.00mm_Horizontal",
           {"1": "VDD", "2": GND, "3": "OUT_P", "4": "OUT_N"}, "JST SM04B-SRSS-TB")
    c.power_nets = {"VDD", GND}
    return c


BOARDS = {"main-board": main_board, "satellite-board": satellite_board, "mic-board": mic_board}

if __name__ == "__main__":
    for name, fn in BOARDS.items():
        c = fn()
        nets = c.nets()
        print(f"{name}: {len(c.parts)} parts, {len(nets)} nets, blocks={len(c.blocks)}")
        for p in c.check():
            print("   ", p)
