# -*-coding:utf-8-*-

## this code is currently for micropython 3.4
from utime import sleep #, sleep_ms
from machine import Pin, I2C #, Signal

# register address-es
REG_INTR_STATUS_1 = 0x00	# Which interrupts are tripped
REG_INTR_STATUS_2 = 0x01
REG_INTR_ENABLE_1 = 0x02	# Which interrupts are active
REG_INTR_ENABLE_2 = 0x03

REG_FIFO_WR_PTR = 0x04		# Where data is being written
REG_OVF_COUNTER = 0x05		# Number of lost samples
REG_FIFO_RD_PTR = 0x06		# Where to read from
REG_FIFO_DATA = 0x07		# Ouput data buffer
REG_FIFO_CONFIG = 0x08		## Bits7:5 for Sample Averaging

REG_MODE_CONFIG = 0x09		# Control register
REG_SPO2_CONFIG = 0x0A		# Oximetry settings

REG_LED1_PA = 0x0C			# LED pulse amplitude
REG_LED2_PA = 0x0D
REG_PILOT_PA = 0x10         ## Not in datasheet?
REG_MULTI_LED_CTRL1 = 0x11  # Pulse width and power of LEDs
REG_MULTI_LED_CTRL2 = 0x12

REG_TEMP_INTR = 0x1F		# Temperature value, whole number (in two's complement)
REG_TEMP_FRAC = 0x20		# Temperature value, fraction
REG_TEMP_CONFIG = 0x21		# TEMP_EN (Temp. enable) Initiates single temp reading.

REG_PROX_INT_THRESH = 0x30
REG_REV_ID = 0xFE			# Part revision
REG_PART_ID = 0xFF			# Part ID, normally 0x11

SPO2_ADC_RGE = {
     2048: 0,
     4096: 1,
     8192: 2,
    16384: 3
}

SPO2_SR = {
    50: 0,
   100: 1,
   200: 2,
   400: 3,
   800: 4,
  1000: 5, 
  1600: 6,
  3200: 7
}

LED_PW = { # corresponding ADC resolution
     69: 0,     #   15             
    118: 1,     #   16
    215: 2,     #   17
    411: 3,     #   18
}

SMP_AVE = {
     1: 0,
     2: 1,
     4: 2,
     8: 3,
    16: 4,
    32: 5
}

class M02():
    ## by default, this assumes that the device is at 0x57 # on channel 1
    ## Ability to set_current_RED/IR changed between MAX30100 (p17) and MAX30102 (p20)
    def __init__(self,
                 # channel=1,                 
                 address=0x57, 
                 clock_pin=5, 
                 data_pin=4, 
                 frequency=400000,
                 
                 mode=0x03, #MODE_SP02,
                 
                 samples_per_second=100,
                 pulse_width = 411,
                 adc_range =4096,
                 
                 max_buffer_len=10000,
                 samples_per_avg= 4,
                 rollover= False,
                 FIFO_A_FULL = 17
                 ):
        
        ##print("Channel: {0}, address: {1}".format(channel, address))
        ## self.channel = channel
        self.bus = I2C(scl=Pin(clock_pin), sda=Pin(data_pin), freq=frequency)
        self.address = address 
        print("I2C connection made at address: {0}".format(self.address))
        
        self.reset()

        sleep(1)  # wait 1 sec

        # read & clear interrupt register (read 3 bytes)
        reg_data = self.bus.readfrom_mem(self.address, REG_INTR_STATUS_1, 3)
#        print("[SETUP] reset complete with interrupt register0: {0}".format(reg_data))
        
        self.set_mode(mode)             # Trigger an initial temperature read.
#        self.set_spo_config(samples_per_second, pulse_width)
        
        self.setup(SPA = samples_per_avg, 
                   RO = rollover,
                   FAF = FIFO_A_FULL,
                   led_mode=mode,
                   SPS = samples_per_second,
                   PW = pulse_width,
                   ADCR = adc_range
                   )
#        print("[SETUP] setup complete")

        self.mode = self.bus.readfrom_mem(self.address, REG_MODE_CONFIG,1)[0]

    def set_mode(self, mode):
    ### 0x02 for read-only, 0x03 for SpO2 mode, 0x07 multimode LED
        reg = self.bus.readfrom_mem(self.address, REG_MODE_CONFIG, 1)[0]
        self.bus.writeto_mem(self.address, REG_MODE_CONFIG, bytearray([reg & 0x74])) # mask the SHDN bit
        self.bus.writeto_mem(self.address, REG_MODE_CONFIG, bytearray([reg | mode])) 
        
        self.mode = self.bus.readfrom_mem(self.address, REG_MODE_CONFIG,1)[0]       

    def setup(self, SPA=4, RO=False, FAF=17, led_mode=3, SPS=100, PW=411, ADCR=4096):
        """
        This will setup the device with the values written in sample Arduino code.
        Note mode is MODE_SPO2 (0x03, 111) by default
        """
        # INTR setting
        # 0xc0 : A_FULL_EN and PPG_RDY_EN = Interrupt will be triggered when
        # fifo almost full & new fifo data ready
        self.bus.writeto_mem(self.address, REG_INTR_ENABLE_1, bytearray([0xc0]))
        self.bus.writeto_mem(self.address, REG_INTR_ENABLE_2, bytearray([0x00]))

        # REG_FIFO_WR_PTR[4:0]
        self.bus.writeto_mem(self.address, REG_FIFO_WR_PTR, bytearray([0x00]))
        # OVF_COUNTER[4:0]
        self.bus.writeto_mem(self.address, REG_OVF_COUNTER, bytearray([0x00]))
        # REG_FIFO_RD_PTR[4:0]
        self.bus.writeto_mem(self.address, REG_FIFO_RD_PTR, bytearray([0x00]))



        spa = _get_valid(SMP_AVE, SPA)            
        SRF_bits = spa<<5 | RO<<4 | 32-FAF
        # 0b 0100 1111: hex(79) = 0x4f: sam_avg=4, fifo rollover = false, fifo_a_full = 17        
        self.bus.writeto_mem(self.address, REG_FIFO_CONFIG, bytearray([SRF_bits]))


      ### 0x02 for read-only, 0x03 for SpO2 mode, 0x07 multimode LED
        self.bus.writeto_mem(self.address, REG_MODE_CONFIG, bytearray([led_mode]))


        adcr = _get_valid(SPO2_ADC_RGE, ADCR)       
        sps = _get_valid(SPO2_SR, SPS)
        pw = _get_valid(LED_PW, PW)
        ASP_bits = adcr<<5 | sps<<2 | pw        
        self.bus.writeto_mem(self.address, REG_SPO2_CONFIG, bytearray([ASP_bits]))
        ## 0b 0010 0111
        ## SPO2_ADC range = 4096nA, SPO2 sample rate = 100Hz, LED pulse-width = 411uS        
        ## self.bus.writeto_mem(self.address, REG_SPO2_CONFIG, bytearray([0x27]))        



        # choose value for ~7mA for LED1
        self.bus.writeto_mem(self.address, REG_LED1_PA, bytearray([0x24]))
        # choose value for ~7mA for LED2
        self.bus.writeto_mem(self.address, REG_LED2_PA, bytearray([0x24]))
        # choose value fro ~25mA for Pilot LED
        self.bus.writeto_mem(self.address, REG_PILOT_PA, bytearray([0x7f]))

    '''   
    def get_data_present(self):
       ## def get_number_of_samples(self):
       ##     write_ptr = self.bus.read_byte_data(self.address, REG_FIFO_WR_PTR)
       ##     read_ptr = self.bus.read_byte_data(self.address, REG_FIFO_RD_PTR)
       ##     return abs(16+write_ptr - read_ptr) % 16
        read_ptr = self.bus.readfrom_mem(self.address, REG_FIFO_RD_PTR, 1)[0]
        write_ptr = self.bus.readfrom_mem(self.address, REG_FIFO_WR_PTR, 1)[0]
        if read_ptr == write_ptr:
            return 0
        else:
            num_samples = write_ptr - read_ptr
            # account for pointer wrap around
            if num_samples < 0:
                num_samples += 32
            return num_samples
    '''
    def read_fifo(self):
        """    This function will read the data register.    """
        red_led = None
        ir_led = None

        # read 1 byte from registers (values are discarded)
        reg_INTR1 = self.bus.readfrom_mem(self.address, REG_INTR_STATUS_1, 1) 
        reg_INTR2 = self.bus.readfrom_mem(self.address, REG_INTR_STATUS_2, 1) 

        # read 6-byte data from the device
        d = self.bus.readfrom_mem(self.address, REG_FIFO_DATA, 6)

        #mask MSB [23:18]
        red_led = (d[0] << 16 | d[1] << 8 | d[2]) & 0x03FFFF
        ir_led = (d[3] << 16 | d[4] << 8 | d[5]) & 0x03FFFF

        return red_led, ir_led
    
    def read_d(self, byte_length):
        
        red_led = None
        ir_led = None
        
        reg_INTR1 = self.bus.readfrom_mem(self.address, REG_INTR_STATUS_1, 1) 
        reg_INTR2 = self.bus.readfrom_mem(self.address, REG_INTR_STATUS_2, 1) 

        # read byte_length long data from the device
        #d = self.bus.readfrom_mem(self.address, REG_FIFO_DATA, byte_length)
        
        return bytearray(d)
    
    def read_temp(self):         
        
        reg_INTR1 = self.bus.readfrom_mem(self.address, REG_INTR_STATUS_1, 1) 
        reg_INTR2 = self.bus.readfrom_mem(self.address, REG_INTR_STATUS_2, 1)
        
        reg = self.bus.readfrom_mem(self.address, REG_TEMP_CONFIG, 1)[0]        
        self.bus.writeto_mem(self.address, REG_TEMP_CONFIG, bytearray([reg | 0x01]))
        
        t_int = self.bus.readfrom_mem(self.address, REG_TEMP_INTR, 1) #[0:8]
        t_int = _twos_complement(t_int[0], 8)
        t_frac = self.bus.readfrom_mem(self.address, REG_TEMP_FRAC, 1) # [0:4]
        t_frac = 0.0625 * t_frac[0]
        
        return t_int + t_frac

    def shutdown(self):
        """         Shutdown the device.         """
        reg = self.bus.readfrom_mem(self.address, REG_MODE_CONFIG, 1)[0]
        self.bus.writeto_mem(self.address, REG_MODE_CONFIG, bytearray([reg | 0x80]))

    def reset(self):
        """         Resets device                """
        """ This clears all settings, so after running this, run setup() again. """
        reg = self.bus.readfrom_mem(self.address, REG_MODE_CONFIG, 1)[0]        
        self.bus.writeto_mem(self.address, REG_MODE_CONFIG, bytearray([reg | 0x40]))

        
    # this won't validate the arguments!
    # use when changing the values from default
    def set_config(self, reg, value):
        self.bus.writeto_mem(self.address, reg, value)     
        
    def get_config(self, reg, length=1):
        val = self.bus.readfrom_mem(self.address, reg, length)[0]
        return val
    

def _twos_complement(val, bits):
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)
    return val

def _get_valid(d, value):
    try:
        return d[value]
    except KeyError:
        raise KeyError("Value %s not valid, use one of: %s" % (value, ', '.join([str(s) for s in d.keys()])))   

'''
# currently not used
MAX_BRIGHTNESS = 255

# i2c address-es # not required?
I2C_WRITE_ADDR = 0xAE #?? #RF-master gives 0x57 for both of these
I2C_READ_ADDR = 0xAF

MODE_HR = 0x02
MODE_SPO2 = 0x03
MODE_MULTI_LED = 0x07

in init:         # self.set_spo_config(sample_rate, pulse_width)     

    def enable_interrupt(self, interrupt_type):
        self.bus.write_byte_data(self.address, INT_ENABLE, (interrupt_type + 1)<<4)
        self.bus.read_byte_data(self.address, INT_STATUS)

    def refresh_temperature(self):
        reg = self.bus.read_byte_data(self.address, REG_MODE_CONFIG)
        self.bus.write_byte_data(self.address, REG_MODE_CONFIG, reg | (1 << 3))

    def get_temperature(self):
        intg = _twos_complement(self.bus.read_byte_data(self.address, REG_TEMP_INTR))
        frac = self.bus.read_byte_data(self.address, REG_TEMP_FRAC)
        return intg + (frac * 0.0625)     
        
    def get_rev_id(self):
        return self.bus.read_byte_data(self.address, REG_REV_ID)

    def get_part_id(self):
        return self.bus.read_byte_data(self.address, REG_PART_ID)

    def get_registers(self):
        return {
            "INT_STATUS": self.bus.read_byte_data(self.address, INT_STATUS),
            "INT_ENABLE": self.bus.read_byte_data(self.address, INT_ENABLE),
            "REG_FIFO_WR_PTR": self.bus.read_byte_data(self.address, REG_FIFO_WR_PTR),
            "OVRFLOW_CTR": self.bus.read_byte_data(self.address, OVRFLOW_CTR),
            "REG_FIFO_RD_PTR": self.bus.read_byte_data(self.address, REG_FIFO_RD_PTR),
            "FIFO_DATA": self.bus.read_byte_data(self.address, FIFO_DATA),
            "REG_MODE_CONFIG": self.bus.read_byte_data(self.address, REG_MODE_CONFIG),
            "SPO2_CONFIG": self.bus.read_byte_data(self.address, SPO2_CONFIG),
            "LED_CONFIG": self.bus.read_byte_data(self.address, LED_CONFIG),
            "REG_TEMP_INTR": self.bus.read_byte_data(self.address, REG_TEMP_INTR),
            "REG_TEMP_FRAC": self.bus.read_byte_data(self.address, REG_TEMP_FRAC),
            "REG_REV_ID": self.bus.read_byte_data(self.address, REG_REV_ID),
            "REG_PART_ID": self.bus.read_byte_data(self.address, REG_PART_ID),
        }

    def set_spo_config(self, samples_per_second=1000, pulse_width=411): 
        reg = self.bus.readfrom_mem(self.address, REG_SPO2_CONFIG, 1)[0]
        reg = reg & 0xFC  # Set LED pulsewidth to 00
        pulse_width = _get_valid(PULSE_WIDTH, pulse_width)
        pw_input = reg | pulse_width
       # if ~isinstance(pulse_width, bytearray):
       #     pulse_width = bytearray(pulse_width)
        self.bus.writeto_mem(self.address, REG_SPO2_CONFIG, bytearray([pulse_width]))

    def read_sequential(self, amount=400):
        """
        This function will read the red-led and ir-led `amount` times.
        This works as blocking function.
        """
        red_buf = []
        ir_buf = []
        count = amount
        while count > 0:
            num_bytes = self.get_data_present()
            while num_bytes > 0:
                    red, ir = self.read_fifo()
                    
                    red_buf.append(red)
                    ir_buf.append(ir)

                    num_bytes -= 1
            count -= 1

        return red_buf, ir_buf

        
'''
