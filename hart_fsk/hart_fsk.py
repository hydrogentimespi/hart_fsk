import serial
import serial.tools.list_ports
import time
from .eddly import Eddly
import struct
from collections import namedtuple
from datetime import datetime

class Hart_fsk(Eddly):

    #SHORTFRAME, LONGFRAME = (0, 1)
    #shortAdr        = 0
    #longAdr         = 0
    retries         = 3
    recent_tries    = 0
    preambles_tx    = 3
    device_id       = 0
    poll_addr       = 0
    ignoreRC        = False     # enable to accept commands with non-zero response code

    port            = None
    isOpen          = False
    #isConnected     = False
    send_timestamp  = 0

    verbose = 1

    def __init__(self, verbosity = 0):
        super().__init__()
        self.verbose = verbosity

    def openport(self, serialdevice = None):
        if serialdevice == None:
            vendor_product_string='VID:PID=23A1:010D'       # Mactek Viator USB HART modem with FTDI chip set
            print("Serial port wasn't specified. Searching devices...")
            ports = serial.tools.list_ports.comports()
            for serialdevice, desc, hwid in sorted(ports):
                #print("#### {}: {} [{}]".format(port, desc, hwid))
                if vendor_product_string in hwid:
                    print("found serial port", serialdevice)
                    break
                else:
                    serialdevice = None
            if serialdevice == None:
                raise Exception(f'No serial device with {vendor_product_string} found')

        self.port = serial.Serial(serialdevice,
                        baudrate    = 1200,
                        bytesize    = serial.EIGHTBITS,
                        parity      = serial.PARITY_ODD,
                        stopbits    = serial.STOPBITS_ONE,
                        write_timeout = 1.0,
                        timeout     = 1.0)
        self.isOpen = True

    def closeport(self):
        self.device_id = 0
        self.isOpen = False
        self.port.close()

    def send_frame(self, islongframe: bool, poll_addr: int, manu_id: int, devtype: int, devid: int, command: int, data: bytearray = [], master_addr = 1):
        """
        Implementes the HART frame layout, assembles the HART frame and sends it over the serial port.
        Handles preambles, extended commands, check sum byte, time stamping.
        If short frame, poll_addr ist used, If long frame, manu_id, devtype, devid is used.
        """
        assert self.isOpen == True, f'serial port not open'
        frame = bytearray()
        for i in range (0, self.preambles_tx):
            frame.extend([0xFF])
        # Delimiter
        frame.extend([  islongframe << 7 |
                        1 << 1                      # master to slave
                    ])
        if islongframe:
            # Long frame address
            frame.extend([  master_addr << 7 |
                            0 << 6 |                # slave in burst mode
                            manu_id & 0x3F          # 6 lsb of manufacturer id
                        ])
            frame.extend([devtype])
            frame.extend(devid.to_bytes(3,'big'))
        else:
            # Short frame address
            frame.extend([  master_addr << 7 |
                            0 << 6 |                # slave in burst mode
                            poll_addr & 0x3F        # 6 bit polling address
                        ])
        # Command
        if (islongframe and (command == 0)):
            raise Exception("only short frame allowed for cmd 0")
        if command > 0xFF:
            # extended commands
            frame.extend([31])
            # Byte count
            frame.extend([len(data)+2])
            frame.extend([command >> 8])
            frame.extend([command & 0xff])
        else:
            frame.extend([command])
            # Byte count
            frame.extend([len(data)])
        # data payload 
        for i in range(len(data)):
            frame.extend([data[i]])
        # xor check byte
        checkbyte=0
        for i in range(self.preambles_tx, len(frame)):
            checkbyte = checkbyte^frame[i]
        #checkbyte = checkbyte + 1       # provoke checksum error
        frame.extend([checkbyte])
        #frame.extend([0x00])            # AppendedBytes?

        #frame=bytes.fromhex("FFFFFFFFFFFFFFFFFFFF0280000082")      # testing cmd #0
        if self.verbose:
            print("send_frame()   : " + frame.hex())
        self.port.write(frame)
        self.send_timestamp = time.time()
        self.port.reset_input_buffer()  


    def receive_frame(self, islongframe = True):
        """
        Receive raw HART frame from serial port.
        Consume preambles, read bytes from serial port according to delimiter, byte count.
        Verify check sum, return frame to caller.
        """
        assert self.isOpen == True, f'serial port not open'
        status=0
        
        # consume all characters up to the last 0xFF
        self.port.timeout = 1.0     #0.250       # STO 256ms
        preambles_seen = 0
        while 1:
            frame = self.port.read(1)
            if len(frame) == 0:
                raise TimeoutError
            if frame[0] == 0xFF:
                preambles_seen += 1

                # STO ends with 1st preamble. Character time = 1/1200 * 11 = 9.166ms. STO = 28 x char time = 257 ms
                # Todo: throw warning/error if STO exceeded
                self.response_time = int((time.time() - self.send_timestamp)*1000)
            if not frame[0] == 0xFF:
                if preambles_seen > 0:      # > 2
                    break

        # Delimiter
        if ((frame[0] & 0x80) != 0) is not (islongframe == True):
            raise Exception("unexpected frame format (short/long) received")
        if not frame[0] & 0x7F == 0b00000110:
            raise Exception("not slave-to-master frame")
            
        if islongframe:
            # Long frame address
            buf = self.port.read(5)
            if len(buf) == 0:
                raise TimeoutError
            if not buf[0] & 0b11000000 == 0b10000000:
                raise Exception("only primary master. non-burst frames supported")
            devtype = buf[1]
            devid   = int.from_bytes(buf[2:4],'big')
        else:
            # Short frame address
            buf = self.port.read(1)
            if len(buf) == 0:
                raise TimeoutError
            if not buf[0] & 0b11000000 == 0b10000000:
                raise Exception("only primary master. non-burst frames supported")
            mfgid_polladdr = buf[0] & 0x0F       # 4 bit polling address
        frame = frame + buf
        
        # Command
        command = self.port.read(1)
        if len(command) == 0:
            raise TimeoutError
        frame = frame + command

        # Byte Count
        byte_count = self.port.read(1)
        if len(byte_count) == 0:
            raise TimeoutError
        frame = frame + byte_count
        
        # data payload
        data = self.port.read(byte_count[0])
        if len(data) == 0:
            raise TimeoutError
        frame = frame + data

        # check byte
        checkbyte_rx = self.port.read(1)
        if len(checkbyte_rx) == 0:
            raise TimeoutError
        frame = frame + checkbyte_rx

        if self.verbose:
            print("receive_frame(): " + frame.hex() + " preambles_rx:", preambles_seen)

        # throw away additional data - if any
        self.port.reset_input_buffer()
            
        checkbyte=0
        for i in range(0, len(frame)):
            checkbyte = checkbyte^frame[i]
        if 0 != checkbyte:
            raise Exception("rx checksum wrong", checkbyte_rx, checkbyte)

        if command == b'\x1f':
            command = int.from_bytes(data[2:4], byteorder='big')        # TODO return to the caller?
            data = data[0:2] + data[4:]         # extended cmd num between rc,dc and data

        return (status, data)


    def command_raw(self, command: int, data: bytearray = []):
        """
        send a HART command by command number with bytearray payload.
        Return tuple (response_code, device_status, data).
        Fail if RC != 0, unless ignoreRC
        """
        if self.verbose:
            print("cmd",command, data)

        if command == 0:    # only cmd 0 may be a short frame
            islongframe     = False
        else:
            if self.device_id == 0:
                raise Exception("device_id not set. Please call connect() first or manually set device_id for long frame communication.")
            islongframe     = True

        self.recent_tries = 1
        while 1:
            try:
                self.send_frame(islongframe, self.poll_addr, self.manufacturer, self.device_type, self.device_id, command, data)
                status, payload = self.receive_frame(islongframe)
                break
            except TimeoutError:
                if self.recent_tries <= self.retries:
                    self.recent_tries += 1
                else:
                    raise Exception("command() execution failed after", self.recent_tries, "tries")
        response_code = payload[0]
        device_status = payload[1]
        if self.verbose:
            print(f'cmd={command}, rc={hex(response_code)}, ds={hex(device_status)} tries={self.recent_tries}, response time: {self.response_time}ms')

        if self.ignoreRC == False:
            if response_code != 0:
                raise Exception(f'command {command} failed')

        return (payload[0], payload[1], payload[2:])


    def connect(self):
        """ send HART cmd 0 to unknown device, get and remember unique address, verify response against edd"""
        assert self.isOpen == True, f'serial port not open'

        (response_code, device_status, data) = self.command_raw(0)
        
        assert data[0] == 0xFE, f'wrong magic in cmd0, data[0]'
        manufacturer_id                               = data[1]
        device_type                                   = data[2]
        number_request_preambles                      = data[3]
        hart_protocol_major_revision                  = data[4]
        software_revision_level                       = data[6]
        hardware_revision_level                       = data[7]>>3
        device_id                                     = int.from_bytes(data[9:12], "big")
        config_change_counter                         = int.from_bytes(data[14:16], "big")
        extended_field_device_stats                   = data[16]

        if (manufacturer_id != self.manufacturer) or (device_type != self.device_type):
            print(f'WARNING: edd wrong for connected device. manufacturer/devicetype edd:{self.manufacturer}/{self.device_type}, device: {manufacturer_id}/{device_type}')
            self.manufacturer       = manufacturer_id
            self.device_type        = device_type
        else:
            assert software_revision_level >= self.device_revision,  f'device sw revision ({software_revision_level}) needs to be at least edd revision {self.device_revision}'

        self.preambles_tx = number_request_preambles
        assert hart_protocol_major_revision == 7, f'HART protocol version mismatch. required: 7, device: {hart_protocol_major_revision}'
        self.device_id  = device_id



        print(f'connected. device id={device_id} HWrev={hardware_revision_level}  SWrev={software_revision_level} extended_field_device_stats={extended_field_device_stats}')

    def float2HART(self, f: float) -> bytearray:
        return struct.pack('>f', f)

    def HART2float(self, b: bytearray) -> float:
        return struct.unpack('>f', b)[0]

    def command(self, cmd_name_or_number, *arg):
        if type(cmd_name_or_number) is int:
            found_cmd_name = ''
            for key in self.commands:
                if self.commands[key]['NUMBER'] == cmd_name_or_number:
                    found_cmd_name = key
            assert found_cmd_name != '', f'command with number {cmd_name_or_number} not in edd'
            cmd_name = found_cmd_name
        else:
            assert cmd_name_or_number in self.commands, f'command {cmd_name_or_number} not in edd'
            cmd_name = cmd_name_or_number

        cmd_dict = self.commands[cmd_name]
        cmdnum = self.commands[cmd_name]['NUMBER']

        if self.verbose:
            print ("\ncmd_name", cmd_name, "found. Number", cmdnum)

        # TODO transactions, OPERATION ?;
        param_count_edd     = len(cmd_dict['TRANSACTIONS'][0]['REQUEST'])
        param_count_call    = len(arg)
        assert param_count_edd == param_count_call, f'command {cmd_name} takes {param_count_edd} parameters, but {param_count_call} were given.'

        data = self.varlist2bytearray(cmd_dict['TRANSACTIONS'][0]['REQUEST'], arg)


        (response_code, device_status, data) = self.command_raw(cmdnum, data)       # execute command


        assert response_code in cmd_dict['RESPONSE_CODES'], f'received response code {response_code} for command {cmd_name} undefined by edd'

        idx_from = 0
        field_names = ['response_code', 'device_status']
        field_values = [response_code, device_status]
        for (var_name, mask) in cmd_dict['TRANSACTIONS'][0]['REPLY'][2:]:        # skip response_code, device_status
            fmt = self.variables[var_name]['FORMAT']
            size = self.get_variable_size(var_name)
            idx_to = size + idx_from
            if fmt=='UNSIGNED_INTEGER' or fmt=='BIT_ENUMERATED' or fmt=='ENUMERATED' or fmt=='INDEX'or fmt=='OCTET':
                var = int.from_bytes(data[idx_from:idx_to], 'big')
                # TODO: ...something useful with mask
            elif fmt=='INTEGER':
                var = int.from_bytes(data[idx_from:idx_to], 'big', signed=True)
            elif fmt=='FLOAT':
                var = self.HART2float(data[idx_from:idx_to])
            elif fmt=='DATE':
                day     = data[idx_from + 0]
                month   = data[idx_from + 1]
                year    = data[idx_from + 2] + 1900
                pt = datetime(year, month, day)
                var = pt.strftime(('%d.%m.%Y'))
            elif fmt=='ASCII':
                var = str(data[idx_from:idx_to].decode('latin-1')).strip('\x00')
            else:   #elif fmt=='DOUBLE' or fmt=='TIME' or fmt=='TIME_VALUE':
                raise("variable format unsupported")     # TODO
            idx_from = idx_to
            field_names.append(var_name)
            field_values.append(var)

        assert len(data) == idx_to, f'cmd rx length mismatch. bytes expected: {len(data)} bytes received: {idx_to}'
        Data = namedtuple('Data', ' '.join(field_names))
        return Data(*field_values)



    def varlist2bytearray(self, varlist, arg) -> bytearray:
        data = bytearray()
        for i in range(len(varlist)):
            data.extend(self.var2bytearray(varlist[i], arg[i]))
        return data

    def var2bytearray(self, var: str, value) -> bytearray:
        assert var in self.variables, f'no such variable {var}.'
        ret = bytearray()

        fmt = self.variables[var]['FORMAT']
        size = self.get_variable_size(var)
        if fmt=='UNSIGNED_INTEGER'or fmt=='BIT_ENUMERATED'or fmt=='ENUMERATED' or fmt=='INDEX'or fmt=='OCTET':
            return value.to_bytes(size, 'big')
        if fmt=='INTEGER':
            return value.to_bytes(size, byteorder='big', signed=True)
        elif fmt=='FLOAT':
            return self.float2HART(value)
        elif fmt=='DATE':
            pt = datetime.strptime(value, '%d.%m.%Y')
            ret = bytearray()
            ret.append(pt.day)
            ret.append(pt.month)
            ret.append(pt.year-1900)
            return ret
        elif fmt=='ASCII':
            string_bytearray = value.encode('latin-1')
            assert len(string_bytearray) <= size, f'the string length of variable {var} must not exceed {size} characters'
            return string_bytearray + bytearray(size - len(string_bytearray))
        else:   #elif fmt=='DOUBLE' or fmt=='TIME' or fmt=='TIME_VALUE':
            raise("variable format unsupported")     # TODO

    def bytearray2varlist(self, bytearray):
        data = bytearray()
        for i in range(len(varlist)):
            data.extend(self.var2bytearray(varlist[i], arg[i]))
        return data


#################### hard-coded commands ###################
# universal
    def read_primary_variable(self):
        (response_code, device_status, data) = self.command_raw(1)
        Data = namedtuple('Data', 'response_code device_status pv_unit pv_value')
        return Data(response_code, device_status, data[0], self.HART2float(data[1:5]))

    def read_loop_current_and_percent_of_range(self):
        (response_code, device_status, rawdata) = self.command_raw(2)
        Data = namedtuple('Data', 'response_code device_status loop_current percent_range')
        return Data(response_code,
                    device_status,
                    self.HART2float(rawdata[0:4]),
                    self.HART2float(rawdata[4:8]))

    def write_polling_address(self, poll_addr: int, loop_current_mode: int):
        data = bytearray()
        data.append(poll_addr)
        data.append(loop_current_mode)
        (response_code, device_status, rawdata) = self.command_raw(6, data)
        return response_code, device_status, rawdata

    def read_condensed_status_mapping_array(self):
        (response_code, device_status, rawdata) = self.command_raw(523, b'\x00\x58')
        Data = namedtuple('Data', 'start_index num_entries status_map_code_1 status_map_code_2 status_map_code_3 status_map_code_4')
        data = Data(rawdata[0],rawdata[1], 
                    rawdata[2] & 0x0f,
                    rawdata[2] >> 4,
                    rawdata[3] & 0x0f,
                    rawdata[3] >> 4)
        return response_code, device_status, data

