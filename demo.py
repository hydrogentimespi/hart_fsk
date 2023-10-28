#!/usr/bin/python


from hart_fsk import Hart_fsk

mydevice = Hart_fsk(verbosity = 1)

mydevice.read_ddl('demo.ddl')
print(f'ddl successfully read. manu {mydevice.manufacturer}, dd_rev {mydevice.dd_revision}, devtyp {mydevice.device_type}, devrev {mydevice.device_revision}, commands {len(mydevice.commands)}, variables {len(mydevice.variables)}' )


mydevice.openport()                                                 # serial port auto-detect
#mydevice.openport('/dev/ttyUSB0')
#mydevice.openport('com5')

mydevice.connect()                                                  # exec command 0, verify answer against edd, remember device ID
#mydevice.device_id = 1                                             # explicitly set device ID for instant long frame communication

#mydevice.ignoreRC = True                                           # don't abort if RC != 0; set this prior to command() call
#mydevice.poll_addr = 63											# change polling address for multi-drop mode

response = mydevice.read_primary_variable()                         # execute python hard-coded command
print("     response_code   ", response.response_code  )
print("     device_status   ", response.device_status  )
print("     pv_value        ", response.pv_value       )
print("     pv_unit         ", response.pv_unit        )

print(mydevice.read_loop_current_and_percent_of_range())            # execute python hard-coded command

print(" recent tx tries             :", mydevice.recent_tries)      # print communication meta data
print(" recent response time (ms)   :", mydevice.response_time)

print(mydevice.command('read_process_status'))                      # execute command defined by edd

print(mydevice.command_raw(9,  b'\x00\x01\x02\x03'))                # execute command with raw data

mydevice.closeport()

