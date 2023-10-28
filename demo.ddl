
MANUFACTURER 255, DEVICE_TYPE 12345, DEVICE_REVISION 1, DD_REVISION 1

VARIABLE process_mode       {TYPE UNSIGNED_INTEGER(1);}
VARIABLE unit_code          {TYPE UNSIGNED_INTEGER(1);}
VARIABLE process_value      {TYPE FLOAT;}
VARIABLE process_threshold  {TYPE FLOAT;}

COMMAND read_process_status
{
    NUMBER 130;
    OPERATION READ;
    TRANSACTION
    {
        REQUEST
        {
        }

        REPLY
        {
            response_code, device_status,
            process_mode, unit_code, process_value, process_threshold
        }
    }
    RESPONSE_CODES {
        0,  SUCCESS,                "Success";
    }
}

