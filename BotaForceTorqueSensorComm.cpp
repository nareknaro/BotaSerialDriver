#include "BotaForceTorqueSensorComm.h"

BotaForceTorqueSensorComm::BotaForceTorqueSensorComm()
{
  _synced = false;
}

uint16_t BotaForceTorqueSensorComm::crc16_mcrf4xx(uint8_t *data, size_t len)
{
    uint16_t crc = 0xffff;
    while (len--) {
        crc ^= *data++;
        for (int i=0; i<8; i++)
            crc = crc & 0x0001 ? (crc >> 1) ^ 0x8408 : crc >> 1;
    }
    return crc;
}

uint16_t BotaForceTorqueSensorComm::crc16_ccitt_false(uint8_t* data, size_t len)
{
    uint16_t crc = 0xffff;
    while (len--) {
        crc ^= *data++ << 8;
        for (int i=0; i < 8; i++)
            crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

uint16_t BotaForceTorqueSensorComm::crc16_x25(uint8_t* data, size_t len) // cyclic redundancy check (detect errors)
{
    uint16_t crc = 0xffff;
    while (len--) {
        crc ^= *data++ << 0;
        for (int i=0; i < 8; i++)
            crc = crc & 0x0001 ? (crc >> 1) ^ 0x8408 : crc >> 1;
    }
    return crc ^ 0xffff;
}

bool BotaForceTorqueSensorComm::isCrcOk() // check for errors
{
  if(crc16_x25(frame.data.bytes, sizeof(frame.data)) == frame.crc)
  {
    return true;
  }
  else
  {
    _crc_err_count += 1;
    return false;
  }
}

bool BotaForceTorqueSensorComm::checkSync()
{
  if (_synced)
  {
    _synced = (frame.header == 0xAA);
  }
  else
  {
    _crc_err_count = 0;
    _synced = (frame.header == 0xAA) & isCrcOk();
  }
  return _synced;
}

BotaForceTorqueSensorComm::ReadFrameRes BotaForceTorqueSensorComm::readFrame(int serial_port) // {VALID_DATA, NOT_VALID_DATA, NOT_ALLIGNED_DATA, NO_DATA}
{
  ReadFrameRes err = NO_FRAME;
  if(serialAvailable(serial_port)>=sizeof(frame))
  {
    serialReadBytes(serial_port,frame.bytes, sizeof(frame));
    if(checkSync())
    {
      if (isCrcOk())
      {
        err = VALID_FRAME;
      }
      else
      {
        err = NOT_VALID_FRAME;
      }
    }
    else
    {
      err = NOT_ALLIGNED_FRAME;
      //read one dummy byte to regain sync
      if (serialAvailable(serial_port))
      {
        uint8_t dummy;
        serialReadBytes(serial_port,&dummy, sizeof(dummy));
      }
    }
  }
  return err; //sucess
}
