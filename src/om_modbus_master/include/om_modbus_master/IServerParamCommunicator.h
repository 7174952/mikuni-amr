#ifndef ICOMMUNICATE_PARAMETER_SERVER_H
#define ICOMMUNICATE_PARAMETER_SERVER_H

namespace om_modbusRTU_node
{

/*---------------------------------------------------------------------------*/
/**
@brief BaseクラスからSecondGenModbusRTUにサーバーパラメータを渡すインターフェース
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
class IServerParamCommunicator
{

public:
  virtual void setAxisNum(const int axis_num)=0;
  virtual int getAxisNum()=0;
  virtual void setGlobalId(const int global_id)=0;
  virtual int getGlobalId()=0;
  virtual ~IServerParamCommunicator(){};
};

}

#endif