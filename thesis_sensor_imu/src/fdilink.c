#include "fdilink.h"
#include "string.h"
 
extern uint8_t CRC8_Table(uint8_t* p, uint8_t counter);
extern uint16_t CRC16_Table(uint8_t* p, uint8_t counter);

void FDILink_Error(FDILink_t* FDILink)
{
	FDILink->RxLost++;
	FDILink->RxStatus = FDILink_Frame_Start;
}
void FDILink_Insert(FDILink_t* FDILink, uint8_t value)
{
	if (FDILink->RxDataLeft <= 0)
	{
		FDILink_Error(FDILink);
		return;
	}
	
	if (FDILink->RxStatus != FDILink_Frame_Data)
	{
		FDILink_Error(FDILink);
		return;
	}
	
	FDILink->Buffer[FDILink->BufferIndex++] = value;  //将元素放入队列尾部
	if (FDILink->BufferIndex >= 256)
	{
		FDILink_Error(FDILink);
		return;
	}
	
	FDILink->RxDataLeft--;
}
void FDILink_Reset(FDILink_t* FDILink)
{
	FDILink->RxStatus = FDILink_Frame_Start;
	FDILink->RxDataLeft = 0;
	FDILink->RxType = 0;
	FDILink->BufferIndex = 0;
	FDILink->TxNumber = 0;
}
int FDILink_RunningData(FDILink_t* FDILink, uint8_t value)
{
	if (FDILink->RxStatus < FDILink_Frame_Start || FDILink->RxStatus > FDILink_Frame_End)
	{
		FDILink_Error(FDILink);
		return -3;
	}
	FDILink->FDILink_Frame_Buffer[FDILink->RxStatus] = value;
	switch (FDILink->RxStatus)
	{
		case FDILink_Frame_Start:
			FDILink_Reset(FDILink);
			if (value == FDILink_Connect_Flag)
			{
				return 0;
			}
			if (value != FDILink_STX_Flag)
			{
				FDILink_Error(FDILink);
				return -1;
			}
			FDILink->RxStatus = FDILink_Frame_CMD;
			break;
		case FDILink_Frame_CMD:
			FDILink->RxType = value;
			FDILink->RxStatus = FDILink_Frame_Length;
			break;
		case FDILink_Frame_Length:
			FDILink->RxDataLeft = value;
			FDILink->RxStatus = FDILink_Frame_SerialNumber;
			break;
		case FDILink_Frame_SerialNumber:
			FDILink->RxNumber = value;
			FDILink->RxStatus = FDILink_Frame_CRC8;
			break;
		case FDILink_Frame_CRC8:
			FDILink->CRC8_Verify = value;
//			if (CRC8_Table(FDILink->FDILink_Frame_Buffer, FDILink_Frame_CRC8) != FDILink->CRC8_Verify)
//			{
//				FDILink_Error(FDILink);
//				return -1;
//			}
			if(FDILink->RxDataLeft == 0)
			{
				FDILink->RxStatus = FDILink_Frame_Start;
				FDILink->RxOKBytes += 5;
				return 1;
			}
			FDILink->RxStatus = FDILink_Frame_CRC16H;
			break;
		case FDILink_Frame_CRC16H:
			FDILink->CRC16_Verify = value;
			FDILink->RxStatus = FDILink_Frame_CRC16L;
			break;
		case FDILink_Frame_CRC16L:
			FDILink->CRC16_Verify = (FDILink->CRC16_Verify << 8) | value;
			FDILink->RxStatus = FDILink_Frame_Data;
			break;
		case FDILink_Frame_Data:
			if (FDILink->RxDataLeft)
			{
				FDILink_Insert(FDILink,value);
				if (FDILink->RxDataLeft == 0)
				{
					FDILink->RxStatus = FDILink_Frame_End;
				}
				break;
			}
			else
			{
				FDILink->RxStatus = FDILink_Frame_End;
			}
		case FDILink_Frame_End:
		{
			if (value != FDILink_EDX_Flag)
			{
				FDILink_Error(FDILink);
				return -1;
			}
//			uint16_t CRC16 = CRC16_Table(FDILink->Buffer, FDILink->BufferIndex);
//			if (CRC16 != FDILink->CRC16_Verify)
//			{
//				FDILink_Error(FDILink);
//				return -1;
//			}
			FDILink->RxStatus = FDILink_Frame_Start;
			FDILink->RxOKBytes += FDILink->BufferIndex + 8;
			return 1;
		}
		default:
			FDILink_Error(FDILink);
			return -1;
	}
	return 0;
}
int FDILink_CheckData(FDILink_t* FDILink, uint8_t len)
{
	if (FDILink->BufferIndex != len)
	{
		FDILink_Error(FDILink);
		return -1;
	}
	return 0;
}


int FDILink_Receive(FDILink_t* FDILink, uint8_t value)
{
	if(FDILink->BootStatus != FDILink_Status_Running)
	{
		FDILink->BufferIndex = 0;
		FDILink->BootStatus = FDILink_Status_Running;
		FDILink->RxStatus = FDILink_Frame_Start;
		for (int i = 0; i < 256; i++)
		{
			FDILink->Buffer[i] = 0;
		}
	}
	FDILink->RxTotalBytes++;
	return FDILink_RunningData(FDILink, value);
}





