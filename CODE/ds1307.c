#include "ds1307.h"
#include "i2c-soft.h"

unsigned char BcdToBin(unsigned char Bcd)
{
	return ( ( ( Bcd >> 4 ) * 10 ) + ( Bcd & 0x0F ) );
}

unsigned char BinToBcd(unsigned char Bin)
{
	return ( ( Bin / 10 ) * 0x10 ) + ( Bin % 10 );
}

unsigned char rtc_read(unsigned char address)
{
	unsigned char data;
	i2c_Start();
	i2c_Write(0xd0);
	i2c_Write(address);
	i2c_Start();
	i2c_Write(0xd1);
	data = i2c_Read( NO_I2C_ACK );
	i2c_Stop();
	return data;
}

void rtc_write( unsigned char address, unsigned char data )
{
	i2c_Start();
	i2c_Write(0xd0);
	i2c_Write(address);
	i2c_Write(data);
	i2c_Stop();
}

void rtc_init ( unsigned char rs, unsigned char sqwe, unsigned char out )
{
	rs&=3;
	if (sqwe) rs|=0x10;
	if (out) rs|=0x80;
	i2c_Start();
	i2c_Write(0xd0);
	i2c_Write(7);
	i2c_Write(rs);
	i2c_Stop();
}

void rtc_get_time( unsigned char *hour, unsigned char *min ,unsigned char *sec )
{
	i2c_Start();
	i2c_Write(0xd0);
	i2c_Write(0);
	i2c_Start();
	i2c_Write(0xd1);
	*sec=BcdToBin( i2c_Read( OK_I2C_ACK ) );
	*min=BcdToBin( i2c_Read( OK_I2C_ACK ) );
	*hour=BcdToBin( i2c_Read( NO_I2C_ACK ) );
	i2c_Stop();
}

void rtc_set_time( unsigned char hour, unsigned char min, unsigned char sec )
{
	i2c_Start();
	i2c_Write(0xd0);
	i2c_Write(0);
	i2c_Write( BinToBcd( sec ) );
	i2c_Write( BinToBcd( min ) );
	i2c_Write( BinToBcd( hour ) );
	i2c_Stop();
}

void rtc_get_date( unsigned char *date, unsigned char *month, unsigned char *year )
{
	i2c_Start();
	i2c_Write(0xd0);
	i2c_Write(4);
	i2c_Start();
	i2c_Write(0xd1);
	*date=BcdToBin( i2c_Read( OK_I2C_ACK ) );
	*month=BcdToBin( i2c_Read( OK_I2C_ACK ) );
	*year=BcdToBin( i2c_Read( NO_I2C_ACK ) );
	i2c_Stop();
}

void rtc_set_date( unsigned char date, unsigned char month, unsigned char year )
{
	i2c_Start();
	i2c_Write(0xd0);
	i2c_Write(4);
	i2c_Write( BinToBcd( date ) );
	i2c_Write( BinToBcd( month ) );
	i2c_Write( BinToBcd( year ) );
	i2c_Stop();
}
